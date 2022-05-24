# ----------------------------------------------------------------------------------------
# Code to interface with custom Nistec ALW501 controller.
#
# Author: Warren Creemers
# See run.py to run robot.
# See license.txt for license info.
# ----------------------------------------------------------------------------------------
import datetime
import os
import sys
import threading
from typing import NamedTuple, List
import serial
from slugify import slugify
from statemachine import StateMachine, State
import time
import camera
import cd_drive
import logging


# add a logging level for the robot comms
# def setup_logging():
# logging.COMMS = 25
# logging.addLevelName(logging.COMMS, "COMMS")  # after Warning, before info
#
# def _comms(self, message, *args, **kws):
#     if self.isEnabledFor(logging.COMMS):
#         self._log(logging.COMMS, message, args, **kws)   # deliberate *args -> args
#
# logging.Logger.comms = _comms
#
# def _comms_helper(msg, *args, **kwargs):
#     if len(logging.root.handlers) == 0:
#         logging.basicConfig()
#     logging.root.comms(msg, *args, **kwargs)
#
# logging.basicConfig(level=logging.COMMS)
# logging.comms = _comms_helper
logging.comms = logging.info

def mills():
    return int(round(time.time() * 1000))


def cmd2bytes(s: str):
    return f"{s}\r".encode()


class RobotFailedException(Exception):
    pass


class PinState(NamedTuple):
    pin: int
    state: int
    transitions: int

    def high(self):
        return self.state != 0

    def low(self):
        return self.state == 0


def parse_param_line(line: str):
    # example:
    # tag: pin=40 state=1, transitions=4

    # get tag
    # logging.info(f"parsing: {line}")
    tag = None
    if ':' in line:
        tag, raw_params = [l.strip() for l in line.split(':', maxsplit=1)]
    else:
        raw_params = line

    # reduce param key/value pair
    if '=' in raw_params:
        raw_params = raw_params.replace(',', ' ')
        raw_params = raw_params.replace(' =', '=')
        raw_params = raw_params.replace('= ', '=')
        params = [p.strip() for p in raw_params.split(' ')]
        params = [p for p in params if '=' in p]
        # params = [[[n.strip(), v.strip()] for n, v in p.split('=', maxsplit=1)] for p in params]
        p_dict = {}
        for p in params:
            n, v = p.split('=', maxsplit=1)
            p_dict[n.strip()] = v.strip()
        # print(f"  - parsed {p_dict}")
        return tag, p_dict
    else:
        return False, {}

class CDRobot(StateMachine):

    # ------------------------------------------------------------------------------------------------------------------
    # States
    unknown = State('unknown', initial=True)  # robot was turned on, needs to be moved to home position.
    home = State('home')    # robot arm home and drive closed
    ready = State('ready')  # disk is available for processing
    error = State('error')  # something went wrong, robot was stopped.

    loaded_disk = State('loaded_disk')            # disk in drive
    reading_disk = State('reading_disk')
    ejected_disk = State('ejected_disk')
    ejected_bad_disk = State('ejected_bad_disk')

    removed_disk = State('removed_disk')
    removed_bad_disk = State('removed_bad_disk')

    # ------------------------------------------------------------------------------------------------------------------
    # State transitions
    init_arm = unknown.to(home)
    disk_available = home.to(ready)

    load_disk = ready.to(loaded_disk)
    begin_read_disk = loaded_disk.to(reading_disk)
    finish_read_disk = reading_disk.to(ejected_disk)
    error_read_disk = reading_disk.to(ejected_bad_disk)

    unload_disk = ejected_disk.to(removed_disk)
    unload_bad_disk = ejected_disk.to(removed_bad_disk)

    cycle_back_to_home = removed_disk.to(home) | removed_bad_disk.to(home)

    fault = unknown.to(error) | home.to(error) | ready.to(error)  \
            | loaded_disk.to(error)  | reading_disk.to(error)     \
            | ejected_disk.to(error) | ejected_bad_disk.to(error) \
            | removed_disk.to(error) | removed_bad_disk.to(error)

    # ------------------------------------------------------------------------------------------------------------------
    cd_storage_sensor_pin: int = 32
    left_sensor_pin: int = 44
    middle_sensor_pin: int = 39
    right_sensor_pin: int = 41
    cd_gripper_pin: int = 40

    # ------------------------------------------------------------------------------------------------------------------
    def __init__(self, port: str, drive: str, iso_folder="c:\\share\\cd_robot", cam_num=-1):
        super().__init__()

        print("-" * 60)
        # connect to com port
        try:
            print(f"  - setting up robot on {port}")
            self.ser = serial.Serial(port, 9600, timeout=0.5)
        except serial.serialutil.SerialException as ex:
            raise RobotFailedException(f"Error opening serial port {str(ex)}.")

        # Check the device transmitted ready upon connection.
        ok, lines = self._get_robot_feedback(10000)
        if not ok or not ('ready' in ''.join(lines).lower()):
            raise RobotFailedException("Device did not appear to initialise correctly.")

        self.last_known_arm_pos = 'u'

        # Call functions to update self.last_known_arm_pos if a sensor can give us an answer.
        self.is_arm_middle()
        self.is_arm_left()
        self.is_arm_right()

        self.drop_distance = 6_400  # to refine this (eg new drive) use: run --mode calibrate_load
        self.iso_folder = iso_folder

        self.cd_drive = drive
        if self.cd_drive is not None:
            # cd_drive.close_cd_drive(self.cd_drive)
            if cd_drive.is_disk_in_drive(self.cd_drive):
                cd_drive.open_cd_drive(self.cd_drive)
                print("Found a disk in the drive during startup.")
                print("Remove and restart software.")
                raise ValueError("Disk in drive on startup.")

        self.cam = camera.get_camera(cam_num)
        self.camera_img = camera.get_image(self.cam)

        self.dry_run = False  # tweak for debug (will skip making an iso file)
        print("-" * 60)

        self.last_disk_ok = True

    # ------------------------------------------------------------------------------------------------------------------
    def main_loop(self):
        self.init_arm()

        print("# Starting robot")
        while True:

            print("## Waiting for disks.")
            while not self.is_disk_in_stroage():
                time.sleep(2)
                # if self.is_disk_in_stroage():

            print("New disks found")
            self._do_new_disk_set_pause(5)
            self.disk_available()  # transition state

            # now ready
            self.load_disk()
            self.begin_read_disk()
            self.finish_read_disk()
            self.unload_disk()
            self.cycle_back_to_home()

    # ------------------------------------------------------------------------------------------------------------------
    def __str__(self):
        return f"CD Robot ({self.ser.port}, arm_at={self.last_known_arm_pos})"

    # ------------------------------------------------------------------------------------------------------------------
    # Serial comm's and parsing.
    # ------------------------------------------------------------------------------------------------------------------
    def _serial_write_line(self, s: str):
        # print(f"    sending cmd {s}")
        self.ser.write(cmd2bytes(s))

    def _serial_readline(self) -> str:
        s = self.ser.readline().decode("utf-8")
        # print(f"read: {s}")
        return s

    def who(self, pin=None) -> List[PinState]:
        # clear the buffer
        self._get_robot_feedback(timeout=0.25, require_pass_fail=False)
        if pin is None:
            lines = self.do_command("who", 2000)
        else:
            lines = self.do_command(f"who {int(pin)}", 2000)
        return self._scan_pins(lines)

    def _scan_pins(self, lines: List[str]) -> List[PinState]:
        #  pin=40 state=1, transitions=4

        pins_states = []
        for line in lines:
            _, params = parse_param_line(line)
            if 'pin' in params and 'state' in params and 'transitions' in params:
                state = PinState(pin=int(params['pin']),
                                 state=int(params['state']),
                                 transitions=int(params['transitions']))
                pins_states.append(state)

        return pins_states

    def get_pin(self, pin: int) -> PinState:
        for i in range(3): # allow for a few retries.
            states = self.who(pin=pin)
            if len(states) == 1:
                return states[0]
            time.sleep(0.5)
        raise RobotFailedException(f"Invalid response for pin {pin}: {states}")


    def list_pins(self, old_pins={}):
        pins = {
            "cd_storage_sensor_pin": self.cd_storage_sensor_pin,
            "left_sensor_pin": self.left_sensor_pin,
            "middle_sensor_pin": self.middle_sensor_pin,
            "right_sensor_pin": self.right_sensor_pin,
            "cd_gripper_pin": self.cd_gripper_pin,
        }

        pin_values = {p: (n, self.get_pin(p)) for n, p in pins.items()}
        changed_pins = {p: (n, r) for p, (n, r) in pin_values.items() if (p not in old_pins) or old_pins[p].state != r.state}

        if len(changed_pins) > 0:
            print("Robot pins:")
            for pin, (name, state) in changed_pins.items():
                print(f"  - {str(name).ljust(15)} ({pin}): {state.state}      {state}")
            print()

        return {p: s for p, (n, s) in pin_values.items()}


    def _get_robot_feedback(self, timeout: int, require_pass_fail=True):

        def tidy(line_raw: str) -> str:
            new_line = line_raw.strip()
            new_line = new_line[1:].strip() if new_line.startswith('>') else new_line
            return new_line

        start = mills()
        lines = []
        while mills() < (start+timeout):
            line = tidy(self._serial_readline())
            if line.lower() == '_ok_':
                return True, lines
            elif line.lower() == '_fail_':
                return False, lines
            else:
                lines.append(line)

        if require_pass_fail:
            print("Error")
            raise RobotFailedException("Robot did not send a pass/fail response after taking an action.")

        return True, lines

    def _get_var(self, lines: [], var: str):
        result_lines = [q for q in lines if q.startswith('info:') and f' {var}=' in q]
        if len(result_lines) > 1:
            raise RobotFailedException(f"robot gave multiple results for {var}.")
        if len(result_lines) == 0:
            raise RobotFailedException(f"robot gave no results for {var}.")
        return result_lines[0].split('=')[1]


    # ------------------------------------------------------------------------------------------------------------------
    # User feedback, logging
    # ------------------------------------------------------------------------------------------------------------------

    exit_thread = threading.Event()
    def _do_new_disk_set_pause(self, count_down: int = 10):
        print("New disk loading detected, pausing to allow manual operation.")
        for i in range(0, count_down):
            print(f"{count_down - i}.. ", end='', flush=True)
            # time.sleep(1)
            self.exit_thread.wait(1)
        print("Ready or not, starting up..")

    # ------------------------------------------------------------------------------------------------------------------
    # Motor & Robot control
    # ------------------------------------------------------------------------------------------------------------------
    def do_command(self, cmd: str, time_out: int) -> List[str]:
        self._serial_write_line(cmd)
        try:
            logging.comms("        Doing command: ", cmd)
            ok, lines = self._get_robot_feedback(time_out)
            for line in lines:
                logging.comms(f"          > {line}")
            logging.comms("")
            if not ok:
                logging.error("Command failed:")
                for line in lines:
                    logging.error(f"          > {line}")
                raise RobotFailedException(f"Command {cmd} returned false")
            return lines
        except RobotFailedException as ex:
            raise RobotFailedException(f"error running command {cmd}: {str(ex)}")

    def let_motor_stop_spinning(self):
        time.sleep(0.5)

    def jog_up(self, distance: int = -1):
        logging.comms(f"  jog_up({distance})")
        if distance > 0:
            if self.set_vert_jog_dist(distance):
                lines = self.do_command("up", 5000)
                self.let_motor_stop_spinning()


    def jog_down(self, distance: int = -1):
        logging.comms(f"  jog_down({distance})")
        if distance > 0:
            if self.set_vert_jog_dist(distance):
                lines = self.do_command("down", 5000)
                self.let_motor_stop_spinning()


    def move_to_right(self):
        logging.comms(f"  move_to_right()")
        if self.is_arm_right():
            print("  - arm already right")
            return
        self.do_command("full_right", 8000)
        self.let_motor_stop_spinning()
        if not self.is_arm_right():
            raise RobotFailedException("Right limit switch not down.")

        # uneventful_moves = 0
        # total_moves = 0
        # while uneventful_moves <=2 and total_moves < 6:
        #     lines = self.do_command("right", 2000)
        #
        #     if self.is_arm_right():
        #         self.let_motor_stop_spinning()
        #         return
        #     elif self.is_arm_middle():
        #         uneventful_moves = 0
        #     else:
        #         uneventful_moves += 1
        # self.let_motor_stop_spinning()
        # raise RobotFailedException("should have gone to right by now.")

    def move_to_left(self):
        logging.comms(f"  move_to_left()")
        if self.is_arm_left():
            print("  - arm already left")
            return
        self.do_command("full_left", 8000)
        self.do_command("left", 1000)  # the cam is not fully engaged when the sensor trips, give the arm a nudge
        self.let_motor_stop_spinning()
        if not self.is_arm_left():
            raise RobotFailedException("Left limit switch not down.")

        # uneventful_moves = 0
        # total_moves = 0
        # while uneventful_moves <= 2 and total_moves < 6:
        #     lines = self.do_command("left", 2000)
        #
        #     if self.is_arm_left():
        #         self.let_motor_stop_spinning()
        #         return
        #     elif self.is_arm_middle():
        #         uneventful_moves = 0
        #     else:
        #         uneventful_moves += 1
        # self.let_motor_stop_spinning()
        # raise RobotFailedException("should have gone to right by now.")

    def move_to_middle(self):
        logging.comms(f"  move_to_middle()")
        if self.is_arm_middle():
            print("  - arm already at middle")
            return

        # Just in case. call functions to update self.last_known_arm_pos
        self.is_arm_left()
        self.is_arm_right()

        # no idea on location, let the logic to move left handle getting us orientated
        if self.last_known_arm_pos in 'um':
            self.move_to_left()

        if self.last_known_arm_pos == 'r':
            self.do_command("left_to_middle", 4000)
        if self.last_known_arm_pos == 'l':
            self.do_command("right_to_middle", 5000)

        # # We think we are heading from the right, take 2 motor jogs and see if we get there
        # if self.last_known_arm_pos == 'r':
        #     self.do_command("left", 2000)
        #     if self.is_arm_middle():
        #         return
        #     if not self.is_arm_left():
        #         self.do_command("left", 2000)
        #         if not self.is_arm_middle():
        #             self.do_command("right", 2000)  # back the arm off, we may have collided with the rear pillar
        #             self.do_command("right", 2000)  # back the arm off, we may have collided with the rear pillar
        #             raise RobotFailedException("Arm motion unable to reach middle position from right.")
        #
        # # We think we are heading from the left, , take 2 motor jogs and see if we get there
        # if self.last_known_arm_pos == 'l':
        #     self.do_command("right", 2000)
        #     if self.is_arm_middle():
        #         return
        #     if not self.is_arm_right():
        #         self.do_command("right", 2000)
        #         if not self.is_arm_middle():
        #             self.do_command("left", 2000)  # back the arm off, we may have collided with the rear pillar
        #             self.do_command("left", 2000)  # back the arm off, we may have collided with the rear pillar
        #             raise RobotFailedException("Arm motion unable to reach middle position from left.")

    def set_vert_jog_dist(self, distance: int):
        logging.comms(f"  set_vert_jog_dist({distance})")
        distance = min(max(1, int(distance)), 20_000)
        lines = self.do_command(f"set_v_dist {distance}", 1000)
        # if any("already at limit switch" in ln for ln in lines):
        #     print("Limit reached can't jog")
        #     return False

        actual = int(self._get_var(lines, 'vert_jog_len'))

        if distance != actual:
            raise RobotFailedException(f"robot command set_v_dist set {actual}, when requested to set {distance}.")

        return True


    def drop(self, raise_and_retry: bool = False):
        logging.comms(f"  drop({raise_and_retry})")
        self._serial_write_line("drop")
        s = self._serial_readline()
        s = self._serial_readline()
        if 'fail' in s.lower():
            self.jog_up()
            self._serial_write_line("drop")
            s = self._serial_readline()

        return 'fail' in s.lower()

    def do_top(self):
        logging.comms(f"  do_top()")
        lines = self.do_command("top", 5000)
        self.let_motor_stop_spinning()
        return lines
        # self.serial_write_line("top")
        # ok, lines = self.get_robot_feedback(5000)
        # if not ok:
        #     raise RobotFailed("Arm did not get to the top.")

    def do_spear_disk(self, time_out=3_000, retry=True):
        logging.comms(f"  do_spear_disk({time_out}, {retry})")
        logging.comms(f"begin spear: is_disk_on_gripper={self.is_disk_on_gripper()}")
        try:
            self.do_command("spear", time_out)
        except RobotFailedException as e:
            if self.is_disk_on_gripper():
                logging.warning("  - Warning: robot reported spear fail, but disk is in gripper. Proceeding.")
                return

            # the first spear often is not far enough
            if not retry:
                self.do_top()
                raise RobotFailedException(f"Unable to spear disk (single attempt) due to: {e}")
            try:
                self.do_command("spear", time_out//2)
            except RobotFailedException:
                self.do_top()
                raise RobotFailedException("Unable to spear disk.")

        logging.comms(f"spear done: is_disk_on_gripper={self.is_disk_on_gripper()}")

        # extra push ?
        # if not self.is_disk_on_gripper():
        #     self.jog_down(50)
        #     print(f"spear extra push: is_disk_on_gripper={self.is_disk_on_gripper()}")

        for i in range(0, 4):
            # move up and see if in gripper, if not plunge down
            print(f"spear small lift {i}: is_disk_on_gripper={self.is_disk_on_gripper()}")
            self.jog_up(200)
            if not self.is_disk_on_gripper():
                self.jog_down(300)

            print(f"spear big lift {i}: is_disk_on_gripper={self.is_disk_on_gripper()}")
            self.jog_up(1000)
            if not self.is_disk_on_gripper():
                # sometimes the disk is in the gripper, but its not on strait and the sensor is not registering
                print(f"spear, cd wobbled of sensor, or dropped {i}: is_disk_on_gripper={self.is_disk_on_gripper()}")
                self.drop()
                self.jog_down(1100)
                self.jog_up(1000)

            if not self.is_disk_on_gripper():
                self.do_command("spear", 1500) # end loop back in speared position
                print(f"spear re-spear{i}: is_disk_on_gripper={self.is_disk_on_gripper()}")
            else:
                print(f"spear grab loop finished {i}: is_disk_on_gripper={self.is_disk_on_gripper()}")
                break

        # print(f"spear done, lifting out of container {i}: is_disk_on_gripper={self.is_disk_on_gripper()}")
        # # final test jog up
        # self.jog_up(1000)
        # if not self.is_disk_on_gripper():
        #     # sometimes the dis is in the gripper, but its not on strait and the sensor is not registering
        #     self.drop()

        # lift up regardless
        print(f"spear done, moving to to{i}: is_disk_on_gripper={self.is_disk_on_gripper()}")
        self.do_top()
        if not self.is_disk_on_gripper():
            raise RobotFailedException("Unable to grab disk")


    # ------------------------------------------------------------------------------------------------------------------
    # Sensors
    # ------------------------------------------------------------------------------------------------------------------
    def is_disk_on_gripper(self):
        #  pin=32 state=0, transitions=13
        return self.get_pin(self.cd_gripper_pin).low()

    def is_disk_in_stroage(self):
        #  pin=32 state=0, transitions=13
        return self.get_pin(self.cd_storage_sensor_pin).low()

    def is_arm_left(self):
        if self.get_pin(self.left_sensor_pin).low():
            self.last_known_arm_pos = 'l'
            return True
        return False

    def is_arm_right(self):
        if self.get_pin(self.right_sensor_pin).low():
            self.last_known_arm_pos = 'r'
            return True
        return False

    def is_arm_middle(self):
        if self.get_pin(self.middle_sensor_pin).low():
            self.last_known_arm_pos = 'm'
            return True
        return False

    # ------------------------------------------------------------------------------------------------------------------
    # on state handlers
    # ------------------------------------------------------------------------------------------------------------------

    def on_enter_unknown(self):
        logging.warning(f"STATE: unknown")

    def on_enter_home(self):
        logging.info(f"STATE: home")

    def on_enter_ready(self):
        logging.info(f"STATE: ready")

    def on_enter_error(self):
        logging.info(f"STATE: error")

    def on_enter_loaded_disk(self):
        logging.info(f"STATE: loaded_disk")

    def on_enter_reading_disk(self):
        logging.info(f"STATE: reading_disk")

    def on_enter_ejected_disk(self):
        logging.info(f"STATE: ejected_disk")

    def on_enter_ejected_bad_disk(self):
        logging.info(f"STATE: ejected bad_disk")

    def on_enter_removed_disk(self):
        logging.info(f"STATE: removed disk")

    def on_enter_removed_bad_disk(self):
        logging.info(f"STATE: removed bad disk")

    # ------------------------------------------------------------------------------------------------------------------
    # Transition event handlers
    # ------------------------------------------------------------------------------------------------------------------
    def on_init_arm(self):
        # move arm to top, with 200 ticks worth of acceleration
        self.do_top()
        self.set_vert_jog_dist(200)
        self.jog_down()
        self.do_top()
        # we start in the middle because left sensor may be down with out the arm being fully left
        # (it needs an extra jog). This makes middle a good home position.
        # Also it makes loading disks easy.
        self.move_to_middle()

    def on_load_disk(self):
        print("  - (LOADING DISK): moving to source stack")
        self.move_to_left()
        self.set_vert_jog_dist(1000)
        self.jog_down()

        print("  - (LOADING DISK): spearing disk")
        self.do_spear_disk()

        print("  - (LOADING DISK): moving to middle")
        self.move_to_middle()

        print("  - (LOADING DISK): moving to source stack")
        cd_drive.open_cd_drive(self.cd_drive)
        # self.set_vert_jog_dist(self.drop_distance)
        self.jog_down(self.drop_distance)
        self.drop()
        self.do_top()
        cd_drive.close_cd_drive(self.cd_drive)

    def on_unload_disk(self):
        self.do_top()
        self.jog_down(6_000)
        self.do_spear_disk(time_out=1000, retry=False)
        self.do_top()
        if self.last_disk_ok:
            self.jog_down(2_300)
        self.move_to_right()
        self.drop()
        self.move_to_middle()

    def on_cycle_back_to_home(self):
        self.do_top()
        # self.move_to_left()

    def on_begin_read_disk(self):
        if self.dry_run:
            return

        print("## Reading disk")
        # cd_drive.open_cd_drive(self.cd_drive)
        volume_label, sn = cd_drive.get_volume_label_and_sn(self.cd_drive)
        if volume_label is not None:
            volume_label = slugify(volume_label)
            ts = int(datetime.datetime.now().timestamp())
            iso_file = f"{volume_label}_SN{sn}_TS{ts}.iso"
            iso_file = os.path.join(self.iso_folder, iso_file)
            cam_file = f"{volume_label}_SN{sn}_TS{ts}.jpg"
            cam_file = os.path.join(self.iso_folder, cam_file)

            print("  - writing camera image to to", iso_file)
            camera.save_image(self.camera_img, cam_file)

            print("  - writing optical disk image to", iso_file)
            # save .iso file and camera image
            self.last_disk_ok = cd_drive.make_iso_image(self.cd_drive, iso_file)

            # self.last_disk_ok = True
        else:
            self.last_disk_ok = False

        # take photo of next disk
        self.camera_img = camera.get_image(self.cam)


    def on_finish_read_disk(self):
        print("## Unloading disk")
        cd_drive.open_cd_drive(self.cd_drive)


