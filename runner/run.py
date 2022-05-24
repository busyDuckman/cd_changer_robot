# ----------------------------------------------------------------------------------------
# Code to rum the customised Nistec ALW501 robot.
#
# Author: Warren Creemers
# See license.txt for license info.
# ----------------------------------------------------------------------------------------
import argparse
import cd_drive
from robot import CDRobot, RobotFailedException


def parse_args(default_iso_path='C:\\share\cd_robot'):
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, required=False, default='run',
                        help="mode[run, test_pins, calibrate_load, home].")
    parser.add_argument('--iso_file_path', type=str, default=default_iso_path,
                        help=f"path to store iso files in (default='{default_iso_path}').")

    args = parser.parse_args()
    return args.mode, args.iso_file_path


def get_com_ports():
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()

    print("Detecting com ports:  ")
    for port, desc, hwid in sorted(ports):
        print("  - {}: {} [{}]".format(port, desc, hwid))

    return ports


def init(iso_file_path) -> CDRobot:
    # find cd drive
    drives = cd_drive.detect_drives()
    if len(drives) == 0:
        print("Error: Non cd-drive found.")
        drive = None
    else:
        drive = drives[0]

    ports = get_com_ports()
    if len(ports) == 0:
        print("No com ports found.")
        exit(1)
    cd_robot = None
    print("Looking for CD robot:  ")
    for port, _, _ in sorted(ports):
        print(f"  - checking port {port}")
        try:
            cd_robot = CDRobot(port, drive, iso_folder=iso_file_path)
            print("  - robot found.")
            break
        except RobotFailedException:
            print("  - robot not found")

    if cd_robot is None:
        print("Robot not found on any com ports.")
        exit(1)

    return cd_robot


def main():
    mode, iso_file_path = parse_args()
    # setup_logging()

    cd_robot = init(iso_file_path)

    print()
    print(f"interfacing with robot {cd_robot}")
    if mode == 'run':
        cd_robot.main_loop()
    elif mode == 'test_pins':
        old_pins = {}
        while True:
            print(".", end='')
            old_pins = cd_robot.list_pins(old_pins)
    elif mode == 'calibrate_load':
        cd_robot.move_to_middle()
        cd_robot.do_top()
        cd_drive.open_cd_drive(cd_robot.cd_drive)
        cd_robot.jog_down(6_400)
        cd_robot.drop()
        cd_robot.do_top()
    elif mode == 'calibrate_unload':
        cd_robot.move_to_middle()
        cd_robot.do_top()
        cd_drive.open_cd_drive(cd_robot.cd_drive)
        # cd_robot.
        cd_robot.jog_down(6_000)
        cd_robot.do_spear_disk(time_out=1000, retry=False)
        # cd_robot.drop()
        # cd_robot.do_top()
    elif mode == 'calibrate_store_low':
        cd_robot.move_to_middle()
        cd_robot.do_top()
        cd_robot.jog_down(2_300)
        cd_robot.move_to_right()
        cd_robot.drop()
        cd_robot.move_to_middle()
        cd_robot.do_top()
    elif mode == 'home':
        cd_robot.do_top()
    else:
        print("unknown mode: ", mode)

    exit(0)


if __name__ == '__main__':
    main()
