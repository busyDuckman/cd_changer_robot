# ----------------------------------------------------------------------------------------
# Code to interface with a CD drive in windows.
#
# Author: Warren Creemers
# See license.txt for license info.
# ----------------------------------------------------------------------------------------
import os
from functools import cache
from time import sleep
from datetime import datetime

@cache
def is_windows():
    import platform
    return "win" in platform.system().lower()


def get_win_drives():
    from ctypes import windll
    drives = []
    bitmask = windll.kernel32.GetLogicalDrives()
    for letter in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
        if bitmask & 1:
            drives.append(letter)
        bitmask >>= 1
    return drives


def mciSendString(msg: str):
    import ctypes
    print(f"    mci_send_string: {msg}", end='')
    ctypes.windll.WINMM.mciSendStringW(msg, None, 0, None)

    print("      (done)")



def open_cd_drive(drive):
    if is_windows():
        drive_letter = drive[0]
        mciSendString(f"open {drive_letter}: type cdaudio alias cdrom")
        print("  - opening cd drive ", end='')
        mciSendString("set cdrom door open")
        print('done')
        mciSendString("close cdrom")
    sleep(1)  # may take a while because the cd needs to spin down.


def close_cd_drive(drive):
    if is_windows():
        drive_letter = drive[0]
        mciSendString(f"open {drive_letter}: type cdaudio alias cdrom")
        mciSendString("set cdrom door closed")
        mciSendString("close cdrom")
    sleep(1)


def detect_drives():
    # https://stackoverflow.com/questions/2288065/enumerate-cd-drives-using-python-windows
    from ctypes import windll

    drive_types = ['DRIVE_UNKNOWN', 'DRIVE_NO_ROOT_DIR', 'DRIVE_REMOVABLE', 'DRIVE_FIXED', 'DRIVE_REMOTE', 'DRIVE_CDROM',
                  'DRIVE_RAMDISK']

    print("Detecting CD drives:")
    cd_drives = []
    for drive in get_win_drives():
        if not drive:
            continue
        print("  - Found drive:", drive, end='')
        try:
            type_index = windll.kernel32.GetDriveTypeW(u"%s:\\" % drive)
            if type_index == 5:
                cd_drives.append(f"{drive}:\\")
            print(f" (type={drive_types[type_index]})")
        except Exception as e:
            print("error:", e)

    print(f"{len(cd_drives)} cd drives found.")

    return cd_drives


def get_volume_label_and_sn(drive):
    if is_windows():
        import win32api
        import pywintypes
        try:
            lbl, sn, _, _, fs = win32api.GetVolumeInformation(drive)

            return lbl, sn
        except pywintypes.error:
            print("error: bad disk")
            return None, None
    else:
        raise ValueError("get_volume_label only supports windows at the moment.")


def make_iso_image_cdburnerxp(drive, iso_file, xp_dir = "C:\\Program Files\\CDBurnerXP"):
    if not is_windows():
        raise ValueError("make_iso_image_cdburnerxp only supported in windows")

    if not os.path.isdir(xp_dir):
        raise ValueError("could not find cd burner xp at:", xp_dir)

    cdbxpcmd = os.path.join(xp_dir, "cdbxpcmd.exe")

    if not os.path.isfile(cdbxpcmd):
        raise ValueError("could not find cdbxpcmd.exe at :", cdbxpcmd)

    print(f"  - found cd burner xp: ", cdbxpcmd)

    # "--burn-data -folder:F:\ -iso:C:\share\test_rip.iso -format:iso"

    # subprocess.run([cdbxpcmd, "burn-data", f"folder:{drive}", f"iso:{iso_file}", "format:iso"])
    cmd = f'"{cdbxpcmd}" --burn-data -folder:{drive} -iso:{iso_file} -format:iso'
    print(f"  - executing: ", cmd)
    os.system(cmd)

    # out = ""
    # if "occured while executing the command" in out:
    #     return False

    if not os.path.isfile(iso_file):
        print("BAD ISO: File not found after ripping: ", iso_file)
        return False

    return True


def make_iso_image_anyburn(drive, iso_file, anyburn_dir = "C:\\Program Files\\AnyBurn"):
    if not is_windows():
        raise ValueError("make_iso_image_anyburn only supported in windows")

    if not os.path.isdir(anyburn_dir):
        raise ValueError("could not find anyburn at:", anyburn_dir)

    abcmd = os.path.join(anyburn_dir, "abcmd.exe")
    print(f"  - found anyburn: ", abcmd)

    drive_letter = drive.upper()[0]
    cmd = f'"{abcmd}" make-image {drive_letter}: -y -o {iso_file.replace(".iso", ".bin")}'
    print(f"  - executing: ", cmd)
    os.system(cmd)

    return True



def make_iso_image(drive, dest_iso_file):
    print(f'Ripping disc in {drive} to "{dest_iso_file}":')
    if is_windows():
        return make_iso_image_cdburnerxp(drive, dest_iso_file)
        # make_iso_image_anyburn(drive, dest_iso_file)


def is_disk_in_drive(drive):
    assert(drive is not None)
    # print(get_volume_label_and_sn(drive))
    drive_letter = drive.lower()[0]
    if is_windows():
        import wmi
        wmi_interface = wmi.WMI()
        drive = [c for c in wmi_interface.Win32_CDROMDrive() if (c.Drive is not None) and (c.Drive.lower()[0] == drive_letter)]
        if len(drive) == 0:
            raise ValueError("Drive not found.")
        return drive[0].MediaLoaded

    raise ValueError("is_disk_in_drive only supported on windows")


def main():
    # import wmi
    # import os
    # import time
    # import ctypes
    # c = wmi.WMI()
    # for cdrom in c.Win32_CDROMDrive():
    #     status = cdrom.MediaLoaded
    #     print('Media present', cdrom, status)
    #
    # exit(0)

    drives = detect_drives()

    if len(drives) == 0:
        print("Exiting: no cd drives found.")
        exit(0)

    drive = drives[0]
    print("Using drive: ", drive)
    open_cd_drive(drive)
    print("  - closing drive")
    close_cd_drive(drive)

    if is_disk_in_drive(drive):
        print("  - getting info")
        volume_label, sn = get_volume_label_and_sn(drive)
        iso_file = f"{volume_label}_{sn}_{int(datetime.now().timestamp())}.iso"
        print(iso_file)

        iso_folder = "c:\\share"
        iso_file = os.path.join(iso_folder, iso_file)

        make_iso_image(drive, iso_file)
    else:
        print("No disk in drive.")

if __name__ == '__main__':
    main()
