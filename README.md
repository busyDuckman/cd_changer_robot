<div align="center">

# CD Changer Robot

**A Nistec ALW-501 optical disk robot, I modernised and rebuilt as a bulk CD ripper.**

[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE.txt)
[![Status: Mothballed](https://img.shields.io/badge/status-mothballed-lightgrey.svg)](#project-status)
[![Python 3](https://img.shields.io/badge/python-3-blue.svg?logo=python&logoColor=white)](runner/)
[![Arduino Mega](https://img.shields.io/badge/arduino-Mega%201280-00979D.svg?logo=arduino&logoColor=white)](firmware_megaatmega1280/)
[![PlatformIO](https://img.shields.io/badge/built%20with-PlatformIO-orange.svg?logo=platformio&logoColor=white)](https://platformio.org/)
[![Platform: Windows](https://img.shields.io/badge/ripping-Windows-0078D6.svg?logo=windows&logoColor=white)](runner/cd_drive.py)
[![Last commit](https://img.shields.io/github/last-commit/busyDuckman/cd_changer_robot.svg)](https://github.com/busyDuckman/cd_changer_robot/commits/main)

I bought this robot ina "parts only" condition of eBay for $50. It was a seriously well built bit of kit and worth the restore. I hope this repo may save someone else a heap of work.
  
[![Watch the robot in action](youtube_thumbnail.jpg)](https://youtube.com/shorts/DimQMZp4Gug)

*Click to watch it in action on YouTube.*

</div>

---

## About

I would routinely backup files to optical disk and toss them into a silo. One day I needed some old files and realised my computer does not
even have a CD drive anymore. A thousand odd backup disks were degrading, so I did the only reasonable thing I knew how, I built a robot. 

The first version was a home made arm, that worked, but did not have the reliability to scale. I saw that old CD burning robots were cheap and grabbed a Nistec ALW-501 of ebay. 

The original mobo on the unit appeared cooked and would not respond to commands. So I ditched it and reverse engineered the motor driver board. Then built a new mobo using an arduino, some Veroboard, and a few matching connectors. Then set up a python harness to run the thing from a PC.

The robot has done its job and is being mothballed. This repo is here for anyone who wants to try a similar project, or who ends up inheriting the robot.

**Note:** It's not shown in the video, but I added a 3d printable part a "CD sorter". using this The drop height at the done pile can be used to determine which of two piles a disk ends up in. The system is configured to use it, but if it is not present, then you just get one pile. 

> [!TIP]
> **Free to good home!** Do you know a good home for this device? A public library that wants to move its CD collection to cloud storage would be ideal.


Features:
  - Using an arduino to replace the existing motherboard.
  - Python script rips disks to .iso images using CDBurnerXP or Anyburn
  - Takes photos of disks before ripping using a webcam.
  - Places bad rips into a separate pile for data recovery.


## Repository layout

| Path | Contents |
| --- | --- |
| [runner/](runner/) | Python control script. Talks to the firmware over a USB serial port, runs the CD drive, and takes photos. |
| [firmware_megaatmega1280/](firmware_megaatmega1280/) | Arduino firmware (PlatformIO project) for the new motherboard. |
| [nistec_ALW_501/](nistec_ALW_501/) | Reverse-engineering notes: pin map, a traced driver-board circuit, and datasheets for its chips. |
| [3d_printed_new_parts/](3d_printed_new_parts/) | Some new parts were needed: Cable guard, CD sorter (reject chute), and a shim for the Lite-On drive. |
| [laser_cut_parts/](laser_cut_parts/) | My robot was missing the silo for the disks. This adapter (.dxf) lets you place a regular disk bin in the exact right spot.|
| [docs/](docs/) | Reference assembly images. |

## How it works

```mermaid
flowchart LR
    subgraph MAIN [Main loop]
        direction TB
        START([run.py]) --> INIT[Init arm:<br/>top, jog down 200, top again,<br/>move to middle]
        INIT --> PHOTO0[Webcam photo of top disk in inbox]
        PHOTO0 --> HOME([Home: arm middle and at top])

        HOME --> INBOX{Disk in inbox?<br/>pin 32}
        INBOX -- no, poll every 2s --> INBOX
        INBOX -- yes --> PAUSE[5 second countdown<br/>for manual intervention]

        %% Load
        PAUSE --> LEFT[full_left, then nudge left<br/>so the cam fully engages]
        LEFT --> DOWN1[Jog down 1000]
        DOWN1 --> SPEAR1[[Spear disk]]
        SPEAR1 --> MID1[Move to middle]
        MID1 --> OPEN1[Open drive tray]
        OPEN1 --> DOWN2[Jog down 6400]
        DOWN2 --> DROP1[Drop disk into tray]
        DROP1 --> TOP1[Top, close drive]

        %% Read
        TOP1 --> LABEL{Volume label<br/>readable?}
        LABEL -- no --> BAD[Mark disk bad]
        LABEL -- yes --> SAVEJPG[Save photo as<br/>label_SNxxx_TSxxx.jpg]
        SAVEJPG --> ISO[Make ISO with CDBurnerXP]
        ISO --> ISOOK{ISO OK?}
        ISOOK -- no --> BAD
        ISOOK -- yes --> GOOD[Mark disk good]
        BAD --> PHOTO[Webcam photo of next disk in inbox]
        GOOD --> PHOTO

        %% Unload
        PHOTO --> OPEN2[Open drive tray]
        OPEN2 --> DOWN3[Top, jog down 6000]
        DOWN3 --> SPEAR2[[Spear disk<br/>single attempt, no retry]]
        SPEAR2 --> TOP2[Top]
        TOP2 --> WASOK{Disk good?}
        WASOK -- yes --> DOWN4[Jog down 2300]
        WASOK -- no, stay at top --> RIGHT
        DOWN4 --> RIGHT[full_right]
        RIGHT --> DROP2[Drop disk<br/>drop height sorts good from bad]
        DROP2 --> MID2[Move to middle, top]
        MID2 --> HOME
    end

    MAIN ~~~ SPEAR

    %% Spear subroutine
    subgraph SPEAR [Spear disk subroutine]
        direction TB
        S1[spear: lower until<br/>gripper sensor pin 40 trips] --> S1OK{Firmware OK?}
        S1OK -- yes --> L0
        S1OK -- no --> S1G{Disk on gripper<br/>anyway?}
        S1G -- yes --> L0
        S1G -- no --> S1R{Retry allowed?}
        S1R -- no --> SFAIL
        S1R -- yes --> S2[spear again,<br/>half timeout]
        S2 -- ok --> L0
        S2 -- fail --> SFAIL

        L0[Attempt i = 1..4] --> L1[Small lift: up 200]
        L1 --> L1C{On gripper?}
        L1C -- no --> L1D[Down 300] --> L2
        L1C -- yes --> L2[Big lift: up 1000]
        L2 --> L2C{On gripper?}
        L2C -- yes --> LDONE
        L2C -- no, disk wobbled<br/>off sensor or fell --> L3[Drop, down 1100, up 1000]
        L3 --> L3C{On gripper?}
        L3C -- yes --> LDONE
        L3C -- no --> L4[Re-spear]
        L4 --> LMORE{Attempts left?}
        LMORE -- yes --> L0
        LMORE -- no --> LDONE

        LDONE[Move to top] --> SFIN{On gripper?}
        SFIN -- yes --> SOK([Disk held])
        SFIN -- no --> SFAIL([Fail: top, raise error])
        SFAIL -.-> ERR([Script stops.<br/>Re-run run.py to recover.])
    end
```



## Controller Circuit

I used a Seeduino mega v1.1, and connected the IO via the dual row header


### Pin mapping: Arduino Mega to original driver board

The Arduino I/O pins are wired to the driver board as per:


| Pin | Function | Firmware name |
| :---: | --- | --- |
| 23 | Front panel **ERROR** light | ERROR_LIGHT_PIN |
| 24 | Arm motor **up** | ARM_UP_PIN |
| 25 | Arm motor **left** | ARM_LEFT_PIN |
| 26 | Front panel **BUSY** light | BUSY_LIGHT_PIN |
| 27 | **Gripper** (LOW = grip, HIGH = release) | GRIPPER_PIN |
| 28 | Arm motor **down** | ARM_DOWN_PIN |
| 29 | Arm motor **right** | ARM_RIGHT_PIN |

**Notes:**  
  - Each motor axis has a pair of direction pins connected to a H-bridge on the driver board, best not drive both pins high at the same time :)
  - All sensor inputs are active low.
  - Pickup works 80% of the time, so you need to check the gripper worked via the sensor and repeat the pickup process if it fails.

### Inputs (driver board to Mega)

| Pin | Function | Used in code as |
| :---: | --- | --- |
| 30 | Doors are open | *(read, not used)* |
| 31 | Right disk switch | *(read, not used)* |
| 32 | Left disk optical sensor: disk present in inbox | cd_storage_sensor_pin |
| 33 | Arm vertical **top** limit | ARM_VERTICAL_STOP_INDEX |
| 38 | Left disk switch | *(read, not used)* |
| 39 | Rotary cam pulse: arm in **middle** position | ARM_MIDDLE_SENSOR_INDEX / middle_sensor_pin |
| 40 | **Disk on gripper** | GRIP_SENSOR_PIN / cd_gripper_pin |
| 41 | Tower/arm at **right** | ARM_RIGHT_SENSOR_INDEX / right_sensor_pin |
| 42 | Arm vertical encoder ticks (used for distance) | ARM_VERTICAL_TICK_INDEX |
| 44 | Tower/arm at **left** | ARM_LEFT_SENSOR_INDEX / left_sensor_pin |
| 47 | CD dropped | *(read, not used)* |

The firmware polls pins 30-33 and 38-53 every 500 us on a timer interrupt and counts transitions on each one. You can see the live state of all of them with the who command or run.py --mode test_pins.

> [!NOTE]
> Still unknown: pin **22** and pin **46**. My unit had some damage, perhaps someone with an intact machine will find out whats up.

### Driver board reference

[nistec_ALW_501/](nistec_ALW_501/) has a diagram of top and bottom traces on the OEM driver board ([reversed_driver_circuit.png](nistec_ALW_501/reversed_driver_circuit.png)), a matching photo of the PCB that it can overlay on. It also holds datasheets for the main components.

<a href="nistec_ALW_501/reversed_driver_circuit.png"><img src="nistec_ALW_501/reversed_driver_circuit.png" alt="Circuit routes" width="400"></a>

[docs/arduino_harness_and_assembly/](docs/arduino_harness_and_assembly) has reference images of the new controller board and its assembly and connection.

<a href="docs/arduino_harness_and_assembly/arduino_on_harness.jpg"><img src="docs/arduino_harness_and_assembly/arduino_on_harness.jpg" alt="Assembly Image" width="400"></a>

docs\arduino_harness_and_assembly\arduino_on_harness.jpg


## Firmware control commands

The new firmware (what I added, not the original unit) uses 9600 baud, newline-terminated text commands. 
Every command returns either '_OK_' or '_FAIL_' when done. Before that arrives diagnostic output is  returned, always starting with 'info:' or 'error:'.

| Command | Action |
| --- | --- |
| help | List commands |
| grip / release | Close / open the gripper |
| drop | Release for 500 ms, then grip again |
| left / right | Jog the arm sideways for 500 ms |
| full_left / full_right | Move the arm until the left/right limit triggers |
| left_to_middle / right_to_middle | Move the arm until the middle sensor triggers |
| up / down | Jog the arm vertically by set_v_dist encoder ticks |
| set_v_dist <n> | Set the vertical jog distance |
| top | Raise the arm until the top limit triggers |
| spear | Lower the arm until a disk is detected on the gripper |
| who [pin] | Print state and transition count for all pins, or for one pin |

## Getting started

### 1. Flash the firmware

The firmware is a [PlatformIO](https://platformio.org/) project for an Arduino Mega 1280. It needs the **TimerOne** and **digitalWriteFast** libraries, which are bundled in [required_libraries.rar](firmware_megaatmega1280/required_libraries.rar).

```bash
cd firmware_megaatmega1280
pio run --target upload
```

### 2. Set up the runner

```bash
cd runner
pip install -r requirements.txt
```

You also need **Windows**, because ISO creation shells out to Windows tools. Robot control and the camera also work on Linux. Install [CDBurnerXP](https://cdburnerxp.se/) (the default) or [AnyBurn](https://www.anyburn.com/) in its standard location under `C:\Program Files\`.

### 3. Run it

The script scans COM ports to find the robot, and uses the first CD drive and the last webcam it finds.

```bash
python run.py                                  # rip everything in the inbox
python run.py --iso_file_path D:\rips          # choose where ISOs go (default C:\share\cd_robot)
python run.py --mode home                      # return the arm to the home position
python run.py --mode test_pins                 # watch sensor states live
```

There are also `calibrate_load`, `calibrate_unload` and `calibrate_store_low` modes for tuning the vertical travel distances.

See [runner/README.md](runner/README.md) for more detail.

## The CD sorter

This nifty device gives you a extra end bay for free. Disks released above it slide away to a box you can store behind the machine. 

<a href="docs/cd_sorter.jpg"><img src="docs/cd_sorter.jpg" alt="CD sorter" width="400"></a>

## Project status

**Mothballed.** The robot has done the job it was built for. The code was *"unapologetically written in a hurry"*, because if I had spent too long on it, I might as well have copied the disks by hand. Expect rough edges, hard-coded paths, etc.

Issues and forks are welcome if you're building something similar.

## License

Released under the [MIT License](LICENSE.txt).
