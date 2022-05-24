**This repo concerns a Nistec ALW-501 Optical Disk Robot I converted to a bulk disk ripper.**

The robot has done its job and is being mothballed.
This repo is here for anyone who would like to attempt a similar project, or inherits my original robot.

See runner/run.py to start the robot.

Features:
  - Converted a Nistec ALW-501 to a CD ripper.
  - Using an arduino to replace the existing motherboard.
  - Python script rips disks to .iso images using CDBurnerXP or Anyburn
    - see runner/cd_drive.py
  - Takes photos of disks before ripping using a webcam.
  - Places bad rips into a separate pile for data recovery.


Mothballed:
   - Does anyone have a good home for this device, eg: a public library wanting to move its CD collection to cloud storage.
