This is a python script to run my modified Nistec ALW-501 Optical Disk Robot.

I'll preface this by saying it was unapologetically written in a hurry.
The robot was built to deal with a situation of transferring circa 1,000 backup disks to a HDD.
If I spent too long on the code, or building the robot, I might as well copy the disks manually.

This python code interfaces to my custom firmware on my custom replacement motherboard for the unit.
It has no understanding of the original RS232 protocol for the robot.

The robot control and camera script is windows/linux friendly, the CD imaging part is windows only.

If the robot becomes stuck, or enters a bad state (causing the script to stop), re-run the script.
The script start-up is pretty robust at getting the robot back to the home state without damaging anything.

To set robot running and make iso images of a pile of disks:
    run.py

To return robot to the home position:
   run.py --mode home

To check all sensors are working:
  run.py --mode test_pins

Robot logic:

    MOVE home
    WHILE disk is in inbox (left spool):
        photo-graph disk
        MOVE left
        MOVE pickup disk
        MOVE center
        CD-DRIVE open
        MOVE down
        MOVE drop disk
        MOVE up
        CD-DRIVE close
        CD-DRIVE make iso image
        CD-DRIVE open
        MOVE pickup disk
        CD-DRIVE close
        IF read failed:
            MOVE down
            Move drop disk   # disk falls through to reject tray
        ELSE:
            MOVE right
            MOVE down
            MOVE drop disk
        MOVE home
        
        
        





