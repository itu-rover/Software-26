# Teleoperation package for 23' manipulator

## Usage

Following commands are required for teleoperation:

```rosrun arm23_teleop teleop_server```

```rosrun arm23_teleop teleop.py```


## Controls
To change mode first press the **middle button** on the F710/F310. Then, 
### Select + Up:    JOINT MODE
This mode allows to control each axis separately.

### Select + Left:  IDLE MODE
This mode puts the manipulator in idle as the name state.

### Select + Right: CARTESIAN MODE
This mode is used to control the position of the end effector in Cartesian space.

### Select + Down:  POSE MODE
this mode is same as idle mode but it allows hardware interface to send serial message. So it is possible to send pose command via rviz in this mode.
