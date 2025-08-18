# AUTONOMOUS NAVIGATION WITH GPS AND WITHOUT GPS

After running localization run:

For aruco detection:
```
roslaunch aruco_detect aruco_detect.launch
```

For autonomous navigation:
```
roslaunch locomove_base locomove.launch
```

For behaviour tree:
```
rosrun auto_nav mission.py
```

For sending x, y coordinates use:

```
rosservice call /mission "x: 0.0
y: 5.0
poles: [2]" 
```

Enter x,y coordinates and pole ids. You can send empty array for ids.

To use GPS, use navigation interface which is available on itu-rover-ui navigation branch.