# Installation
1. clone the repository in `~/ros2_ws/src`
   ```shell
   cd ~/ros2_ws/src
   git clone https://github.com/Ramisha-Anjum/test_pkg.git
   ```
2. build the packages in `~/ros2_ws`
   ```shell
   cd ~/ros2_ws
   colcon build --symlink-install
   ```
# Necessary Commands
All the necessary commands are in the `command for test_pkg` file. Use them while needed.

# Necessary Info:
Topic used for subscriber in test_noode:
```shell
/Mavic_2_PRO/gps    # used to read the robot position (x,y)
/Mavic_2_PRO/gps/speed_vector    # used to read the robot velocity (x,y)
/Mavic_2_PRO/imu    # used to read the difference of angles of robot axis and world axis
/fd/ee_pose    # used to read the positions of falcon hands
```

Topic used for publisher in test_noode:
```shell
Mavic_2_PRO/cmd_vel    # to publish the velocity in order to move the robots
/fd/fd_controller/commands    # to publish the feedback force in falcon hands
```
