# ft-sensor-calibration
ROS 2 calibration procedures for Torque-Force sensor in Agimus project

## Install

```bash
vcs import --shallow --recursive src < src/panda_ft_sensor_calibration/dependencies.repos

rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i --from-paths src --rosdistro $ROS_DISTRO
# parameter --symlink-install is optional
colcon build --symlink-install
```

## Static calibration

First record data

```bash
ros2 launch panda_ft_sensor_calibration jtc_follow.launch.py robot_ip:=<ip of the robot> ft_sensor_ip:=<ip of the ft sensor> output_rosbag_path:=<path to store rosbag>
```

Then preprocess it
```bash
ros2 run rosbag2pandas rosbag2pandas <path to a rosbag from previous step> <path to store parsed data> -s mcap --format parquet
```

After all those steps are done move to a notebook [mass_calibration.ipynb](./notebooks/mass_calibration.ipynb) and follow steps there.

## Dynamic calibration

First manually record the motion

```bash
ros2 launch panda_ft_sensor_calibration record_motion.launch.launch.py robot_ip:=<ip of the robot> ft_sensor_ip:=<ip of the ft sensor> output_rosbag_path:=<path to store a rosbag with manual motion>
```

Then rerecord that motion without external forces from a human
```bash
ros2 launch panda_ft_sensor_calibration playback_with_sensor_recording.launch.launch.launch.py robot_ip:=<ip of the robot> ft_sensor_ip:=<ip of the ft sensor> output_rosbag_path:=<path to store rosbag with replayed> input_rosbag_path:=<path to a rosbag from previous step>
```

Then preprocess the data
```bash
ros2 run rosbag2pandas rosbag2pandas <path to store rosbag> <path to rosbag with replayed motion> -s mcap --format parquet
```
