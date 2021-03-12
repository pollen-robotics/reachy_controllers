# Reachy controllers

ROS2 package handling Reachy:
- joints state and command,
- fans management,
- force sensor reading,
- publishing images from both cameras,
- and controlling the zoom.

**ROS2 Version: Foxy**

Dependencies: [reachy_msgs](https://github.com/pollen-robotics/reachy_msgs), [reachy_pyluos_hal](https://github.com/pollen-robotics/reachy_pyluos_hal), [zoom_kurokesu](https://github.com/pollen-robotics/zoom_kurokesu)

How to install:

```bash
cd ~/reachy_ws/src
git clone https://github.com/pollen-robotics/reachy_controllers.git
cd ~/reachy_ws/
colcon build --packages-select reachy_controllers
```


## Published topics

* **/left_image** ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html))[[camera_publisher](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/camera_publisher.py)] - Compressed image from the left camera.<br> Default size: 1280x720, default fps: 30.

* **/right_image** ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html))[[camera_publisher](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/camera_publisher.py)] - Compressed image from the right camera.<br> Default size: 1280x720, default fps: 30.

* **/joint_states** ([sensors_msgs/msg/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html))[[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py)] - Present position, velocity and effort from each joint (both arms, orbita and antennas). <br> Default rate: 100Hz.

* **/joint_temperatures** ([reachy_msgs/msg/JointTemperature](https://github.com/pollen-robotics/reachy_msgs/blob/master/msg/JointTemperature.msg))[[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py)] Temperature from each joint (both arms, orbita and antennas).
<br> Default rate: 0.1Hz.

* **/force_sensors** ([reachy_msgs/msg/ForceSensor](https://github.com/pollen-robotics/reachy_msgs/blob/master/msg/ForceSensor.msg))[[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py)] - Force sensor value for left and right gripper. <br> Default rate: 10Hz.

* **/fan_states** ([reachy_msgs/msg/FanState](https://github.com/pollen-robotics/reachy_msgs/blob/master/msg/FanState.msg))[[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py)] State for each fan.
<br> Default rate: 0.1Hz.

## Subscribed topics
* **/joint_goals** ([sensors_msgs/msg/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html))[[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py)] -
The callback of this topic uses [reachy_pyluos_hal](https://github.com/pollen-robotics/reachy_pyluos_hal) to set the goal_position, effort and velocity of the given joints.

## Services

* **/set_camera_zoom_level** ([reachy_msgs/srv/SetCameraZoomLevel.srv](https://github.com/pollen-robotics/reachy_msgs/blob/master/srv/SetCameraZoomLevel.srv))[[camera_zoom_controller_service](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/camera_zoom_service.py)] - Change the zoom on the given camera side to one of the predefined level, or perform homing. <br> See [zoom_kurokesu repository](https://github.com/pollen-robotics/zoom_kurokesu) for more information.

* **/set_camera_zoom_speed** ([reachy_msgs/srv/SetCameraZoomSpeed.srv](https://github.com/pollen-robotics/reachy_msgs/blob/master/srv/SetCameraZoomSpeed.srv))[[camera_zoom_controller_service](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/camera_zoom_service.py)] - Change the speed of the zoom motors. <br> See [zoom_kurokesu repository](https://github.com/pollen-robotics/zoom_kurokesu) for more information.

* **/get_joint_full_state** ([reachy_msgs/srv/GetJointFullState.srv](https://github.com/pollen-robotics/reachy_msgs/blob/master/srv/GetJointFullState.srv))[[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py)] - Returns position, velocity, effort, temperature, compliance, goal position, speed limit, torque limit and PID of requested joints.

* **/set_joint_compliancy** ([reachy_msgs/srv/SetJointCompliancy.srv](https://github.com/pollen-robotics/reachy_msgs/blob/master/srv/SetJointCompliancy.srv))[[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py)] - Set compliancy of the requested joints to the requested compliance boolean (True=compliant, False=stiff). 

* **/set_joint_pid** ([reachy_msgs/srv/SetJointPidGain.srv](https://github.com/pollen-robotics/reachy_msgs/blob/master/srv/SetJointPidGain.srv))[[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py)] - Set PID gains of the requested joints. Warning the required values and their range may change depending on the type of joint. Please refer to the motor documentation for more information. 

* **/set_fan_state** ([reachy_msgs/srv/SetFanState.srv](https://github.com/pollen-robotics/reachy_msgs/blob/master/srv/SetFanState.srv))[[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py)] - Set requested fans to requested state (on or off). 


## Launch files

* **camera_controller.launch.py** - Launch camera_publisher and camera_zoom_controller_service nodes. 
* **camera_publisher.launch.py** - Launch camera_publisher node.
* **camera_zoom_service.launch.py** - Launch camera_zoom_controller_service node.
* **joint_state_controller.launch.py** - Launch joint_state_controller node.
* **reachy_controllers.launch.py** - Launch camera_publisher, camera_zoom_controller_service and joint_state_controller nodes.

---
This package is part of the ROS2-based software release of the version 2021 of Reachy.

Visit [pollen-robotics.com](https://pollen-robotics.com) to learn more or visit [our forum](https://forum.pollen-robotics.com) if you have any questions.
