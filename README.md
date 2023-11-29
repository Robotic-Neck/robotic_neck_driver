# robotic_neck_driver
ROS2 package to bringup the comunication between the platform and the computer`s user, to connect the RPI pico and the arduino-IMU. it also contain a node to apply teleoperation by a PS4 controller.
<!--
<p align="center">
  <img width="640" height="480" src="neck_mec_sim.png">
</p>
-->
## Dependencies
* Operating system: [Ubuntu 22.04](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
* Robotic Frameworks:
  * [ROS2 Humble (desktop)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
  * [Vulcanexus (base)](https://docs.vulcanexus.org/en/humble/rst/installation/linux_binary_installation.html)
* [launch_utils](https://github.com/MonkyDCristian/launch_utils)

## Install and Compile
**Note:** Install [launch_utils](https://github.com/MonkyDCristian/launch_utils) in your workspace before follow this step
```
cd <path to your workspace>/src/
git clone https://github.com/Robotic-Neck/robotic_neck_driver.git
cd ..
colcon build --packages-select robotic_neck_viz
```

## Install ROS packages dependencies with rosdep  
```
cd <path to your workspace>/src/
rosdep install -i --from-path src --rosdistro humble -y
```

## Demo
**Note**: Execute all these launches in different terminals.

1. Conect the RPI pico:
```
ls /dev/serial/by-id/*
ros2 run micro_ros_agent micro_ros_agent serial --dev <port>
```

2. Open the IK vizualization:
```
ros2 launch robotic_neck_viz robotic_neck_urdf.launch.py 
```

3. Connect the Arduino-IMU by port /dev/ttyUSB0:
```
ros2 run robotic_neck_driver imu_driver
```
**Note:** To change the port edit it [here](/robotic_neck_driver/imu_driver.py) on line 12.


4. Start controller:
```
ros2 launch platform_controller robotic_neck_controller.launch.py
```

## Documentation
The IMU code is [here](/arduino/IMU), check it out in Arduino IDE.
