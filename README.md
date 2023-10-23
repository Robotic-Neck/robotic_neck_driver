# robotic_neck_driver
ROS2 package to bringup the comunication between the platform and the computer`s user. it also contain a node to apply teleoperation by a PS4 controller.
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
```
TODO
```

## Documentation
TODO
