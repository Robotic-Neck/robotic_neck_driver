
"""
    Launch file for joystick control of the robotic neck by joy_teleop pkg
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from launch_utils.utils import include_launch

def generate_launch_description():
    launchs = []

    # ADD JOYSTICK CONTROL
    #launchs.append(include_launch(package_name="robotic_neck_driver", launch_file="joystick.launch.py"))
    
    # ADD ROBOT MICRO ROS AGENT FOR ACTUATOR 
    launchs.append(Node(package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent_actuator', 
                        arguments=["serial", "-D", "/dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_875862E639A074D3-if00"]))
    
    # ADD ROBOT MICRO ROS AGENT FOR IMU
    launchs.append(Node(package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent_imu', 
                        arguments=["serial", "-D", "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6625887D36E1E2F-if00"]))
    
    # ADD ROBOT STATE AND TARGET VIZUALIZATION
    launchs.append(include_launch(package_name="robotic_neck_viz", launch_file="robotic_neck_urdf.launch.py"))
    
    # ADD CONTROLLER
    launchs.append(include_launch(package_name="platform_controller", launch_file="robotic_neck_controller.launch.py"))

    # ADD OAK-D-LITE CAMERA
    #launchs.append(include_launch(package_name="robotic_neck_driver", launch_file="camera.launch.py"))

    return LaunchDescription(launchs)
