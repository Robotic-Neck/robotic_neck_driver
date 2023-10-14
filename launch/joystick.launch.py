# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# https://github.com/ros2/teleop_twist_joy/tree/humble
# https://github.com/ros-teleop/teleop_tools/tree/master/joy_teleop

# NOTE: This launch it is not tested with the xbox controller

"""
    Launch file for joystick control of the robotic neck by joy_teleop pkg
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_processes = []

    launch_processes.append(Node(package='joy', executable='joy_node'))
    launch_processes.append(Node(package='robotic_neck_driver', executable='joystick', name='robotic_neck_joy'))
    
    return LaunchDescription(launch_processes)
