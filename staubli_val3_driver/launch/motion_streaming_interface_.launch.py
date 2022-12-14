# Copyright (c) 2022 Ivo Dekker ACRO Diepenbeek KULeuven

# Permission is hereby granted, free of charge, to any person
# obtaining a copy of this software and associated documentation
# files (the "Software"), to deal in the Software without
# restriction, including without limitation the rights to use,
# copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following
# conditions:

# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
# OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_ip_launch_arg = DeclareLaunchArgument(
        "robot_ip",
        description="IP address of the robot",
    )
    robot_ip = LaunchConfiguration(
        "robot_ip",
    )
    
    robot_state_node = Node(
        package="industrial_robot_client",
        executable="robot_state",
        name="robot_state",
        parameters=[
            {"robot_ip_address": robot_ip},
        ],
    )
    motion_streaming_interface_node = Node(
        package="industrial_robot_client",
        executable="motion_streaming_interface",
        name="motion_streaming_interface",
        parameters=[
            {"robot_ip_address": robot_ip},
        ]
    )
    joint_trajectory_action_node = Node(
        package="industrial_robot_client",
        executable="joint_trajectory_action",
        name="joint_trajectory_action",
    )

    return LaunchDescription(
        [
            robot_ip_launch_arg,
            robot_state_node,
            motion_streaming_interface_node,
            joint_trajectory_action_node
        ]
    )