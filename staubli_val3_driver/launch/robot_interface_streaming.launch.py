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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    #Declare all necessary parameters
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
        "robot_ip",
        description="IP address of the robot",
        )
    )
    #Init arguments
    robot_ip = LaunchConfiguration(
        "robot_ip",
    )

    #Include all required launch files
    launch_files = []
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/robot_state.launch.py"]),
            launch_arguments={
                "robot_ip": robot_ip,
            }.items()
        )
    )
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/motion_streaming_interface.launch.py"]),
            launch_arguments={
                "robot_ip": robot_ip,
            }.items(),
        )
    )
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/io_interface.launch.py"]),
            launch_arguments={
                "robot_ip": robot_ip,
            }.items(),
        )
    )
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/system_interface.launch.py"]),
            launch_arguments={
                "robot_ip": robot_ip,
            }.items(),
        )
    )

    #Nodes
    nodes = []
    nodes.append(
            Node(
            package="industrial_robot_client",
            executable="joint_trajectory_action",
        )
    )

    return LaunchDescription(
         declared_arguments + 
         launch_files +
         nodes
    )
