# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    slam_config_path = '/home/humble_ws/src/linorobot2_navigation/config/slam.yaml'

    navigation_launch_path = '/home/humble_ws/src/linorobot2_navigation/launch/slam_navigation.launch.py'

    nav2_config_path = '/home/humble_ws/src/linorobot2_navigation/config/navigation.yaml'

    rviz_config_path = '/home/humble_ws/src/linorobot2_navigation/rviz/linorobot2_slam.rviz'

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_config_path,
            {'use_sim_time': True}
        ]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            'use_sim_time': "True",
            'params_file': nav2_config_path
        }.items()
    )

    wheel_odometry = Node(
        package='linorobot2_localization',
        executable='wheel_odometry',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    wheel_unraveller = Node(
        package='linorobot2_localization',
        executable='wheel_unraveller',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': True}
        ]
    )

    ld = LaunchDescription()

    ld.add_action(slam_toolbox)
    # ld.add_action(navigation)
    ld.add_action(wheel_odometry)
    ld.add_action(wheel_unraveller)
    ld.add_action(rviz)

    return ld
