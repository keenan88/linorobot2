from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    rviz_config_path = '/home/humble_ws/src/linorobot2_navigation/rviz/linorobot2_map_editor.rviz'

    tf_image_publisher = Node(
        package='linorobot2_localization',
        executable='slam_recording_publisher',
        parameters=[
            {'use_sim_time': False}
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': False}
        ]
    )

    map_server = Node(
        package = 'nav2_map_server',
        executable = 'map_server',
        namespace = "nav2",
        parameters = [
            '/home/humble_ws/src/linorobot2_navigation/config/map_editor.yaml',
            {
                'use_sim_time' : False,
            }
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace='nav2',
        parameters=[
            {'use_sim_time': False},
            '/home/humble_ws/src/linorobot2_navigation/config/map_editor.yaml',
        ]
    )

    ld = LaunchDescription()

    ld.add_action(map_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(tf_image_publisher)
    ld.add_action(rviz)

    return ld
