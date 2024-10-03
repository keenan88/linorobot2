from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    slam_config_path = '/home/humble_ws/src/linorobot2_navigation/config/slam.yaml'

    navigation_launch_path = '/home/humble_ws/src/linorobot2_navigation/launch/slam_navigation.launch.py'

    rviz_config_path = '/home/humble_ws/src/linorobot2_navigation/rviz/linorobot2_slam.rviz'

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace = 'nav2',
        parameters=[
            slam_config_path,
            {'use_sim_time': True}
        ],
        remappings=[("/map", "/nav2/map")],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
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
    ld.add_action(navigation)
    
    
    ld.add_action(rviz)

    return ld
