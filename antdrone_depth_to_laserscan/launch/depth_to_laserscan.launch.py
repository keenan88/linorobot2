import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    ld = launch.LaunchDescription()

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/humble_ws/src/antdrone_depth_to_laserscan/config/depth_to_laserscan.rviz'],
    )

    # depth to laserscan node reference: https://github.com/ros-perception/depthimage_to_laserscan/blob/ros2/launch/depthimage_to_laserscan-launch.py
    depth_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[
            # ('/cloud_in', '/front_rs/pointcloud_downsampled'),
            ('/cloud_in', '/front_rs/pointcloud_cropped'),
            ('/scan', '/front_rs/scan')
            # ('/depth', '/front_rs/front_rs/depth/image_rect_raw'),
            # ('/depth_camera_info', 'front_rs/front_rs/depth/camera_info')
        ],
        parameters = [
            '/home/humble_ws/src/antdrone_depth_to_laserscan/config/depth_to_laserscan.yaml'
        ]
    )
    
    ld.add_action(depth_to_scan)
    ld.add_action(rviz_node)

    return ld