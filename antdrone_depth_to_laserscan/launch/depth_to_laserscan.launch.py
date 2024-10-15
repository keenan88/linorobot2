import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    ld = launch.LaunchDescription()


    for realsense_placement in ['front_rs', 'left_rs', 'right_rs']:

        depth_to_scan = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name = realsense_placement + '_pointcloud_to_laserscan',
            remappings=[
                ('/cloud_in', '/' + realsense_placement + '/pointcloud_downsampled'),
                ('/scan', '/' + realsense_placement + '/scan')
            ],
            parameters = [
                '/home/humble_ws/src/antdrone_depth_to_laserscan/config/' + realsense_placement + '_pcl_to_laserscan.yaml'
            ]
        )

        ld.add_action(depth_to_scan)

    realsense_placement = 'rear_rs'

    depth_to_scan_rear_rs_left_side = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name = realsense_placement + '_left_pointcloud_to_laserscan',
        remappings=[
            ('/cloud_in', '/' + realsense_placement + '/pointcloud_downsampled'),
            ('/scan', '/' + realsense_placement + '/scan_left')
        ],
        parameters = [
            '/home/humble_ws/src/antdrone_depth_to_laserscan/config/' + realsense_placement + '_pcl_to_laserscan_left.yaml'
        ]
    )
    
    depth_to_scan_rear_rs_right_side = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name = realsense_placement + '_right_pointcloud_to_laserscan',
        remappings=[
            ('/cloud_in', '/' + realsense_placement + '/pointcloud_downsampled'),
            ('/scan', '/' + realsense_placement + '/scan_right')
        ],
        parameters = [
            '/home/humble_ws/src/antdrone_depth_to_laserscan/config/' + realsense_placement + '_pcl_to_laserscan_right.yaml'
        ]
    )

    scan_merger = Node(
        package='antdrone_depth_to_laserscan',
        executable='laser_scan_merger',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/humble_ws/src/antdrone_depth_to_laserscan/config/depth_to_laserscan.rviz'],
    )

    ld.add_action(depth_to_scan_rear_rs_left_side)
    ld.add_action(depth_to_scan_rear_rs_right_side)
    # ld.add_action(rviz_node)

    ld.add_action(scan_merger)

    return ld