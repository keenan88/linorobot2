import launch
from launch_ros.actions import Node
import os

# Robot has a realsense on front and back, each with own namespacing and frame names
def generate_launch_description():

    use_sim = os.environ.get("USE_SIM")
    use_sim = use_sim == "True"

    ld = launch.LaunchDescription()

    # "left_rs", "right_rs"
    for realsense_placement in ["front_rs", "rear_rs"]:

        pointcloud_cropper = Node(
            package='pcl_ros',
            executable='filter_crop_box_node',
            name = realsense_placement + "_pointcloud_cropper",
            output='screen',
            parameters=[
                '/home/humble_ws/src/linorobot2_pcl/config/' + realsense_placement + '_filter_crop_box.yaml',
                {
                    'use_sim_time' : use_sim,
                    "input_frame": realsense_placement + '_depth_optical_frame',
                    "output_frame": realsense_placement + '_depth_optical_frame'
                },
            ],
            remappings=[
                ('input', '/' + realsense_placement + '/' + realsense_placement + '/depth/color/points'),
                ('output', realsense_placement + '/pointcloud_cropped')
            ]
        )

        pointcloud_downsampler = Node(
            package='pcl_ros',
            executable='filter_voxel_grid_node',
            name = realsense_placement + "_pointcloud_downsampler",
            output='screen',
            parameters=[
                '/home/humble_ws/src/linorobot2_pcl/config/' + realsense_placement + '_pcl_downsample_filter.yaml',
                {'use_sim_time' : use_sim}
            ],
            remappings=[
                ('input', realsense_placement + '/pointcloud_cropped'),
                ('output', realsense_placement + '/pointcloud_downsampled')
            ]
        )

        ld.add_action(pointcloud_cropper)
        ld.add_action(pointcloud_downsampler)


    # bridge_out_pcl = Node(
    #     package="domain_bridge",
    #     executable="domain_bridge",
    #     name = "bridge_out_pcl",
    #     arguments = ['/home/humble_ws/src/linorobot2_pcl/config/bridge_out_pcl.yaml']
    # )

    # # ld.add_action(rviz_node)
    # if not use_sim:
    #     ld.add_action(bridge_out_pcl)
    
    
    return ld