from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rviz_config_path = '/home/humble_ws/src/linorobot2_navigation/rviz/linorobot2_slam.rviz'
    
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        namespace='nav2',
        parameters=[
            {'use_sim_time': True}, 
            '/home/humble_ws/src/linorobot2_navigation/config/controller.yaml'
        ],
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        namespace='nav2',
        parameters=[
            {'use_sim_time': True},
            '/home/humble_ws/src/linorobot2_navigation/config/planner.yaml'
        ],
    )

    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        namespace='nav2',
        parameters=[
            {'use_sim_time': True}, 
            '/home/humble_ws/src/linorobot2_navigation/config/behaviors.yaml'
        ],
    )

    bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        namespace='nav2',
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': True},
            '/home/humble_ws/src/linorobot2_navigation/config/bt.yaml'
        ],
    )

    map_server = Node(
        package = 'nav2_map_server',
        executable = 'map_server',
        namespace = "nav2",
        parameters = [
            '/home/humble_ws/src/linorobot2_navigation/config/map_server.yaml',
            {'use_sim_time' : True}
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace='nav2',
        parameters=[
            {'use_sim_time': True},
            '/home/humble_ws/src/linorobot2_navigation/config/lifecycle_manager.yaml',
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

    keepout_filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        parameters=[
            '/home/humble_ws/src/linorobot2_navigation/config/filter_mask_server.yaml',
            {
                'use_sim_time': True,
            }
        ]
    )

    keepout_filter_map_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        parameters=[
            '/home/humble_ws/src/linorobot2_navigation/config/filter_mask_server.yaml',
            {
                'use_sim_time': True,
            }
        ]
    )

    keepout_filter_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        parameters=[
            {
                'use_sim_time': True,
            },
            '/home/humble_ws/src/linorobot2_navigation/config/lifecycle_manager_costmap_filters.yaml'
        ]
    )

    navigate_to_named_pos_server = Node(
        package='linorobot2_navigation',
        executable='navigate_to_named_pose', 
        parameters=[
            {
                'named_poses_file' : '/home/humble_ws/src/linorobot2_navigation/config/named_poses.yaml',
                'use_sim_time': True,
            }
        ]
    )

    wheel_odometry = Node(
        package='linorobot2_localization',
        executable='wheel_odometry', 
        parameters=[
            {
                'use_sim_time' : True
            }
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        namespace='nav2',
        parameters=[
            '/home/humble_ws/src/linorobot2_navigation/config/amcl.yaml',
            {
            'use_sim_time': True,
            }
        ]
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            '/home/humble_ws/src/linorobot2_localization/config/ekf.yaml',
            {
            'use_sim_time': True,
            }
        ]
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

    ld = LaunchDescription()

    # ld.add_action(controller)
    # ld.add_action(planner)
    # ld.add_action(behaviors)
    # ld.add_action(bt)
    # ld.add_action(map_server)
    # ld.add_action(lifecycle_manager)
    ld.add_action(rviz)
    ld.add_action(wheel_unraveller)
    ld.add_action(wheel_odometry)
    # ld.add_action(amcl)



    return ld


