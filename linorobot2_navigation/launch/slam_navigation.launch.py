from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
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

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace='nav2',
        parameters=[
            {'use_sim_time': True},
            '/home/humble_ws/src/linorobot2_navigation/config/slam_lifecycle_manager.yaml',
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

    ld.add_action(controller)
    ld.add_action(planner)
    ld.add_action(behaviors)
    ld.add_action(bt)
    ld.add_action(lifecycle_manager)
    ld.add_action(wheel_unraveller)
    ld.add_action(wheel_odometry)



    return ld


