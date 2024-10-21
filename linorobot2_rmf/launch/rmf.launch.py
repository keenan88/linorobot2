from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    dummy_ingestor = Node(
        package='linorobot2_rmf',
        executable='dummy_ingestor',
        parameters=[
            {'use_sim_time': True}
        ],
    )

    dummy_dispenser = Node(
        package='linorobot2_rmf',
        executable='dummy_ingestor',
        parameters=[
            {'use_sim_time': True}
        ],
    )

    linorobot_rmf_client = Node(
        package='linorobot2_rmf_client',
        executable='linorobot2_rmf_client',
        parameters=[
            {'use_sim_time': True}
        ],
    )

    linorobot2_rmf = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            '/home/humble_ws/src/linorobot2_rmf/launch/greenhouse.launch.xml'
        )
    )
    

    ld = LaunchDescription()

    ld.add_action(dummy_ingestor)
    ld.add_action(dummy_dispenser)
    ld.add_action(linorobot_rmf_client)
    ld.add_action(linorobot2_rmf)

    return ld
