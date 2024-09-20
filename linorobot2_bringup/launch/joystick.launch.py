from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joystick_driver = Node(
        package = 'joy', 
        executable = 'joy_node', 
        name = 'joy_node',
        parameters = [
            '/home/humble_ws/src/linorobot2_joystick/config/joy_node_config.yaml',
            {'dev': '/dev/input/js0'}
        ]
    )
    
    joystick_twist = Node(
        package = 'teleop_twist_joy', 
        executable = 'teleop_node',
        name = 'teleop_twist_joy_node',
        parameters = [
            '/home/humble_ws/src/linorobot2_joystick/config/teleop_twist_joy_config.yaml'
        ]
    )    

    ld = LaunchDescription()
    ld.add_action(joystick_driver)
    ld.add_action(joystick_twist)
    
    return ld
