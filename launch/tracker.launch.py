from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    parameter = os.path.join(get_package_share_directory('earth_velocity_controller'), 'config', 'tracker.yaml')

    converter_node = Node(
        package='earth_velocity_controller',
        executable='velocity_tracker',
        name='velocity_tracker',
        output='screen',
        parameters=[parameter]
    )


    ld.add_action(converter_node)

    return ld