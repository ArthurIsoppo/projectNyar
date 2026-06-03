import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vector_logic_node = Node(
        package='accel_logic_client',
        executable='vector_logic_client',
        name='vector_logic_client',
        output='screen'
    )

    return LaunchDescription([
        vector_logic_node
    ])
