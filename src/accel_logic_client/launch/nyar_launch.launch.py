import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Configuração do nó lógico (Cérebro - roda na Workstation)
    vector_logic_node = Node(
        package='accel_logic_client',
        executable='vector_logic_client',
        name='vector_logic_client',
        output='screen'
    )
    
    # Cria a descrição de lançamento contendo apenas o nó da Workstation
    return LaunchDescription([
        vector_logic_node
    ])