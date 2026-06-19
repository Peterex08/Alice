import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'alice_xacro_desc'
    pkg_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    # 1. Converter Xacro para URDF
    robot_description_content = Command(['xacro ', xacro_file])
    params = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Publicador da "Planta Baixa" (Links)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # ---------------------------------------------------------
    # A SOLUÇÃO: Publicador mudo dos ângulos (Zera as juntas)
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': True},
            params  # <--- Crucial: Ele precisa ler o URDF para achar os nomes das juntas
        ]
    )
    # ---------------------------------------------------------

    # 2. INJEÇÃO FORÇADA DO GAZEBO NO DOCKER (Headless puro com plugins do ROS)
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '--verbose'],
        output='screen'
    )

    # 3. Spawnar o robô 
    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'alice_robot'],
        output='screen'
    )

    # 4. RViz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # Lembre-se de sempre adicionar novos nós aqui no return!
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher,  # <--- Nó adicionado na fila de execução
        gzserver_cmd,
        node_spawn_entity,
        node_rviz
    ])