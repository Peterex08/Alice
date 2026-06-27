import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue # <--- Import adicionado

def generate_launch_description():
    pkg_path = get_package_share_directory('alice_xacro_desc')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_path, 'rviz', 'display.rviz')

    # Converte o Xacro usando um Command do Launch
    robot_description_content = Command(['xacro ', xacro_file])
    
    # Envolvendo o conteúdo em ParameterValue(..., value_type=str)
    params = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Habilita a janela com os sliders
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])