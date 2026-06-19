import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'alice_xacro_desc'
    pkg_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    # 1. Converter Xacro para URDF e publicar o estado do robô
    robot_description_content = Command(['xacro ', xacro_file])
    params = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 2. Iniciar o Gazebo com um mundo vazio
    gazebo_pkg_dir = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg_dir, 'launch', 'gazebo.launch.py'))
    )

    # 3. Spawnar o robô no Gazebo usando o tópico gerado pelo Xacro
    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'alice_robot'],
        output='screen'
    )

    # 4. TF Estático (Substitui o tf_footprint_base do ROS 1)
    # A ordem no ROS 2 é: x y z roll pitch yaw frame_id child_frame_id
    node_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )

    # 5. Publicador fake de calibração (Substitui o rostopic pub do ROS 1)
    fake_calibration = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/calibrated', 'std_msgs/msg/Bool', '{data: true}'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo_launch,
        node_spawn_entity,
        node_static_tf,
        fake_calibration
    ])