"""
Levanta todo con un solo comando:
Gazebo con la pista + robot + robot_state_publisher + bridge + RViz2 + odometría
 
    ros2 launch mi_robot_diferencial mi_robot.launch.py                       # pista de competencia
    ros2 launch mi_robot_diferencial mi_robot.launch.py pista:=propia         # pista propia
"""
import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
 
# ===== Pistas disponibles: archivo del world y punto de spawn (x, y, yaw) =====
PISTAS = {
    'competencia': {'world': 'pista_competencia.sdf', 'spawn': (-0.60, 0.0, 0.0)},
    'propia':      {'world': 'pista.sdf',             'spawn': (-2.60, -2.4, 0.0)},  # se reemplaza en el Paso 8
}
 
 
def lanzar(context):
    pista = context.launch_configurations['pista']
    if pista not in PISTAS:
        raise RuntimeError(f"pista:='{pista}' no existe. Opciones: {list(PISTAS)}")
    cfg = PISTAS[pista]
 
    pkg = get_package_share_directory('mi_robot_diferencial')
    xacro_path = os.path.join(pkg, 'urdf', 'mi_robot.urdf.xacro')
    world_path = os.path.join(pkg, 'worlds', cfg['world'])
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'mi_robot.rviz')
    x, y, yaw = cfg['spawn']
 
    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)
 
    # 1) Gazebo con la pista (-r = arranca corriendo)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v4 {world_path}'}.items(),
    )
 
    # 2) robot_state_publisher: /robot_description y TF del robot
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )
 
    # 3) Spawn en el punto de salida de la pista elegida
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'mi_robot',
                   '-x', str(x), '-y', str(y), '-z', '0.05', '-Y', str(yaw)],
        output='screen',
    )
 
    # 4) Bridge ROS 2 <-> Gazebo
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )
 
    # 5) RViz2
    rviz = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )
 
    # 6) Odometría por cinemática de las llantas
    odom_ruedas = Node(
        package='mi_robot_cinematica', executable='odom_ruedas',
        parameters=[{'use_sim_time': True}], output='screen',
    )
 
    return [gazebo, robot_state_publisher, spawn, bridge, rviz, odom_ruedas]
 
 
def generate_launch_description():
    models_dir = os.path.join(get_package_share_directory('mi_robot_diferencial'), 'models')
    return LaunchDescription([
        DeclareLaunchArgument('pista', default_value='competencia',
                              description='competencia | propia'),
        # Para que Gazebo encuentre model://pista_competencia
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_dir),
        OpaqueFunction(function=lanzar),
    ])
