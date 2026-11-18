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
    'propia':      {'world': 'pista_propia.sdf', 'spawn': (-0.60, 0.0, 0.0)},
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
 
    # 7) Mux + AEB: lleva /cmd_vel_teleop o /cmd_vel_auto a /cmd_vel pasando por el AEB
    mux_aeb = Node(
        package='mi_robot_control', executable='mux_aeb',
        parameters=[{'use_sim_time': True, 'modo': context.launch_configurations['modo']}],
        output='screen',
    )
 
    # 7b) Cronómetro de vueltas y error de cierre (pose estimada vs pose real)
    vueltas = Node(
        package='mi_robot_cinematica', executable='vueltas',
        parameters=[{'use_sim_time': True, 'separacion_ruedas': 0.181}], output='screen',
    )

    nodos = [gazebo, robot_state_publisher, spawn, bridge, rviz, odom_ruedas, mux_aeb, vueltas]

    # 9) Modo teleoperado -> /cmd_vel_teleop
    if context.launch_configurations['modo'] == 'teleop':
        if context.launch_configurations['control'] == 'ps':
            joy_cfg = os.path.join(pkg, 'config', 'joy_ps5.yaml')
            nodos.append(Node(package='joy', executable='joy_node',
                              parameters=[joy_cfg, {'use_sim_time': True}]))
            nodos.append(Node(package='mi_robot_control', executable='control_ps',
                              parameters=[joy_cfg, {'use_sim_time': True}]))
        else:
            # El teclado necesita su propia ventana para leer las teclas
            nodos.append(Node(package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
                              prefix='xterm -fa Monospace -fs 12 -e',
                              parameters=[{'speed': 1.0, 'turn': 2.0}],
                              remappings=[('/cmd_vel', '/cmd_vel_teleop')]))

    # 10) Obstáculo para demostrar el AEB (caja en la recta principal, a 3 m de la salida)
    if context.launch_configurations['obstaculo'] == 'true':
        caja = ('<sdf version="1.9"><model name="obstaculo"><static>true</static><link name="l">'
                '<collision name="c"><geometry><box><size>0.2 0.2 0.3</size></box></geometry></collision>'
                '<visual name="v"><geometry><box><size>0.2 0.2 0.3</size></box></geometry>'
                '<material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material></visual>'
                '</link></model></sdf>')
        nodos.append(Node(package='ros_gz_sim', executable='create', output='screen',
                          arguments=['-name', 'obstaculo', '-string', caja,
                                     '-x', '3.0', '-y', '0.0', '-z', '0.15']))

    # 8) Modo autónomo: seguimiento de pared -> /cmd_vel_auto
    if context.launch_configurations['modo'] == 'auto':
        nodos.append(Node(
            package='mi_robot_control', executable='seguidor_pared',
            parameters=[{'use_sim_time': True,
                         'v_max': float(context.launch_configurations['v_max'])}],
            output='screen',
        ))
    return nodos
 
 
def generate_launch_description():
    models_dir = os.path.join(get_package_share_directory('mi_robot_diferencial'), 'models')
    return LaunchDescription([
        DeclareLaunchArgument('modo', default_value='teleop', description='teleop | auto'),
        DeclareLaunchArgument('v_max', default_value='1.0', description='velocidad máxima del modo auto [m/s]'),
        DeclareLaunchArgument('control', default_value='ps', description='ps | teclado (solo en modo teleop)'),
        DeclareLaunchArgument('obstaculo', default_value='false', description='true = caja en la recta para probar el AEB'),
        DeclareLaunchArgument('pista', default_value='competencia',
                              description='competencia | propia'),
        # Para que Gazebo encuentre model://pista_competencia
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_dir),
        OpaqueFunction(function=lanzar),
    ])
