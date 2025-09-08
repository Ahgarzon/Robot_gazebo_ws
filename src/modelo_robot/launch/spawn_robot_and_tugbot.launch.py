# spawn_robot_and_tugbot.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Ruta al share de tu paquete “modelo_robot”
    pkg_share = get_package_share_directory('modelo_robot')

    # 1) El path a tu file .world que creaste antes
    world_file = os.path.join(pkg_share, 'worlds', 'robot_and_tugbot.world')

    # 2) El path a tu URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'Seguidor_linea_robot.urdf')

    # -------------------------------------------------------------------------
    # Paso 1: Ejecutar Gazebo con tu world
    launch_gazebo = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py',
            'world:={}'.format(world_file)
        ],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # Paso 2: Después de X segundos, hacer spawn del URDF
    # Le damos 5 segundos a Gazebo para que cargue todo el mundo (sun, ground_plane, tugbot).
    # Ajusta este tiempo si tu máquina es muy lenta (p. ej., 7 ó 10 s).
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                arguments=[
                    '-entity', 'mi_seguidor',          # Nombre que le das a tu robot en Gazebo
                    '-file', urdf_file,
                    '-x', '0',                          # Ajusta X, Y, Z para ubicarlo donde quieras
                    '-y', '-2',
                    '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        launch_gazebo,
        spawn_robot,
    ])
