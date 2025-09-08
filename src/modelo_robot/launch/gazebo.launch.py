# #!/usr/bin/env python3
# """
# • Arranca gzserver con el mundo “house.world”
# • Conecta la GUI -gzclient-
# • Publica el URDF como /robot_description
# • Spawnea el robot una vez que el servidor está listo
# • Inicia RViz2 con tu configuración
# """

# from __future__ import annotations
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import (
#     DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction)
# from launch.event_handlers import OnProcessStart
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description() -> LaunchDescription:
#     pkg_share    = get_package_share_directory('modelo_robot')
#     use_sim_time = LaunchConfiguration('use_sim_time')

#     # ────────────── 1) gzserver: abre el mundo con la casa ─────────────
#     world_file = os.path.join(pkg_share, 'worlds', 'house.world')
#     gzserver = ExecuteProcess(
#         cmd=[
#             'gzserver', world_file, '--verbose',
#             '-s', 'libgazebo_ros_factory.so',
#             '-s', 'libgazebo_ros_init.so'
#         ],
#         output='screen')

#     # ────────────── 2) gzclient cuando el server esté activo ───────────
#     gzclient = RegisterEventHandler(
#         OnProcessStart(
#             target_action=gzserver,
#             on_start=[ExecuteProcess(cmd=['gzclient'], output='screen')]))

#     # ────────────── 3) robot_description a partir del URDF ─────────────
#     urdf_path = os.path.join(pkg_share, 'urdf', 'Seguidor_linea_robot.urdf')
#     with open(urdf_path, 'r') as urdf:
#         robot_desc = urdf.read().replace('<?xml version="1.0" encoding="utf-8"?>', '')

#     state_pub = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{'use_sim_time': use_sim_time,
#                      'robot_description': robot_desc}])

#     # ────────────── 4) RViz2 opcional ──────────────────────────────────
#     rviz_cfg = os.path.join(pkg_share, 'rviz', 'default.rviz')
#     rviz2 = Node(
#         package='rviz2', executable='rviz2',
#         arguments=['-d', rviz_cfg],
#         parameters=[{'use_sim_time': use_sim_time}],
#         output='screen')

#     # ────────────── 5) spawnea el URDF en Gazebo (3 s después) ─────────
#     spawn = Node(
#         package='gazebo_ros', executable='spawn_entity.py',
#         arguments=['-topic', '/robot_description',
#                    '-entity', 'Seguidor_linea_robot',
#                    '-x', '0', '-y', '0', '-z', '0.05'],
#         output='screen')

#     spawn_handler = RegisterEventHandler(
#         OnProcessStart(target_action=gzserver,
#                        on_start=[TimerAction(period=3.0, actions=[spawn])]))

#     # ────────────── LaunchDescription ──────────────────────────────────
#     return LaunchDescription([
#         DeclareLaunchArgument('use_sim_time',
#                               default_value='true',
#                               description='Use simulation (Gazebo) clock'),
#         gzserver,
#         gzclient,
#         state_pub,
#         rviz2,
#         spawn_handler
#     ])
# #!/usr/bin/env python3
# """
# Lanza Gazebo (server + GUI) en vacío, publica el URDF como /robot_description,
# spawnea el robot y arranca RViz2.
# """

# from __future__ import annotations
# import os
# import re

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import (
#     DeclareLaunchArgument,
#     ExecuteProcess,
#     RegisterEventHandler,
#     TimerAction,
# )
# from launch.event_handlers import OnProcessStart
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description() -> LaunchDescription:
#     pkg_share = get_package_share_directory("modelo_robot")
#     use_sim_time = LaunchConfiguration("use_sim_time")

#     # 1) gzserver SIN world (usa empty world por defecto)
#     gzserver = ExecuteProcess(
#         cmd=[
#             "gzserver",
#             "--verbose",
#             "-s",
#             "libgazebo_ros_init.so",
#             "-s",
#             "libgazebo_ros_factory.so",
#         ],
#         output="screen",
#     )

#     # 2) gzclient cuando gzserver esté activo
#     gzclient = RegisterEventHandler(
#         OnProcessStart(
#             target_action=gzserver,
#             on_start=[
#                 ExecuteProcess(
#                     cmd=["gzclient"],
#                     output="screen",
#                 )
#             ],
#         )
#     )

#     # 3) Publicar URDF sin cabecera <?xml encoding="UTF-8"?>
#     urdf_path = os.path.join(pkg_share, "urdf", "Seguidor_linea_robot.urdf")
#     with open(urdf_path, "r", encoding="utf-8") as inf:
#         xml_str = inf.read()
#     # Eliminar línea '<?xml version="1.0" encoding="UTF-8"?>' si está presente
#     xml_str = re.sub(r"<\?xml[^>]*\?>\s*", "", xml_str)

#     robot_state_pub = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         parameters=[
#             {
#                 "use_sim_time": use_sim_time,
#                 "robot_description": xml_str,
#             }
#         ],
#         output="screen",
#     )

#     # 4) RViz2 (opcional)
#     rviz_cfg = os.path.join(pkg_share, "rviz", "default.rviz")
#     rviz2 = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="screen",
#         arguments=["-d", rviz_cfg],
#         parameters=[{"use_sim_time": use_sim_time}],
#     )

#     # 5) spawn_entity.py  → esperar 3 s a que gzserver esté listo
#     spawn_robot = Node(
#         package="gazebo_ros",
#         executable="spawn_entity.py",
#         arguments=[
#             "-topic",
#             "/robot_description",
#             "-entity",
#             "Seguidor_linea_robot",
#             "-x",
#             "0",
#             "-y",
#             "0",
#             "-z",
#             "0.05",
#         ],
#         output="screen",
#     )
#     spawn_handler = RegisterEventHandler(
#         OnProcessStart(
#             target_action=gzserver,
#             on_start=[TimerAction(period=3.0, actions=[spawn_robot])],
#         )
#     )

#     return LaunchDescription(
#         [
#             DeclareLaunchArgument(
#                 "use_sim_time",
#                 default_value="true",
#                 description="Use simulation (Gazebo) clock",
#             ),
#             gzserver,
#             gzclient,
#             robot_state_pub,
#             rviz2,
#             spawn_handler,
#         ]
#     )

# import os
  
# from ament_index_python.packages import get_package_share_directory
 
# from launch_ros.parameter_descriptions import ParameterValue
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
# from launch.conditions import IfCondition
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch.actions import RegisterEventHandler
# from launch.event_handlers import OnProcessExit
  
# def generate_launch_description():
 
 
#     model_arg = DeclareLaunchArgument(name='model', description='Absolute path to robot urdf file')
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
#     use_sim_time = LaunchConfiguration('use_sim_time') 
#     package_name = 'modelo_robot'
#     pkg_share = FindPackageShare(package=package_name).find(package_name)
#     pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros') 

#     #world_file_path = 'world.world'
#     #world = LaunchConfiguration('world')
#     #world_path = os.path.join(pkg_share, 'worlds',  world_file_path)

#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         name='use_sim_time',
#         default_value='true',
#         description='Use simulation (Gazebo) clock if true'
#         )

#     robot_name_in_model = 'Seguidor_linea_robot'

#     # Get URDF via xacro

#     urdf_file_name = 'Seguidor_linea_robot.urdf'
#     urdf = os.path.join(
#         get_package_share_directory('modelo_robot'),
#         'urdf',
#         urdf_file_name
#         )
#     with open(urdf, 'r') as infp:
#         robot_desc = infp.read()

#     robot_description = {"robot_description": robot_desc}
 
 
#     #rivz2
#     rviz2 = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='log',
#         parameters=[{'use_sim_time': use_sim_time}],
#     )
 
#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters= [{'use_sim_time': use_sim_time, 'robot_description': robot_desc}] #[{'use_sim_time': use_sim_time, "robot_description": robot_description_content}],
#     )

#     start_joint_state_publisher_cmd = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         parameters=[{'use_sim_time': use_sim_time}],
#         name='joint_state_publisher',
#     )
 

#     '''declare_world_cmd = DeclareLaunchArgument(
#         name='world',
#         default_value=world_path,
#         description='Full path to the world model file to load'
#         ) '''
 
#     #spawn the robot 
#     spawn = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=["-topic", "/robot_description", 
#                     "-entity", robot_name_in_model,
#                     "-x", '0.0',
#                     "-y", '0.0',
#                     "-z", '0.05',
#                     "-Y", '0.0']
#     )


#     gazebo = ExecuteProcess(
#         cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', 
#         '-s', 'libgazebo_ros_init.so'], output='screen',
#         )

     
#     return LaunchDescription([
#     declare_use_sim_time_cmd,
#     rviz2,
#     spawn,
#     robot_state_publisher_node,
#     gazebo
# ])

# #!/usr/bin/env python3
# """
# Lanza todo en un único Launch:
#   1) gzserver cargando el mundo “house.world”
#   2) gzclient (la GUI) se inicia solo cuando gzserver esté arriba
#   3) robot_state_publisher publica tu URDF en /robot_description
#   4) spawn_entity.py “spawnea” tu robot en Gazebo (con un pequeño delay)
#   5) RViz2 se abre con tu configuración (default.rviz)
# """

# from __future__ import annotations
# import os
# import re

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import (
#     DeclareLaunchArgument,
#     ExecuteProcess,
#     RegisterEventHandler,
#     TimerAction
# )
# from launch.event_handlers import OnProcessStart
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description() -> LaunchDescription:
#     # ---------------------------------------------------------------------
#     # 1) Parámetros básicos
#     # ---------------------------------------------------------------------
#     pkg_share     = get_package_share_directory('modelo_robot')
#     use_sim_time  = LaunchConfiguration('use_sim_time')

#     # ---------------------------------------------------------------------
#     # 2) Declarar el argumento `use_sim_time`
#     # ---------------------------------------------------------------------
#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         name='use_sim_time',
#         default_value='true',
#         description='Usar el reloj de simulación de Gazebo'
#     )

#     # ---------------------------------------------------------------------
#     # 3) gzserver: carga el mundo “house.world”
#     # ---------------------------------------------------------------------
#     #    - Asume que house.world está en <modelo_robot>/worlds/house.world
#     world_path = os.path.join(pkg_share, 'worlds', 'house.world')
#     gzserver = ExecuteProcess(
#         cmd=[
#             'gzserver', world_path, '--verbose',
#             '-s', 'libgazebo_ros_factory.so',
#             '-s', 'libgazebo_ros_init.so'
#         ],
#         output='screen'
#     )

#     # ---------------------------------------------------------------------
#     # 4) gzclient: arranca la GUI **solo** después de que gzserver esté arriba
#     # ---------------------------------------------------------------------
#     gzclient = RegisterEventHandler(
#         OnProcessStart(
#             target_action=gzserver,
#             on_start=[
#                 ExecuteProcess(
#                     cmd=['gzclient'],
#                     output='screen'
#                 )
#             ]
#         )
#     )

#     # ---------------------------------------------------------------------
#     # 5) robot_state_publisher: leer URDF, limpiar la cabecera XML (encoding)
#     # ---------------------------------------------------------------------
#     urdf_path = os.path.join(pkg_share, 'urdf', 'Seguidor_linea_robot.urdf')
#     # Abrimos con UTF-8 y eliminamos cualquier <?xml ... encoding="..."?> para evitar
#     # el error "Unicode strings with encoding declaration are not supported"
#     with open(urdf_path, 'r', encoding='utf-8') as infp:
#         xml_str = infp.read()

#     # – Quitar cabeceras <?xml version="1.0" encoding="UTF-8"?> (comillas dobles)
#     xml_str = re.sub(r'<\?xml\s+version="1\.0"\s+encoding="[^"]*"\s*\?>\s*', '', xml_str)
#     # – Quitar cabeceras <?xml version='1.0' encoding='UTF-8'?> (comillas simples)
#     xml_str = re.sub(r"<\?xml\s+version='1\.0'\s+encoding='[^']*'\s*\?>\s*", '', xml_str)
#     # – Atrapamos cualquier otra línea <?xml ... ?> sobrante
#     xml_str = re.sub(r'<\?xml[^>]*\?>\s*', '', xml_str)

#     robot_description_content = xml_str

#     robot_state_pub_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{
#             'use_sim_time': use_sim_time,
#             'robot_description': robot_description_content
#         }],
#         output='screen'
#     )

#     # ---------------------------------------------------------------------
#     # 6) spawn_entity: “spawnea” tu robot en Gazebo (3 segundos de delay)
#     # ---------------------------------------------------------------------
#     #    - Leemos de /robot_description
#     #    - Lo posicionamos en (0, 0, 0.05)
#     spawn_entity = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=[
#             '-topic', '/robot_description',
#             '-entity', 'Seguidor_linea_robot',
#             '-x', '0', '-y', '0', '-z', '0.05'
#         ],
#         output='screen'
#     )

#     #   Lo hacemos “TimerAction” para que espere 3 s después de que gzserver termine de iniciar
#     spawn_handler = RegisterEventHandler(
#         OnProcessStart(
#             target_action=gzserver,
#             on_start=[TimerAction(period=3.0, actions=[spawn_entity])]
#         )
#     )

#     # ---------------------------------------------------------------------
#     # 7) RViz2: opcional, arranca RViz con tu archivo default.rviz
#     # ---------------------------------------------------------------------
#     rviz_config_file = os.path.join(pkg_share, 'rviz', 'default.rviz')
#     rviz2_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='screen',
#         arguments=['-d', rviz_config_file],
#         parameters=[{'use_sim_time': use_sim_time}]
#     )

#     # ---------------------------------------------------------------------
#     # 8) Construir el LaunchDescription final
#     # ---------------------------------------------------------------------
#     ld = LaunchDescription()

#     # 8.1) Añadir argumento de sim_time
#     ld.add_action(declare_use_sim_time_cmd)

#     # 8.2) Añadir gzserver + gzclient
#     ld.add_action(gzserver)
#     ld.add_action(gzclient)

#     # 8.3) Añadir robot_state_publisher
#     ld.add_action(robot_state_pub_node)

#     # 8.4) Añadir spawn_entity (con su handler)
#     ld.add_action(spawn_handler)

#     # 8.5) Añadir RViz2
#     ld.add_action(rviz2_node)

#     return ld

# #!/usr/bin/env python3
# """
# • Arranca gzserver con el mundo “house.world”
# • Al arrancar gzserver, lanza gzclient
# • Publica el URDF en /robot_description (robot_state_publisher + joint_state_publisher)
# • Espera 3 s tras levantar gzserver y spawnea el robot
# • Abre RViz2 (sin necesidad de un default.rviz específico)
# """

# from __future__ import annotations
# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
# from launch.event_handlers import OnProcessStart
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description() -> LaunchDescription:
#     pkg_share = get_package_share_directory('modelo_robot')
#     use_sim_time = LaunchConfiguration('use_sim_time')

#     # ─────────────── Declare Launch Argument ───────────────
#     declare_use_sim_time = DeclareLaunchArgument(
#         name='use_sim_time',
#         default_value='true',
#         description='Use simulation (Gazebo) clock if true'
#     )

#     # ─────────────── 1) gzserver (con world) ────────────────
#     world_file = os.path.join(pkg_share, 'worlds', 'house.world')
#     gzserver = ExecuteProcess(
#         cmd=[
#             'gzserver', world_file, '--verbose',
#             '-s', 'libgazebo_ros_factory.so',
#             '-s', 'libgazebo_ros_init.so'
#         ],
#         output='screen',
#     )

#     # ─── 2) gzclient apenas gzserver esté arriba ───
#     gzclient = RegisterEventHandler(
#         OnProcessStart(
#             target_action=gzserver,
#             on_start=[ ExecuteProcess(cmd=['gzclient'], output='screen') ]
#         )
#     )

#     # ─── 3) Cargar el URDF (quitamos la cabecera <?xml … encoding="utf-8"?>) ───
#     urdf_path = os.path.join(pkg_share, 'urdf', 'Seguidor_linea_robot.urdf')
#     with open(urdf_path, 'r') as infp:
#         urdf_contents = infp.read().replace('<?xml version="1.0" encoding="utf-8"?>', '')

#     # ─── 4) robot_state_publisher para publicar /robot_description y TFs ───
#     rsp_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{
#             'use_sim_time': use_sim_time,
#             'robot_description': urdf_contents
#         }],
#         output='screen'
#     )

#     # ─── 5) joint_state_publisher (publica joint_states vacíos) ───
#     jsp_node = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         parameters=[{ 'use_sim_time': use_sim_time }],
#         output='screen'
#     )

#     # ─── 6) Spawn del robot (3 s después de arrancar gzserver) ───
#     spawn_robot = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=[
#             '-topic', '/robot_description',
#             '-entity', 'Seguidor_linea_robot',
#             '-x', '0.0', '-y', '0.0', '-z', '0.05',
#             '-Y', '0.0'
#         ],
#         output='screen'
#     )
#     spawn_handler = RegisterEventHandler(
#         OnProcessStart(
#             target_action=gzserver,
#             on_start=[ TimerAction(period=3.0, actions=[ spawn_robot ]) ]
#         )
#     )

#     # ─── 7) RViz2 (sin configuración externa; arranca con su layout por defecto) ───
#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         parameters=[{ 'use_sim_time': use_sim_time }],
#         output='screen'
#     )

#     # ─────────────── Build LaunchDescription ───────────────
#     return LaunchDescription([
#         declare_use_sim_time,
#         gzserver,
#         gzclient,
#         rsp_node,
#         jsp_node,
#         rviz_node,
#         spawn_handler,
#     ])

# #!/usr/bin/env python3
# """
# Arranca Gazebo Classic en blanco, lanza gzclient, publica el URDF del
# Seguidor_linea_robot, lo spawnea 3 s después y abre RViz2.

# Guárdalo como  modelo_robot/launch/gazebo_empty.launch.py
# (acuérdate de añadirlo al package.xml / CMakeLists si fuera necesario).
# """

# from __future__ import annotations
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
# from launch.event_handlers import OnProcessStart
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description() -> LaunchDescription:
#     pkg_share = get_package_share_directory('modelo_robot')
#     use_sim_time = LaunchConfiguration('use_sim_time')

#     # ────────── argumento --use_sim_time ──────────
#     declare_use_sim_time = DeclareLaunchArgument(
#         name='use_sim_time',
#         default_value='true',
#         description='Usar reloj simulado'
#     )

#     # ────────── 1) gzserver SIN world (carga empty.world por defecto) ──────────
#     gzserver = ExecuteProcess(
#         cmd=[
#             'gzserver',                       # ← no se pasa ningún .world
#             '--verbose',
#             '-s', 'libgazebo_ros_factory.so',
#             '-s', 'libgazebo_ros_init.so'
#         ],
#         output='screen',
#     )

#     # ────────── 2) gzclient cuando gzserver esté arriba ──────────
#     gzclient = RegisterEventHandler(
#         OnProcessStart(
#             target_action=gzserver,
#             on_start=[ExecuteProcess(cmd=['gzclient'], output='screen')]
#         )
#     )

#     # ────────── 3) leer y limpiar el URDF ──────────
#     urdf_file = os.path.join(pkg_share, 'urdf', 'Seguidor_linea_robot.urdf')
#     with open(urdf_file, 'r') as f:
#         urdf = f.read().replace('<?xml version="1.0" encoding="utf-8"?>', '')

#     # ────────── 4) robot_state_publisher ──────────
#     rsp_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{'use_sim_time': use_sim_time,
#                      'robot_description': urdf}],
#         output='screen'
#     )

#     # ────────── 5) joint_state_publisher ──────────
#     jsp_node = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         parameters=[{'use_sim_time': use_sim_time}],
#         output='screen'
#     )

#     # ────────── 6) spawn del robot (3 s tras arrancar gzserver) ──────────
#     spawn_robot = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=[
#             '-topic', '/robot_description',
#             '-entity', 'Seguidor_linea_robot',
#             '-x', '0', '-y', '0', '-z', '0.05'
#         ],
#         output='screen'
#     )
#     spawn_handler = RegisterEventHandler(
#         OnProcessStart(
#             target_action=gzserver,
#             on_start=[TimerAction(period=3.0, actions=[spawn_robot])]
#         )
#     )

#     # ────────── 7) RViz2 ──────────
#     rviz = Node(
#         package='rviz2',
#         executable='rviz2',
#         parameters=[{'use_sim_time': use_sim_time}],
#         output='screen'
#     )

#     # Lanzamos todo
#     return LaunchDescription([
#         declare_use_sim_time,
#         gzserver,
#         gzclient,
#         rsp_node,
#         jsp_node,
#         rviz,
#         spawn_handler,
#     ])
#!/usr/bin/env python3
# modelo_robot/launch/seguidor_linea_gazebo.launch.py
import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable,
                                  LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_modelo = FindPackageShare('modelo_robot').find('modelo_robot')

    default_model  = PathJoinSubstitution([pkg_modelo, 'urdf', 'Seguidor_linea_robot.urdf'])
    default_world  = PathJoinSubstitution([pkg_modelo, 'worlds', 'empty.world'])
    default_rviz   = PathJoinSubstitution([pkg_modelo, 'rviz',  'robot.rviz'])

    declare_model = DeclareLaunchArgument('model', default_value=default_model)
    declare_world = DeclareLaunchArgument('world', default_value=default_world)
    declare_sim   = DeclareLaunchArgument('use_sim_time', default_value='true')

    model        = LaunchConfiguration('model')
    world        = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Añadimos nuestros meshes al path de Gazebo
    set_gazebo_models = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[pkg_modelo, ':', LaunchConfiguration('GAZEBO_MODEL_PATH', default='')]
    )

    # ---------- robot_description ----------
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', model]),
            value_type=str)
    }

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[robot_description,
                           {'use_sim_time': use_sim_time}],
               output='both')

    jsp = Node(package='joint_state_publisher',
               executable='joint_state_publisher',
               parameters=[{'use_sim_time': use_sim_time}],
               output='screen')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments={'world': world, 'verbose': 'true'}.items())

    spawn = Node(package='gazebo_ros',
                 executable='spawn_entity.py',
                 arguments=['-topic', 'robot_description',
                            '-entity', 'Seguidor_linea_robot',
                            '-x', '0', '-y', '0', '-z', '0.05'],
                 output='screen')

    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', default_rviz],
                parameters=[{'use_sim_time': use_sim_time}],
                output='log')

    return LaunchDescription([
        declare_model,
        declare_world,
        declare_sim,
        set_gazebo_models,
        gazebo,
        rsp,
        jsp,
        spawn,
        rviz
    ])
