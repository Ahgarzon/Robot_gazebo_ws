# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     pkg_modelo = FindPackageShare('modelo_robot').find('modelo_robot')

#     default_model  = PathJoinSubstitution([pkg_modelo, 'urdf', 'Seguidor_linea_robot.urdf'])
#     default_world  = PathJoinSubstitution([pkg_modelo, 'worlds', 'obstacles.world'])
#     default_rviz   = PathJoinSubstitution([pkg_modelo, 'rviz',  'robot.rviz'])
#     slam_params    = PathJoinSubstitution([pkg_modelo, 'config', 'mapper_params_online_async.yaml'])

#     declare_model = DeclareLaunchArgument('model', default_value=default_model)
#     declare_world = DeclareLaunchArgument('world', default_value=default_world)
#     declare_sim   = DeclareLaunchArgument('use_sim_time', default_value='true')

#     model        = LaunchConfiguration('model')
#     world        = LaunchConfiguration('world')
#     use_sim_time = LaunchConfiguration('use_sim_time')

#     # GAZEBO MODEL PATH
#     set_gazebo_models = SetEnvironmentVariable(
#         name='GAZEBO_MODEL_PATH',
#         value=[pkg_modelo, ':', LaunchConfiguration('GAZEBO_MODEL_PATH', default='')]
#     )

#     # ROBOT DESCRIPTION
#     robot_description = {
#         'robot_description': ParameterValue(
#             Command([FindExecutable(name='xacro'), ' ', model]),
#             value_type=str)
#     }

#     # NODES
#     rsp = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[robot_description, {'use_sim_time': use_sim_time}],
#         output='both'
#     )

#     jsp = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         parameters=[{'use_sim_time': use_sim_time}],
#         output='screen'
#     )

#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py']),
#         launch_arguments={'world': world, 'verbose': 'true'}.items()
#     )

#     spawn = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-topic', 'robot_description', '-entity', 'Seguidor_linea_robot',
#                    '-x', '0', '-y', '0', '-z', '0.05'],
#         output='screen'
#     )

#     slam = Node(
#         package='slam_toolbox',
#         executable='async_slam_toolbox_node',
#         name='slam_toolbox',
#         parameters=[slam_params, {'use_sim_time': use_sim_time}],
#         output='screen'
#     )

#     map_server = Node(
#         package='nav2_map_server',
#         executable='map_server',
#         name='map_server',
#         parameters=[{'use_sim_time': use_sim_time}],
#         output='screen'
#     )

#     amcl = Node(
#         package='nav2_amcl',
#         executable='amcl',
#         name='amcl',
#         parameters=[{'use_sim_time': use_sim_time}],
#         output='screen'
#     )

#     lifecycle = Node(
#         package='nav2_lifecycle_manager',
#         executable='lifecycle_manager',
#         name='lifecycle_manager_localization',
#         parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
#                      'node_names': ['map_server', 'amcl']}],
#         output='screen'
#     )

#     rviz = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         arguments=['-d', default_rviz],
#         parameters=[{'use_sim_time': use_sim_time}],
#         output='log'
#     )

#     return LaunchDescription([
#         declare_model,
#         declare_world,
#         declare_sim,
#         set_gazebo_models,
#         gazebo,
#         rsp,
#         jsp,
#         spawn,
#         slam,
#         #map_server,
#         #amcl,
#         #lifecycle,
#         rviz
#     ])

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
#                                   Command, FindExecutable)
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     pkg = FindPackageShare("modelo_robot").find("modelo_robot")

#     model  = PathJoinSubstitution([pkg, "urdf",  "Seguidor_linea_robot.urdf"])
#     world  = PathJoinSubstitution([pkg, "worlds","obstacles.world"])
#     rviz   = PathJoinSubstitution([pkg, "rviz",  "robot.rviz"])
#     slam_y = PathJoinSubstitution([pkg,"config","mapper_params_online_async.yaml"])

#     use_sim = LaunchConfiguration("use_sim_time", default="true")

#     # ── Gazebo model path ──
#     set_models = SetEnvironmentVariable(
#         name="GAZEBO_MODEL_PATH",
#         value=[pkg, ":", LaunchConfiguration("GAZEBO_MODEL_PATH", default="")]
#     )

#     # ── robot_description ──
#     robot_description = {
#         "robot_description": ParameterValue(
#             Command([FindExecutable(name="xacro"), " ", model]),
#             value_type=str)
#     }

#     rsp = Node(package="robot_state_publisher",
#                executable="robot_state_publisher",
#                parameters=[robot_description, {"use_sim_time": use_sim}],
#                output="screen")

#     jsp = Node(package="joint_state_publisher",
#                executable="joint_state_publisher",
#                parameters=[{"use_sim_time": use_sim}],
#                output="screen")

#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]),
#         launch_arguments={"world": world, "verbose": "true"}.items())

#     spawn = Node(package="gazebo_ros",
#                  executable="spawn_entity.py",
#                  arguments=["-topic", "robot_description",
#                             "-entity", "Seguidor_linea_robot",
#                             "-x", "0", "-y", "0", "-z", "0.05"],
#                  output="screen")

#     # ── SLAM Toolbox ──
#     slam = Node(package="slam_toolbox",
#                 executable="async_slam_toolbox_node",
#                 name="slam_toolbox",
#                 parameters=[slam_y, {"use_sim_time": use_sim}],
#                 output="screen")

#     # ── Joystick PS4 ──
#     teleop_params = {
#         "require_enable_button": False,
#         "axis_linear.x": 3,          # stick izq vertical
#         "scale_linear.x": 0.5,
#         "axis_angular.yaw": 1,       # stick der horizontal
#         "scale_angular.yaw": 0.5,
#     }
#     joy  = Node(package="joy", executable="joy_node",
#                 parameters=[{"use_sim_time": use_sim}], output="screen")
#     tele = Node(package="teleop_twist_joy", executable="teleop_node",
#                 parameters=[teleop_params, {"use_sim_time": use_sim}],
#                 output="screen")

#     rviz2 = Node(package="rviz2", executable="rviz2",
#                  arguments=["-d", rviz],
#                  parameters=[{"use_sim_time": use_sim}],
#                  output="log")

#     return LaunchDescription([
#         DeclareLaunchArgument("use_sim_time", default_value="true"),
#         set_models, gazebo, rsp, jsp, spawn,
#         slam, joy, tele, rviz2
#     ])

# #!/usr/bin/env python3
# """
# Gazebo + robot + SLAM Toolbox (async) + joystick + RViz
# Usa mapper_params_online_async.yaml  ⭢  publica /map
# """

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     pkg   = FindPackageShare("modelo_robot").find("modelo_robot")
#     model = PathJoinSubstitution([pkg, "urdf",  "Seguidor_linea_robot.urdf"])
#     world = PathJoinSubstitution([pkg, "worlds", "obstacles.world"])
#     rviz  = PathJoinSubstitution([pkg, "rviz",  "rviz_slam.rviz"])
#     slam  = PathJoinSubstitution([pkg, "config","mapper_params_online_async.yaml"])

#     use_sim = LaunchConfiguration("use_sim_time", default="true")

#     set_models = SetEnvironmentVariable(
#         name="GAZEBO_MODEL_PATH",
#         value=[pkg, ":", LaunchConfiguration("GAZEBO_MODEL_PATH", default="")])

#     robot_description = {"robot_description": ParameterValue(
#         Command([FindExecutable(name="xacro"), " ", model]), value_type=str)}

#     rsp = Node(package="robot_state_publisher",
#                executable="robot_state_publisher",
#                parameters=[robot_description, {"use_sim_time": use_sim}], output="screen")

#     jsp = Node(package="joint_state_publisher",
#                executable="joint_state_publisher",
#                parameters=[{"use_sim_time": use_sim}], output="screen")

#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"),
#                                        "/launch/gazebo.launch.py"]),
#         launch_arguments={"world": world}.items())

#     spawn = Node(package="gazebo_ros", executable="spawn_entity.py",
#                  arguments=["-topic", "robot_description", "-entity", "seguidor"],
#                  output="screen")

#     slam_toolbox = Node(package="slam_toolbox", executable="async_slam_toolbox_node",
#                         name="slam_toolbox",
#                         parameters=[slam, {"use_sim_time": use_sim}],
#                         output="screen")

#     joy = Node(package="joy", executable="joy_node",
#                parameters=[{"use_sim_time": use_sim}], output="screen")

#     tele = Node(package="teleop_twist_joy", executable="teleop_node",
#                 parameters=[{
#                     "require_enable_button": False,
#                     "axis_linear.x": 1, "scale_linear.x": 0.5,
#                     "axis_angular.yaw": 3, "scale_angular.yaw": 0.5,
#                     "use_sim_time": use_sim}], output="screen")

#     rviz = Node(package="rviz2", executable="rviz2",
#                 arguments=["-d", rviz],
#                 parameters=[{"use_sim_time": use_sim}], output="log")

#     return LaunchDescription([
#         DeclareLaunchArgument("use_sim_time", default_value="true"),
#         set_models, gazebo, rsp, jsp, spawn,
#         slam_toolbox, joy, tele, rviz
#     ])
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = FindPackageShare('modelo_robot').find('modelo_robot')

    # ————— arguments —————
    declare_world   = DeclareLaunchArgument('world',
                        default_value=PathJoinSubstitution([pkg, 'worlds', 'obstacles.world']))
    declare_model   = DeclareLaunchArgument('model',
                        default_value=PathJoinSubstitution([pkg, 'urdf', 'Seguidor_linea_robot.urdf']))
    declare_rviz    = DeclareLaunchArgument('rviz', 
                        default_value=PathJoinSubstitution([pkg, 'rviz', 'robot.rviz']))
    declare_slam    = DeclareLaunchArgument('slam_params',
                        default_value=PathJoinSubstitution([pkg, 'config', 'mapper_params_online_async.yaml']))
    declare_simtime = DeclareLaunchArgument('use_sim_time', default_value='true')

    world_file      = LaunchConfiguration('world')
    model_file      = LaunchConfiguration('model')
    rviz_file       = LaunchConfiguration('rviz')
    slam_params     = LaunchConfiguration('slam_params')
    use_sim_time    = LaunchConfiguration('use_sim_time')

    # — Gazebo MODEL_PATH so meshes load —
    set_gz_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[pkg, ':', LaunchConfiguration('GAZEBO_MODEL_PATH', default='')]
    )

    # — robot_description via xacro —
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', model_file]), value_type=str
        )
    }
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[robot_description, {'use_sim_time': use_sim_time}],
               output='screen')
    jsp = Node(package='joint_state_publisher', executable='joint_state_publisher',
               parameters=[{'use_sim_time': use_sim_time}],
               output='screen')

    # — Gazebo server+client —
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'),
                                  'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-topic', 'robot_description',
                            '-entity', 'Seguidor_linea_robot',
                            '-x', '0', '-y', '0', '-z', '0.05'],
                 output='screen')

    # — SLAM Toolbox —
    slam = Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[slam_params, {'use_sim_time': use_sim_time}],
                output='screen')

    # — RViz for SLAM —
    rviz = Node(package='rviz2', executable='rviz2', name='rviz_slam',
                arguments=['-d', rviz_file],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen')

    return LaunchDescription([
        declare_world, declare_model, declare_rviz, declare_slam, declare_simtime,
        set_gz_model_path, gazebo, rsp, jsp, spawn, slam, rviz
    ])
