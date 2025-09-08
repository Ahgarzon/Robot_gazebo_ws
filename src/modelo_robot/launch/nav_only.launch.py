from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  Command, FindExecutable)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("modelo_robot").find("modelo_robot")

    default_model = PathJoinSubstitution([pkg, "urdf",  "Seguidor_linea_robot.urdf"])
    default_world = PathJoinSubstitution([pkg, "worlds", "obstacles.world"])
    default_rviz  = PathJoinSubstitution([pkg, "rviz",  "robot.rviz"])
    default_map   = PathJoinSubstitution([pkg, "maps",  "my_map.yaml"])

    declare_model = DeclareLaunchArgument("model", default_value=default_model)
    declare_world = DeclareLaunchArgument("world", default_value=default_world)
    declare_map   = DeclareLaunchArgument("map_yaml", default_value=default_map)
    declare_sim   = DeclareLaunchArgument("use_sim_time", default_value="true")

    model        = LaunchConfiguration("model")
    world        = LaunchConfiguration("world")
    map_yaml     = LaunchConfiguration("map_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # -------- Gazebo model path ----------
    set_models = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[pkg, ":", LaunchConfiguration("GAZEBO_MODEL_PATH", default="")]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", model]), value_type=str)
    }

    rsp = Node(package="robot_state_publisher",
               executable="robot_state_publisher",
               parameters=[robot_description, {"use_sim_time": use_sim_time}],
               output="screen")

    jsp = Node(package="joint_state_publisher",
               executable="joint_state_publisher",
               parameters=[{"use_sim_time": use_sim_time}],
               output="screen")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]),
        launch_arguments={"world": world, "verbose": "true"}.items())

    spawn = Node(package="gazebo_ros",
                 executable="spawn_entity.py",
                 arguments=["-topic", "robot_description",
                            "-entity", "Seguidor_linea_robot",
                            "-x", "0", "-y", "0", "-z", "0.05"],
                 output="screen")

    # --------- Map server + AMCL ----------
    map_server = Node(package="nav2_map_server",
                      executable="map_server",
                      name="map_server",
                      parameters=[{"yaml_filename": map_yaml,
                                   "use_sim_time": use_sim_time}],
                      output="screen")

    amcl = Node(package="nav2_amcl",
                executable="amcl",
                name="amcl",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen")

    lifecycle = Node(package="nav2_lifecycle_manager",
                     executable="lifecycle_manager",
                     name="lifecycle_manager_localization",
                     parameters=[{"use_sim_time": use_sim_time,
                                  "autostart": True,
                                  "node_names": ["map_server", "amcl"]}],
                     output="screen")

    # --------- Joystick PS4 -----------
    teleop_params = {
        "require_enable_button": False,
        "axis_linear.x": 1,          # stick izq vertical
        "scale_linear.x": 0.5,
        "scale_linear_turbo.x": 1.0,
        "axis_angular.yaw": 3,       # stick der horizontal  (eje 3 para DS4)
        "scale_angular.yaw": 0.5,
        "scale_angular_turbo.yaw": 1.0,
    }

    joy_node = Node(package="joy",
                    executable="joy_node",
                    parameters=[{"use_sim_time": use_sim_time}],
                    output="screen")

    teleop = Node(package="teleop_twist_joy",
                  executable="teleop_node",
                  parameters=[teleop_params, {"use_sim_time": use_sim_time}],
                  output="screen")

    rviz = Node(package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", default_rviz],
                parameters=[{"use_sim_time": use_sim_time}],
                output="log")

    return LaunchDescription([
        declare_model, declare_world, declare_map, declare_sim,
        set_models,
        gazebo, rsp, jsp, spawn,
        map_server, amcl, lifecycle,
        joy_node, teleop,
        rviz
    ])

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     pkg = FindPackageShare("modelo_robot").find("modelo_robot")

#     model = PathJoinSubstitution([pkg, "urdf",  "Seguidor_linea_robot.urdf"])
#     world = PathJoinSubstitution([pkg, "worlds","obstacles.world"])
#     rviz  = PathJoinSubstitution([pkg, "rviz",  "robot.rviz"])
#     map_y = PathJoinSubstitution([pkg, "maps",  "my_map.yaml"])
#     use_sim = LaunchConfiguration("use_sim_time", default="true")

#     set_models = SetEnvironmentVariable(
#         name="GAZEBO_MODEL_PATH",
#         value=[pkg,":",LaunchConfiguration("GAZEGO_MODEL_PATH",default="")]
#     )

#     robot_description = {
#         "robot_description": ParameterValue(
#             Command([FindExecutable(name="xacro"), " ", model]), value_type=str)
#     }

#     rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
#                parameters=[robot_description, {"use_sim_time": use_sim}], output="screen")
#     jsp = Node(package="joint_state_publisher", executable="joint_state_publisher",
#                parameters=[{"use_sim_time": use_sim}], output="screen")

#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]),
#         launch_arguments={"world": world}.items())

#     spawn = Node(package="gazebo_ros", executable="spawn_entity.py",
#                  arguments=["-topic", "robot_description", "-entity", "seguidor"],
#                  output="screen")

#     map_server = Node(package="nav2_map_server", executable="map_server",
#                       parameters=[{"yaml_filename": map_y, "use_sim_time": use_sim}], output="screen")
#     amcl = Node(package="nav2_amcl", executable="amcl",
#                 parameters=[{"use_sim_time": use_sim}], output="screen")
#     lifecycle = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
#                      name="lifecycle_manager_loc",
#                      parameters=[{"use_sim_time": use_sim,
#                                   "autostart": True,
#                                   "node_names": ["map_server","amcl"]}],
#                      output="screen")

#     teleop_parms = {
#         "require_enable_button": False,
#         "axis_linear.x": 3,
#         "scale_linear.x": 0.5,
#         "axis_angular.yaw": 1,
#         "scale_angular.yaw": 0.5,
#     }
#     joy  = Node(package="joy", executable="joy_node",
#                 parameters=[{"use_sim_time": use_sim}], output="screen")
#     tele = Node(package="teleop_twist_joy", executable="teleop_node",
#                 parameters=[teleop_parms, {"use_sim_time": use_sim}], output="screen")

#     rviz2 = Node(package="rviz2", executable="rviz2",
#                  arguments=["-d", rviz], parameters=[{"use_sim_time": use_sim}], output="log")

#     return LaunchDescription([
#         DeclareLaunchArgument("use_sim_time", default_value="true"),
#         set_models, gazebo, rsp, jsp, spawn,
#         map_server, amcl, lifecycle,
#         joy, tele, rviz2
#     ])
#!/usr/bin/env python3
# """
# Localización (AMCL) + pila Nav2 sobre un mapa .yaml ya guardado
# No lanza Gazebo. Asume que el mundo/robot siguen corriendo (o es hardware real)
# """

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     pkg   = FindPackageShare("modelo_robot").find("modelo_robot")
#     rviz  = PathJoinSubstitution([pkg, "rviz",  "rviz_nav.rviz"])
#     mapa  = PathJoinSubstitution([pkg, "maps",  "my_map.yaml"])
#     nav2  = PathJoinSubstitution([pkg, "config","nav2_params.yaml"])

#     use_sim = LaunchConfiguration("use_sim_time", default="true")

#     #  map + AMCL
#     map_server = Node(package="nav2_map_server", executable="map_server",
#                       name="map_server",
#                       parameters=[{"yaml_filename": mapa, "use_sim_time": use_sim}],
#                       output="screen")

#     amcl = Node(package="nav2_amcl", executable="amcl",
#                 parameters=[{"use_sim_time": use_sim}], output="screen")

#     loc_manager = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
#                        name="lifecycle_manager_localization",
#                        parameters=[{
#                            "use_sim_time": use_sim, "autostart": True,
#                            "node_names": ["map_server", "amcl"]}], output="screen")

#     # Nav2 (idéntico a antes, sin Gazebo)
#     planner   = Node(package="nav2_planner",    executable="planner_server",
#                      parameters=[nav2, {"use_sim_time": use_sim}], output="screen")
#     controller= Node(package="nav2_controller", executable="controller_server",
#                      parameters=[nav2, {"use_sim_time": use_sim}], output="screen")
#     smoother  = Node(package="nav2_smoother",   executable="smoother_server",
#                      parameters=[nav2, {"use_sim_time": use_sim}], output="screen")
#     behaviors = Node(package="nav2_behavior_tree", executable="behavior_server",
#                      parameters=[nav2, {"use_sim_time": use_sim}], output="screen")
#     bt_nav    = Node(package="nav2_bt_navigator", executable="bt_navigator",
#                      parameters=[nav2, {"use_sim_time": use_sim}], output="screen")
#     waypoint  = Node(package="nav2_waypoint_follower", executable="waypoint_follower",
#                      parameters=[nav2, {"use_sim_time": use_sim}], output="screen")
#     vel_smooth= Node(package="nav2_velocity_smoother", executable="velocity_smoother",
#                      parameters=[nav2, {"use_sim_time": use_sim}], output="screen")

#     nav_manager = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
#                        name="lifecycle_manager_navigation",
#                        parameters=[{
#                            "use_sim_time": use_sim, "autostart": True,
#                            "node_names": [
#                                "controller_server","planner_server","smoother_server",
#                                "behavior_server","bt_navigator",
#                                "waypoint_follower","velocity_smoother"]}],
#                        output="screen")

#     rviz = Node(package="rviz2", executable="rviz2",
#                 arguments=["-d", rviz],
#                 parameters=[{"use_sim_time": use_sim}], output="log")

#     return LaunchDescription([
#         DeclareLaunchArgument("use_sim_time", default_value="true"),
#         map_server, amcl, loc_manager,
#         planner, controller, smoother, behaviors,
#         bt_nav, waypoint, vel_smooth, nav_manager,
#         rviz
#     ])

# #!/usr/bin/env python3
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.substitutions import FindPackageShare
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue

# def generate_launch_description():
#     pkg = FindPackageShare('modelo_robot').find('modelo_robot')

#     # 1️⃣ arguments
#     declare_model   = DeclareLaunchArgument('model',
#                         default_value=PathJoinSubstitution([pkg, 'urdf', 'Seguidor_linea_robot.urdf']))
#     declare_map     = DeclareLaunchArgument('map_yaml',
#                         default_value=PathJoinSubstitution([pkg, 'maps', 'my_map.yaml']))
#     declare_rviz    = DeclareLaunchArgument('rviz',
#                         default_value=PathJoinSubstitution([pkg, 'rviz', 'robot.rviz']))
#     declare_simtime = DeclareLaunchArgument('use_sim_time', default_value='true')

#     model_file    = LaunchConfiguration('model')
#     map_yaml_file = LaunchConfiguration('map_yaml')
#     rviz_file     = LaunchConfiguration('rviz')
#     use_sim_time  = LaunchConfiguration('use_sim_time')

#     # 2️⃣ let Gazebo find your meshes
#     set_gz_model_path = SetEnvironmentVariable(
#         name='GAZEBO_MODEL_PATH',
#         value=[pkg, ':', LaunchConfiguration('GAZEBO_MODEL_PATH', default='')]
#     )

#     # 3️⃣ robot_description
#     robot_description = {
#         'robot_description': ParameterValue(
#             Command([FindExecutable(name='xacro'), ' ', model_file]),
#             value_type=str
#         )
#     }
#     rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
#                parameters=[robot_description, {'use_sim_time': use_sim_time}],
#                output='screen')
#     jsp = Node(package='joint_state_publisher', executable='joint_state_publisher',
#                parameters=[{'use_sim_time': use_sim_time}],
#                output='screen')

#     # 4️⃣ Gazebo
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([FindPackageShare('gazebo_ros'),
#                                   'launch', 'gazebo.launch.py'])
#         ),
#         launch_arguments={'verbose': 'true'}.items()
#     )
#     spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
#                  arguments=['-topic', 'robot_description',
#                             '-entity', 'Seguidor_linea_robot',
#                             '-x', '0', '-y', '0', '-z', '0.05'],
#                  output='screen')

#     # 5️⃣ Nav2: map_server + AMCL + lifecycle
#     map_srv = Node(package='nav2_map_server', executable='map_server',
#                    name='map_server',
#                    parameters=[{'yaml_filename': map_yaml_file,
#                                 'use_sim_time': use_sim_time}],
#                    output='screen')
#     amcl = Node(package='nav2_amcl', executable='amcl',
#                 name='amcl',
#                 parameters=[{'use_sim_time': use_sim_time}],
#                 output='screen')
#     lifecycle = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
#                      name='lifecycle_manager_localization',
#                      parameters=[{
#                        'use_sim_time': use_sim_time,
#                        'autostart':    True,
#                        'node_names':   ['map_server','amcl']
#                      }],
#                      output='screen')

#     # 6️⃣ Teleop
#     teleop_params = {
#         'require_enable_button': False,
#         'axis_linear.x':         1,
#         'scale_linear.x':        0.5,
#         'axis_angular.yaw':      3,
#         'scale_angular.yaw':     0.5,
#     }
#     joy_node = Node(package='joy', executable='joy_node',
#                     parameters=[{'use_sim_time': use_sim_time}],
#                     output='screen')
#     teleop   = Node(package='teleop_twist_joy', executable='teleop_node',
#                     parameters=[teleop_params, {'use_sim_time': use_sim_time}],
#                     output='screen')

#     # 7️⃣ RViz for Nav2
#     rviz2 = Node(package='rviz2', executable='rviz2', name='rviz_nav',
#                  arguments=['-d', rviz_file],
#                  parameters=[{'use_sim_time': use_sim_time}],
#                  output='screen')

#     return LaunchDescription([
#         declare_model, declare_map, declare_rviz, declare_simtime,
#         set_gz_model_path,
#         gazebo, rsp, jsp, spawn,
#         map_srv, amcl, lifecycle,
#         joy_node, teleop,
#         rviz2,
#     ])
