# #!/usr/bin/env python3
# """
# Un solo comando para:
#   • Gazebo + obstacles.world (con el robot)
#   • map_server + AMCL + pila completa de Nav2
#   • Joystick PS4 (stick izq = lin, der = ang)
#   • RViz con panel Navigation2 listo para “Nav2 Goal”
# """

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import (
#     LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
# )
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     pkg = FindPackageShare("modelo_robot").find("modelo_robot")

#     # ───────── rutas por defecto ─────────
#     def_model = PathJoinSubstitution([pkg, "urdf",  "Seguidor_linea_robot.urdf"])
#     def_world = PathJoinSubstitution([pkg, "worlds", "obstacles.world"])
#     def_rviz  = PathJoinSubstitution([pkg, "rviz",  "robot.rviz"])
#     def_map   = PathJoinSubstitution([pkg, "maps",  "my_map.yaml"])
#     def_nav2  = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])

#     # ───────────── argumentos CLI ─────────────
#     declare_model = DeclareLaunchArgument("model",       default_value=def_model)
#     declare_world = DeclareLaunchArgument("world",       default_value=def_world)
#     declare_map   = DeclareLaunchArgument("map_yaml",    default_value=def_map)
#     declare_nav2  = DeclareLaunchArgument("nav2_params", default_value=def_nav2)
#     declare_sim   = DeclareLaunchArgument("use_sim_time", default_value="true")

#     model        = LaunchConfiguration("model")
#     world        = LaunchConfiguration("world")
#     map_yaml     = LaunchConfiguration("map_yaml")
#     nav2_yaml    = LaunchConfiguration("nav2_params")
#     use_sim_time = LaunchConfiguration("use_sim_time")

#     # ───── Gazebo encuentra los meshes ─────
#     set_models = SetEnvironmentVariable(
#         "GAZEBO_MODEL_PATH", [pkg, ":", LaunchConfiguration("GAZEBO_MODEL_PATH", default="")]
#     )

#     # ───── robot_description
#     robot_description = {
#         "robot_description": ParameterValue(
#             Command([FindExecutable(name="xacro"), " ", model]), value_type=str)
#     }

#     rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
#                parameters=[robot_description, {"use_sim_time": use_sim_time}],
#                output="screen")

#     jsp = Node(package="joint_state_publisher", executable="joint_state_publisher",
#                parameters=[{"use_sim_time": use_sim_time}], output="screen")

#     # ───── Gazebo + Spawn
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
#         ),
#         launch_arguments={"world": world, "verbose": "true"}.items())

#     spawn = Node(package="gazebo_ros", executable="spawn_entity.py",
#                  arguments=["-topic", "robot_description",
#                             "-entity", "Seguidor_linea_robot",
#                             "-x", "0", "-y", "0", "-z", "0.05"],
#                  output="screen")

#     # ───── Map-server + AMCL
#     map_server = Node(package="nav2_map_server", executable="map_server",
#                       name="map_server",
#                       parameters=[{"yaml_filename": map_yaml,
#                                    "use_sim_time": use_sim_time}],
#                       output="screen")

#     amcl = Node(package="nav2_amcl", executable="amcl",
#                 name="amcl",
#                 parameters=[{"use_sim_time": use_sim_time}],
#                 output="screen")

#     loc_manager = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
#                        name="lifecycle_manager_localization",
#                        parameters=[{
#                            "use_sim_time": use_sim_time,
#                            "autostart": True,
#                            "node_names": ["map_server", "amcl"]}],
#                        output="screen")

#     # ───── Pila Nav2
#     planner      = Node(package="nav2_planner",    executable="planner_server",
#                         parameters=[nav2_yaml, {"use_sim_time": use_sim_time}], output="screen")
#     controller   = Node(package="nav2_controller", executable="controller_server",
#                         parameters=[nav2_yaml, {"use_sim_time": use_sim_time}], output="screen")
#     smoother     = Node(package="nav2_smoother",   executable="smoother_server",
#                         parameters=[nav2_yaml, {"use_sim_time": use_sim_time}], output="screen")
#     behavior_srv = Node(package="nav2_behaviors",  executable="behavior_server",
#                         parameters=[nav2_yaml, {"use_sim_time": use_sim_time}], output="screen")
#     bt_nav       = Node(package="nav2_bt_navigator", executable="bt_navigator",
#                         parameters=[nav2_yaml, {"use_sim_time": use_sim_time}], output="screen")
#     waypoint     = Node(package="nav2_waypoint_follower", executable="waypoint_follower",
#                         parameters=[nav2_yaml, {"use_sim_time": use_sim_time}], output="screen")
#     vel_smoother = Node(package="nav2_velocity_smoother", executable="velocity_smoother",
#                         parameters=[nav2_yaml, {"use_sim_time": use_sim_time}], output="screen")

#     nav_manager = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
#                        name="lifecycle_manager_navigation",
#                        parameters=[{
#                            "use_sim_time": use_sim_time,
#                            "autostart": True,
#                            "node_names": [
#                                "planner_server", "controller_server",
#                                "smoother_server", "behavior_server",
#                                "bt_navigator", "waypoint_follower",
#                                "velocity_smoother"]}],
#                        output="screen")

#     # ───── Joystick PS4
#     teleop_params = {
#         "require_enable_button": False,
#         "axis_linear.x": 1,  "scale_linear.x": 0.5,
#         "axis_angular.yaw": 3, "scale_angular.yaw": 0.5,
#     }
#     joy  = Node(package="joy", executable="joy_node",
#                 parameters=[{"use_sim_time": use_sim_time}], output="screen")
#     tele = Node(package="teleop_twist_joy", executable="teleop_node",
#                 parameters=[teleop_params, {"use_sim_time": use_sim_time}],
#                 output="screen")

#     # ───── RViz
#     rviz = Node(package="rviz2", executable="rviz2",
#                 arguments=["-d", def_rviz],
#                 parameters=[{"use_sim_time": use_sim_time}], output="log")

#     return LaunchDescription([
#         declare_model, declare_world, declare_map, declare_nav2, declare_sim,
#         set_models,
#         gazebo, rsp, jsp, spawn,
#         map_server, amcl, loc_manager,
#         planner, controller, smoother, behavior_srv,
#         bt_nav, waypoint, vel_smoother, nav_manager,
#         joy, tele, rviz
#     ])
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare("modelo_robot").find("modelo_robot")

    # --- default file paths ---
    urdf_xacro = PathJoinSubstitution([pkg, "urdf", "Seguidor_linea_robot.urdf"])
    world_file = PathJoinSubstitution([pkg, "worlds", "obstacles.world"])
    rviz_config = PathJoinSubstitution([pkg, "rviz", "robot.rviz"])
    map_yaml   = PathJoinSubstitution([pkg, "maps", "my_map.yaml"])

    use_sim = LaunchConfiguration("use_sim_time", default="true")

    # --- ensure Gazebo can find your meshes ---
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[pkg, ":", LaunchConfiguration("GAZEBO_MODEL_PATH", default="")]
    )

    # --- robot_description from xacro ---
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ",
                urdf_xacro
            ]),
            value_type=str
        )
    }

    # robot_state_publisher + joint_state_publisher
    rsp = Node(package="robot_state_publisher",
               executable="robot_state_publisher",
               parameters=[robot_description, {"use_sim_time": use_sim}],
               output="screen")
    jsp = Node(package="joint_state_publisher",
               executable="joint_state_publisher",
               parameters=[{"use_sim_time": use_sim}],
               output="screen")

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"
        ]),
        launch_arguments={
            "world": world_file,
            "verbose": "true"
        }.items()
    )

    spawn_entity = Node(package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=[
                          "-topic", "robot_description",
                          "-entity", "Seguidor_linea_robot",
                          "-x", "0", "-y", "0", "-z", "0.05"
                        ],
                        output="screen")

    # --- SLAM Toolbox ---
    slam = Node(package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[map_yaml, {"use_sim_time": use_sim}],
                output="screen")

    # --- Nav2: map_server, AMCL, lifecycle_manager ---
    map_server = Node(package="nav2_map_server",
                      executable="map_server",
                      name="map_server",
                      parameters=[{"yaml_filename": map_yaml,
                                   "use_sim_time": use_sim}],
                      output="screen")
    amcl = Node(package="nav2_amcl",
                executable="amcl",
                name="amcl",
                parameters=[{"use_sim_time": use_sim}],
                output="screen")
    lifecycle = Node(package="nav2_lifecycle_manager",
                     executable="lifecycle_manager",
                     name="lifecycle_manager_localization",
                     parameters=[{
                         "use_sim_time": use_sim,
                         "autostart": True,
                         "node_names": ["map_server", "amcl"]
                     }],
                     output="screen")

    # --- joystick teleop (PS4) ---
    teleop_params = {
      "require_enable_button": False,
      "axis_linear.x":       1,
      "scale_linear.x":      0.5,
      "axis_angular.yaw":    3,
      "scale_angular.yaw":   0.5,
    }
    joy    = Node(package="joy", executable="joy_node",
                  parameters=[{"use_sim_time": use_sim}],
                  output="screen")
    teleop = Node(package="teleop_twist_joy", executable="teleop_node",
                  parameters=[teleop_params, {"use_sim_time": use_sim}],
                  output="screen")

    # --- RViz2 ---
    rviz2 = Node(package="rviz2", executable="rviz2",
                 name="rviz2",
                 arguments=["-d", rviz_config],
                 parameters=[{"use_sim_time": use_sim}],
                 output="screen")

    return LaunchDescription([
      # declare args
      DeclareLaunchArgument("use_sim_time", default_value="true"),
      set_gazebo_model_path,
      # start up
      gazebo, rsp, jsp, spawn_entity,
      slam,
      map_server, amcl, lifecycle,
      joy, teleop,
      rviz2,
    ])
