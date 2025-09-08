"""
Solo lanza:
  • Gazebo con obstacles.world
  • Spawnea el robot
  • Robot State Publisher + Joint State Publisher
  • Joystick PS4  (eje-1 linear, eje-3 angular)
  • Un RViz con robot.rviz
NO SLAM, NO map_server, NO AMCL
"""

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

    model = PathJoinSubstitution([pkg, "urdf", "Seguidor_linea_robot.urdf"])
    world = PathJoinSubstitution([pkg, "worlds", "obstacles.world"])
    rviz  = PathJoinSubstitution([pkg, "rviz",  "robot.rviz"])
    use_sim = LaunchConfiguration("use_sim_time", default="true")

    # Para que Gazebo encuentre tus meshes
    set_models = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[pkg, ":", LaunchConfiguration("GAZEBO_MODEL_PATH", default="")]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", model]),
            value_type=str)
    }

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[robot_description, {"use_sim_time": use_sim}], output="screen")

    jsp = Node(package="joint_state_publisher", executable="joint_state_publisher",
               parameters=[{"use_sim_time": use_sim}], output="screen")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]),
        launch_arguments={"world": world}.items())

    spawn = Node(package="gazebo_ros", executable="spawn_entity.py",
                 arguments=["-topic", "robot_description",
                            "-entity", "seguidor"], output="screen")

    # Joystick PS4  (stick izq eje 1 = lin;  stick der eje 3 = ang)
    teleop_params = {
        "require_enable_button": False,
        "axis_linear.x": 1,
        "scale_linear.x": 0.5,
        "axis_angular.yaw": 3,
        "scale_angular.yaw": 0.5,
    }
    joy  = Node(package="joy", executable="joy_node",
                parameters=[{"use_sim_time": use_sim}], output="screen")
    tele = Node(package="teleop_twist_joy", executable="teleop_node",
                parameters=[teleop_params, {"use_sim_time": use_sim}],
                output="screen")

    rviz2 = Node(package="rviz2", executable="rviz2",
                 arguments=["-d", rviz],
                 parameters=[{"use_sim_time": use_sim}],
                 output="log")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        set_models, gazebo, rsp, jsp, spawn,
        joy, tele, rviz2
    ])
