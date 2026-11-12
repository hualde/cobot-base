"""Lado ROS 2 de la simulación en Isaac Sim: TF del robot con cámara, controladores y MoveIt 2 + RViz.

Terminal 1 (Isaac Sim, con la escena en Play): ver README, sección Isaac Sim.
Terminal 2 (dentro del contenedor dev): ros2 launch cobot_bringup isaac_moveit.launch.py [ur_type:=ur5e]

El Controller Manager vive dentro de Isaac Sim (isaacsim.ros2.control) con config/isaac_controllers.yaml;
aquí solo se activan los controladores y se publica la descripción del robot con la cámara.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    pkg = FindPackageShare("cobot_bringup")

    args = [
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("rviz", default_value="true"),
    ]

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([pkg, "urdf", "ur_wrist_camera.urdf.xacro"]),
            " ur_type:=", ur_type,
            # El nombre del robot debe coincidir con el del SRDF de ur_moveit_config (= ur_type).
            " name:=", ur_type,
        ]),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    # --controller-manager-timeout largo: el Controller Manager aparece cuando se pulsa Play en Isaac.
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "120"],
            output="screen",
        )
        for name in ("joint_state_broadcaster", "scaled_joint_trajectory_controller")
    ]

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    return LaunchDescription(args + [rsp, *spawners, moveit])
