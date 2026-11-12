"""Driver ROS 2 de UR contra URSim (o un robot real) + MoveIt 2 con RViz.

Primero: ./scripts/ursim_up.sh   (arranca URSim, enciende y suelta frenos)
Luego, dentro del contenedor dev: ros2 launch cobot_bringup ursim_moveit.launch.py [robot_ip:=192.168.56.101] [ur_type:=ur5e]

Para el robot físico basta con cambiar robot_ip (y headless_mode:=false si se usa el programa
External Control desde el Teach Pendant).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")

    args = [
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.56.101"),
        DeclareLaunchArgument(
            "headless_mode", default_value="true",
            description="El driver envía el programa URScript sin tocar Polyscope",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
    ]

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": LaunchConfiguration("robot_ip"),
            "headless_mode": LaunchConfiguration("headless_mode"),
            "launch_rviz": "false",
        }.items(),
    )

    moveit = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])
            ),
            launch_arguments={"ur_type": ur_type, "launch_rviz": LaunchConfiguration("rviz")}.items(),
        )],
    )

    return LaunchDescription(args + [driver, moveit])
