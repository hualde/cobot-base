"""Driver ROS 2 de UR + MoveIt 2 (con RViz opcional). Es el launch de producción y también el de URSim.

Contra URSim:  ./scripts/ursim_up.sh   (arranca URSim, enciende y suelta frenos)
               ros2 launch cobot_bringup ur_moveit.launch.py
Robot real:    ros2 launch cobot_bringup ur_moveit.launch.py robot_ip:=<IP del robot> \
                   kinematics_params_file:=<calibración del robot> rviz:=false

La calibración se extrae del propio robot con ur_calibration (ver README, "Calibración del robot").
Sin ella se usan los valores nominales del modelo, válidos para URSim pero menos precisos en un robot real.

headless_mode:=false si se usa el programa External Control desde el Teach Pendant.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _driver(context):
    # "nominal" = calibración genérica del modelo (la que trae ur_description).
    kinematics = LaunchConfiguration("kinematics_params_file").perform(context)
    if kinematics in ("", "nominal"):
        kinematics = PathJoinSubstitution(
            [FindPackageShare("ur_description"), "config", LaunchConfiguration("ur_type"), "default_kinematics.yaml"]
        )
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "headless_mode": LaunchConfiguration("headless_mode"),
            "kinematics_params_file": kinematics,
            "launch_rviz": "false",
        }.items(),
    )]


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")

    args = [
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument(
            "robot_ip", default_value="192.168.56.101",
            description="IP del controlador UR (por defecto, la de URSim)",
        ),
        DeclareLaunchArgument(
            "kinematics_params_file", default_value="nominal",
            description="YAML de calibración generado con ur_calibration, o 'nominal' para los valores genéricos",
        ),
        DeclareLaunchArgument(
            "headless_mode", default_value="true",
            description="El driver envía el programa URScript sin tocar Polyscope",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
    ]

    # MoveIt lee robot_description del tópico que publica el driver, así que hereda la calibración.
    # No va dentro de un TimerAction: ur_moveit_config ya espera a robot_description por su cuenta, y dentro
    # de un temporizador sus argumentos (warehouse_sqlite_path...) dejan de existir antes de arrancar move_group.
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])
        ),
        launch_arguments={"ur_type": ur_type, "launch_rviz": LaunchConfiguration("rviz")}.items(),
    )

    return LaunchDescription(args + [OpaqueFunction(function=_driver), moveit])
