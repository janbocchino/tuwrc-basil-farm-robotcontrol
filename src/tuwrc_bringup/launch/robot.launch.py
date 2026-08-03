"""Unified bringup for view (mock) and hardware (real arm + rail hold) modes."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    mode = LaunchConfiguration("mode").perform(context).lower()
    if mode not in {"view", "hardware"}:
        raise RuntimeError(f"mode must be view or hardware; got: {mode}")

    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() == "true"
    use_moveit = LaunchConfiguration("use_moveit").perform(context).lower() == "true"
    use_gui = LaunchConfiguration("use_gui").perform(context).lower() == "true"
    port = LaunchConfiguration("port").perform(context)
    gui_port = LaunchConfiguration("gui_port").perform(context)

    desc_share = Path(get_package_share_directory("lerobot_description"))
    driver_share = Path(get_package_share_directory("six_motor_driver"))
    moveit_share = Path(get_package_share_directory("six_motor_moveit_config"))

    urdf_file = desc_share / "urdf" / "so101_base.xacro"
    calibration = driver_share / "config" / "six_motor_calibration.yaml"
    controller_file = (
        "config/controllers.yaml" if mode == "view" else "config/controllers_real.yaml"
    )
    rviz_config = (
        moveit_share / "config" / "moveit.rviz"
        if use_moveit
        else desc_share / "rviz" / "display.rviz"
    )

    robot_description = ParameterValue(
        Command(["xacro ", str(urdf_file)]), value_type=str
    )

    actions = [
        LogInfo(msg=f"Starting TUWRC robot in mode={mode}"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
    ]

    if mode == "view":
        actions.append(
            Node(
                package="tuwrc_mock_hardware",
                executable="mock_hardware",
                output="screen",
            )
        )
    else:
        actions.extend(
            [
                Node(
                    package="six_motor_driver",
                    executable="six_motor_driver",
                    parameters=[
                        str(calibration),
                        {
                            "port": port,
                            "center_on_start": False,
                            "bridge_sensor_wrap": True,
                        },
                    ],
                    output="screen",
                ),
                Node(
                    package="tuwrc_mock_hardware",
                    executable="rail_hold",
                    output="screen",
                ),
            ]
        )

    if use_moveit:
        moveit_config = (
            MoveItConfigsBuilder(
                "so101_rail",
                package_name="six_motor_moveit_config",
            )
            .robot_description(file_path=str(urdf_file))
            .robot_description_semantic(file_path="config/so101.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .trajectory_execution(
                file_path=controller_file,
                moveit_manage_controllers=False,
            )
            .planning_pipelines(
                pipelines=["ompl"],
                default_planning_pipeline="ompl",
            )
            .to_moveit_configs()
        )
        actions.append(
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                parameters=[moveit_config.to_dict()],
                output="screen",
            )
        )
        if use_rviz:
            actions.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    arguments=["-d", str(rviz_config)],
                    parameters=[moveit_config.to_dict()],
                    output="screen",
                )
            )
    elif use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(rviz_config)],
                output="screen",
            )
        )

    if use_gui:
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "lerobot_gui",
                    "joint_state_gui",
                    "--ros-args",
                    "--",
                    "--port",
                    gui_port,
                    "--mode",
                    mode,
                ],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="view",
                description="view (mock) or hardware (real arm + fixed rail)",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_moveit", default_value="true"),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("gui_port", default_value="3000"),
            OpaqueFunction(function=launch_setup),
        ]
    )
