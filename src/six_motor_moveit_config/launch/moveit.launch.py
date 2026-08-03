from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


VALID_MODES = {"real", "sim", "mirror"}


def launch_setup(context):
    mode = LaunchConfiguration("controller_mode").perform(context)
    if mode not in VALID_MODES:
        raise RuntimeError(
            f"controller_mode muss real, sim oder mirror sein; erhalten: {mode}"
        )

    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    )
    start_hardware = LaunchConfiguration("start_hardware")

    lerobot_share = Path(get_package_share_directory("lerobot_description"))
    driver_share = Path(get_package_share_directory("six_motor_driver"))
    moveit_share = Path(
        get_package_share_directory("six_motor_moveit_config")
    )

    urdf_file = lerobot_share / "urdf" / "so101_base.xacro"
    calibration = driver_share / "config" / "six_motor_calibration.yaml"
    controller_file = f"config/controllers_{mode}.yaml"

    moveit_config = (
        MoveItConfigsBuilder(
            "so101",
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

    common_parameters = [
        moveit_config.to_dict(),
        {"use_sim_time": use_sim_time},
    ]

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                moveit_config.robot_description,
                {"use_sim_time": use_sim_time},
            ],
            output="screen",
        ),
        Node(
            package="six_motor_driver",
            executable="six_motor_driver",
            condition=IfCondition(start_hardware),
            parameters=[
                str(calibration),
                {
                    "port": LaunchConfiguration("port"),
                    "center_on_start": ParameterValue(
                        LaunchConfiguration("center_on_start"),
                        value_type=bool,
                    ),
                    "bridge_sensor_wrap": ParameterValue(
                        LaunchConfiguration("bridge_sensor_wrap"),
                        value_type=bool,
                    ),
                    "wrap_rotate_speed_steps_s": ParameterValue(
                        LaunchConfiguration("wrap_rotate_speed_steps_s"),
                        value_type=int,
                    ),
                },
            ],
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=common_parameters,
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(moveit_share / "config" / "moveit.rviz")],
            parameters=common_parameters,
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_mode",
                default_value="real",
                description="real, sim oder mirror",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "start_hardware",
                default_value="true",
                description=(
                    "true startet den echten six_motor_driver direkt mit"
                ),
            ),
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("center_on_start", default_value="false"),
            DeclareLaunchArgument("bridge_sensor_wrap", default_value="true"),
            DeclareLaunchArgument(
                "wrap_rotate_speed_steps_s", default_value="80"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
