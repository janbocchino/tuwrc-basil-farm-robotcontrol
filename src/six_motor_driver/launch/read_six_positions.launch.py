from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    driver_share = Path(get_package_share_directory("six_motor_driver"))
    calibration_file = (
        driver_share / "config" / "six_motor_calibration.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("read_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("print_rate_hz", default_value="2.0"),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/six_motor/joint_states",
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=str(calibration_file),
            ),
            Node(
                package="six_motor_driver",
                executable="six_motor_position_reader",
                parameters=[
                    LaunchConfiguration("calibration_file"),
                    {
                        "port": LaunchConfiguration("port"),
                        "read_rate_hz": ParameterValue(
                            LaunchConfiguration("read_rate_hz"),
                            value_type=float,
                        ),
                        "print_rate_hz": ParameterValue(
                            LaunchConfiguration("print_rate_hz"),
                            value_type=float,
                        ),
                        "joint_states_topic": LaunchConfiguration(
                            "joint_states_topic"
                        ),
                    }
                ],
                output="screen",
            ),
        ]
    )
