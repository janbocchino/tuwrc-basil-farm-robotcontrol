"""Project defaults for the physical Orbbec Gemini 2 camera."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    vendor_launch = (
        Path(get_package_share_directory("orbbec_camera"))
        / "launch"
        / "gemini2.launch.py"
    )

    arguments = [
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("serial_number", default_value=""),
        DeclareLaunchArgument("usb_port", default_value=""),
        DeclareLaunchArgument("color_width", default_value="0"),
        DeclareLaunchArgument("color_height", default_value="0"),
        DeclareLaunchArgument("color_fps", default_value="0"),
        DeclareLaunchArgument("depth_width", default_value="0"),
        DeclareLaunchArgument("depth_height", default_value="0"),
        DeclareLaunchArgument("depth_fps", default_value="0"),
        DeclareLaunchArgument(
            "enable_colored_point_cloud",
            default_value="false",
            description="Publish the additional RGB-colored registered cloud.",
        ),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    forwarded = {
        name: LaunchConfiguration(name)
        for name in (
            "camera_name",
            "serial_number",
            "usb_port",
            "color_width",
            "color_height",
            "color_fps",
            "depth_width",
            "depth_height",
            "depth_fps",
            "enable_colored_point_cloud",
            "log_level",
        )
    }
    project_defaults = {
        "enable_color": "true",
        "enable_depth": "true",
        "enable_ir": "false",
        "enable_accel": "false",
        "enable_gyro": "false",
        "depth_registration": "true",
        "enable_point_cloud": "true",
        "enable_frame_sync": "true",
        "publish_tf": "true",
    }

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(vendor_launch)),
        launch_arguments={**project_defaults, **forwarded}.items(),
    )

    return LaunchDescription([*arguments, camera])
