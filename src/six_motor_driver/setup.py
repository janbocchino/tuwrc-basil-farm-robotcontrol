from glob import glob
import os

from setuptools import find_packages, setup


package_name = "six_motor_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="max",
    maintainer_email="max@example.com",
    description="ROS 2 driver and passive position reader for ST3215 servos",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "calibrate_six_endpoints = six_motor_driver.calibrate_six_endpoints:main",
            "configure_servo_id = six_motor_driver.configure_servo_id:main",
            "print_moveit_limits = six_motor_driver.print_moveit_limits:main",
            "show_six_positions = six_motor_driver.show_six_positions:main",
            "six_motor_driver = six_motor_driver.six_motor_driver:main",
            "six_motor_position_reader = six_motor_driver.six_motor_position_reader:main",
            "st3215_driver = six_motor_driver.st3215_driver:main",
        ],
    },
)
