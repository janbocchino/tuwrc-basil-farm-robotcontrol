from setuptools import setup

package_name = "lerobot_gui"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="David Buening",
    maintainer_email="david@example.com",
    description="Small browser-based tools for monitoring and controlling the LeRobot arm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "joint_state_gui = lerobot_gui.joint_state_gui:main",
        ],
    },
)
