from setuptools import setup

package_name = "tuwrc_motion_examples"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TUWRC Team",
    maintainer_email="team@tuwrc.local",
    description="Safe bounded motion examples for the TUWRC robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "small_arm_motion = tuwrc_motion_examples.small_arm_motion:main",
        ],
    },
)
