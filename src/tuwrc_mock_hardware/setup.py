from setuptools import setup

package_name = "tuwrc_mock_hardware"

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
    description="View-mode mock controllers and hardware-mode rail hold.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mock_hardware = tuwrc_mock_hardware.mock_hardware:main",
            "rail_hold = tuwrc_mock_hardware.rail_hold:main",
        ],
    },
)
