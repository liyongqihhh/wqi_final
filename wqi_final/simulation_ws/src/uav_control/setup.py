from glob import glob
import os

from setuptools import find_packages, setup


package_name = "uav_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="wqi",
    maintainer_email="liyongqihhh@users.noreply.github.com",
    description="Flight control and collision safety facade for the campus UAV.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "flight_controller = uav_control.flight_controller:main",
            "flight_state_monitor = uav_control.flight_state_monitor:main",
            "safety_monitor = uav_control.safety_monitor:main",
            "battery_manager = uav_control.battery_manager:main",
        ],
    },
)
