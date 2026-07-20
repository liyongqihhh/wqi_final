from glob import glob
import os

from setuptools import find_packages, setup


package_name = "uav_navigation"

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
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="wqi",
    maintainer_email="liyongqihhh@users.noreply.github.com",
    description="Campus UAV waypoint and airspace configuration.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "waypoint_visualizer = uav_navigation.waypoint_visualizer:main",
        ],
    },
)
