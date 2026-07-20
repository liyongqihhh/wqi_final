from glob import glob
import os

from setuptools import find_packages, setup


package_name = "uav_application"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="wqi",
    maintainer_email="liyongqihhh@users.noreply.github.com",
    description="Standalone campus UAV delivery mission manager.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "delivery_mission_manager = uav_application.delivery_mission_manager:main",
        ],
    },
)
