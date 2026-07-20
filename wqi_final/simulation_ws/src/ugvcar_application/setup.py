from setuptools import find_packages, setup

package_name = "ugvcar_application"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/delivery_task.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wqi",
    maintainer_email="liyongqihhh@users.noreply.github.com",
    description="Optimized Nav2 campus delivery manager for the UGV.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "init_robot_pose=ugvcar_application.init_robot_pose:main",
            "delivery_task=ugvcar_application.delivery_task_manager:main",
        ],
    },
)
