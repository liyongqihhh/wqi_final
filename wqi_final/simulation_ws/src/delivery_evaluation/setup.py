from glob import glob
import os

from setuptools import find_packages, setup


package_name = "delivery_evaluation"

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
    description="Repeatable campus delivery experiments and thesis reports.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "experiment_runner = delivery_evaluation.experiment_runner:main",
            "experiment_matrix = delivery_evaluation.experiment_matrix:main",
            "generate_report = delivery_evaluation.report_generator:main",
        ],
    },
)
