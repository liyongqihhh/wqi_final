from pathlib import Path

from setuptools import find_packages, setup


package_name = "simulation_ui"

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
            str(Path("share") / package_name / "resource"),
            ["resource/dashboard.qss"],
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="wqi",
    maintainer_email="liyongqihhh@users.noreply.github.com",
    description="Desktop control panel for campus delivery simulations.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "simulation_dashboard = simulation_ui.app:main",
        ],
    },
)
