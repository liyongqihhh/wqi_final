"""One real regional UGV with RViz goal-pose navigation, no delivery manager."""
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
import yaml


def manual_rviz(original, namespace):
    def rewrite(value):
        if isinstance(value, dict):
            return {key: rewrite(item) for key, item in value.items()}
        if isinstance(value, list):
            return [rewrite(item) for item in value]
        if isinstance(value, str) and value.startswith("/ugv_00/"):
            return "/"+namespace+value[len("/ugv_00"):]
        return value
    return rewrite(original)


def prepare(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    share=Path(get_package_share_directory("campus_regional_delivery"))
    original=yaml.safe_load((share/"rviz/ugv_navigation.rviz").read_text())
    root=Path(context.environment.get("WQI_BUILD_ROOT","/tmp"))/"generated_models"
    root.mkdir(parents=True,exist_ok=True)
    folder=Path(tempfile.mkdtemp(prefix="manual_ugv_",dir=root))
    config=folder/"navigation.rviz"
    config.write_text(yaml.safe_dump(manual_rviz(original,value("vehicle"))))
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/"launch/regional_campus.launch.py")),
        launch_arguments={"vehicle":value("vehicle"),"uav":"false","delivery_mode":"ugv_only",
            "start_mission_manager":"false","rviz_config":str(config),"gui":value("gui"),
            "rviz":value("rviz"),"rmw_implementation":value("rmw_implementation")}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vehicle",default_value="ugv_00",choices=["ugv_00","ugv_01","ugv_02","ugv_03"]),
        DeclareLaunchArgument("gui",default_value="false"),
        DeclareLaunchArgument("rviz",default_value="true"),
        DeclareLaunchArgument("rmw_implementation",default_value=EnvironmentVariable(
            "RMW_IMPLEMENTATION",default_value="rmw_cyclonedds_cpp")),
        OpaqueFunction(function=prepare),
    ])
