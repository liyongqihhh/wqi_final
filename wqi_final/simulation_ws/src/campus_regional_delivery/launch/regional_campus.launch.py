"""One Gazebo/PX4 backend, central carrier plus three regional carriers."""
import os
from pathlib import Path
import tempfile
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory,get_package_prefix,PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, SetLaunchConfiguration,
                            Shutdown, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer,Node,SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


def gazebo_exited(event, context):
    """Fail closed on a server crash, without a second shutdown during Ctrl+C."""
    details = getattr(event.action, "process_details", {})
    if details.get("name", "").startswith("gzserver-") and not context.is_shutdown:
        return [Shutdown(reason="Regional Gazebo server exited")]
    return []


def rviz_environment(driver):
    """Keep the VM graphics workaround local to the RViz process."""
    if driver == "softpipe":
        return {"GALLIUM_DRIVER":"softpipe","DRAW_USE_LLVM":"0"}
    if driver == "llvmpipe":
        return {"GALLIUM_DRIVER":"llvmpipe"}
    raise ValueError("rviz_render_driver must be softpipe or llvmpipe")


def prepare(context):
    from campus_regional_delivery.layout import RegionalLayout
    from campus_regional_delivery.fleet import (fleet_model,fleet_parameters,fleet_rviz,
        prepare_uav_model,NAVIGATION_COMPONENTS)
    from cooperative_delivery.relay_vehicle import second_vehicle_remappings
    def share(name):
        return Path(get_package_share_directory(name))
    def value(name):
        return LaunchConfiguration(name).perform(context)
    def enabled(name):
        return value(name).lower() in ("true","yes","1")
    middleware=value("rmw_implementation")
    try:
        get_package_prefix(middleware)
    except PackageNotFoundError as error:
        raise RuntimeError("ROS middleware is missing: "+middleware+
            ". For this scene install: sudo apt install ros-humble-rmw-cyclonedds-cpp") from error
    package = share("campus_regional_delivery")
    gate=Path(get_package_prefix("campus_regional_delivery"))/"lib/campus_regional_delivery/wait_for_regional_navigation"
    if not os.access(gate,os.X_OK):
        raise RuntimeError("Regional startup helper is not executable; check source script permissions and rebuild the workspace")
    import socket
    address=urlparse(context.environment.get("GAZEBO_MASTER_URI","http://localhost:11345"))
    try:
        connection=socket.create_connection((address.hostname or "localhost",address.port or 11345),timeout=.5)
    except OSError:
        pass
    else:
        connection.close()
        raise RuntimeError("The selected Gazebo master is occupied; stop the previous simulation first")
    layout = RegionalLayout(value("layout_file"))
    delivery_mode=value("delivery_mode")
    if delivery_mode not in ("cooperative","ugv_only"):
        raise ValueError("delivery_mode must be cooperative or ugv_only")
    with_uav=enabled("uav") and delivery_mode=="cooperative"
    assets = package/"assets"
    # Refuse stale generated assets instead of silently mixing coordinate sets.
    import hashlib,json
    manifest=json.loads((assets/"layout_validation.json").read_text())
    if hashlib.sha256(layout.path.read_bytes()).hexdigest() != manifest["layout_sha256"]:
        raise RuntimeError("Layout changed: regenerate regional assets before starting Gazebo")
    if with_uav:
        import socket
        for kind,port in ((socket.SOCK_STREAM,4560),(socket.SOCK_DGRAM,8888)):
            with socket.socket(socket.AF_INET,kind) as probe:
                if kind==socket.SOCK_STREAM:
                    probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                try:
                    probe.bind(("0.0.0.0",port))
                except OSError as error:
                    raise RuntimeError("Stop the previous PX4 simulation before starting this scene") from error
    generated_root=Path(os.environ.get("WQI_BUILD_ROOT","/tmp"))/"generated_models"
    generated_root.mkdir(parents=True,exist_ok=True)
    generated=Path(tempfile.mkdtemp(prefix="regional_campus_",dir=generated_root))
    original=xacro.process_file(str(share("ugvcar_description")/"urdf/ugvcar/ugvcar.urdf.xacro"),
        mappings={"visualize_sensor_rays":value("visualize_sensor_rays")}).toxml()
    original_params=yaml.safe_load((share("ugvcar_navigation2")/"config/nav2_params.yaml").read_text())
    namespaces=[r["namespace"] for r in layout.vehicles.values()]
    entities=[]
    for r in layout.vehicles.values():
        ns=r["namespace"]
        description=fleet_model(original,ns)
        params=generated/f"{ns}_nav2.yaml"
        params.write_text(yaml.safe_dump(fleet_parameters(
            original_params,share("ugvcar_navigation2"),ns,namespaces,
            map_resolution=layout.resolution)))
        entities.append(Node(package="robot_state_publisher",executable="robot_state_publisher",namespace=ns,
            name="robot_state_publisher",parameters=[{"use_sim_time":True,
            "robot_description":ParameterValue(description,value_type=str)}],output="screen"))
    rviz=generated/"regional.rviz"
    rviz.write_text(yaml.safe_dump(fleet_rviz(yaml.safe_load((share("cooperative_delivery")/"rviz/cooperative.rviz").read_text()),layout.vehicles,with_uav)))
    def include(path,arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),launch_arguments=arguments.items())
    entities += [RegisterEventHandler(OnProcessExit(on_exit=gazebo_exited)),
        include(share("gazebo_ros")/"launch/gazebo.launch.py",
        {"world":str(assets/"campus_regional.world"),"gui":value("gui"),"verbose":"true","server_required":"false"})]
    shared=[]
    lifecycle=[]
    for name,topic,yaml_file in (("regional_map_server","/map","regional_map.yaml"),
        ("regional_global_mask","/global_keepout_filter_mask","regional_keepout.yaml"),
        ("regional_local_mask","/local_keepout_filter_mask","regional_keepout.yaml")):
        shared.append(Node(package="nav2_map_server",executable="map_server",name=name,output="screen",
            parameters=[{"use_sim_time":True,"yaml_filename":str(assets/yaml_file),"topic_name":topic}]))
        lifecycle.append(name)
    for kind in ("global","local"):
        name=f"regional_{kind}_filter_info"
        shared.append(Node(package="ugvcar_navigation2",executable="managed_filter_info_server",name=name,
            output="screen",parameters=[{"use_sim_time":True,"type":0,"base":0.,"multiplier":1.,
            "filter_info_topic":f"/{kind}_costmap_filter_info","mask_topic":f"/{kind}_keepout_filter_mask"}]))
        lifecycle.append(name)
    shared.append(Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",name="regional_map_lifecycle",
        output="screen",parameters=[{"use_sim_time":True,"autostart":True,"node_names":lifecycle}]))
    entities += shared
    nav_stages=[]
    nodes=tuple((package,name) for name,(package,_) in NAVIGATION_COMPONENTS.items())
    for ns in namespaces:
        stage=[]
        components=[]
        stage.append(Node(package="campus_regional_delivery",executable="regional_localizer",namespace=ns,
            output="screen",parameters=[{"use_sim_time":True}]))
        for pkg,exe in nodes:
            remaps=[(a,"/"+ns+b[len("/ugv_b"):]) for a,b in second_vehicle_remappings()]
            if exe in ("controller_server","behavior_server"):
                remaps += [("cmd_vel","cmd_vel_nav")]
            if exe=="velocity_smoother":
                remaps += [("cmd_vel","cmd_vel_nav"),("cmd_vel_smoothed","cmd_vel")]
            params=[str(generated/f"{ns}_nav2.yaml"),{"use_sim_time":True,"bond_timeout":30.}]
            if enabled("use_composition"):
                components.append(ComposableNode(package=pkg,plugin=NAVIGATION_COMPONENTS[exe][1],
                    name=exe,namespace=ns,parameters=params,remappings=remaps))
            else:
                stage.append(Node(package=pkg,executable=exe,name=exe,namespace=ns,output="screen",
                    parameters=params,remappings=remaps))
        lifecycle_params={"use_sim_time":True,"autostart":True,"bond_timeout":30.,
            "node_names":[exe for _,exe in nodes]}
        if enabled("use_composition"):
            components.append(ComposableNode(package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",name="lifecycle_manager_navigation",
                namespace=ns,parameters=[lifecycle_params]))
            # Unload components before stopping their ROS context.
            stage.append(ComposableNodeContainer(package="campus_regional_delivery",executable="regional_nav_container",
                name="nav2_container",namespace=ns,output="screen",
                arguments=["--vehicle-namespace",ns],sigterm_timeout="45.0",sigkill_timeout="10.0",
                parameters=[str(generated/f"{ns}_nav2.yaml"),{"use_sim_time":True}],
                composable_node_descriptions=components))
        else:
            stage.append(Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",name="lifecycle_manager_navigation",
                namespace=ns,output="screen",parameters=[lifecycle_params]))
        waiter=Node(package="campus_regional_delivery",executable="wait_for_regional_navigation",
            name="wait_for_"+ns,output="screen",parameters=[{"use_sim_time":True,
            "vehicle_namespace":ns,"timeout_wall_s":240.}])
        nav_stages.append(stage+[waiter])
    extra=[Node(package="campus_regional_delivery",executable="regional_markers",output="screen",
        parameters=[{"use_sim_time":True,"layout_file":value("layout_file")}]),
        TimerAction(period=10.,actions=[Node(package="rviz2",executable="rviz2",name="regional_rviz",
        arguments=["-d",str(rviz)],parameters=[{"use_sim_time":True}],
        additional_env=rviz_environment(value("rviz_render_driver")),
        output="screen",condition=IfCondition(value("rviz")))])]
    aircraft_start=[]
    aircraft_inserted=[False]
    if with_uav:
        px4=Path(value("px4_autopilot_dir"))
        uav_sdf=generated/"regional_uav.sdf"
        prepare_uav_model(px4/"Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf",
            px4/"build/px4_sitl_default/build_gazebo-classic/libgazebo_imu_plugin.so",share("uav_description"),
            namespaces,uav_sdf,enabled("visualize_sensor_rays"))
        uav_xml=xacro.process_file(str(share("uav_description")/"urdf/uav.urdf.xacro"),mappings={
            "params_path":str(share("uav_description")/"config/uav_model.yaml"),"enable_docking":"true",
            "visualize_sensor_rays":value("visualize_sensor_rays")}).toxml()
        hx,hy,hz,hyaw=layout.hub["uav_pad"]
        aircraft_spawn=Node(package="gazebo_ros",executable="spawn_entity.py",name="spawn_regional_uav",output="screen",
                arguments=["-entity","campus_uav","-file",str(uav_sdf),"-robot_namespace","/uav",
                "-timeout","180","-x",str(hx),"-y",str(hy),"-z",str(hz),"-Y",str(hyaw)])
        aircraft_start = [Node(package="robot_state_publisher",executable="robot_state_publisher",namespace="uav",
            name="robot_state_publisher",parameters=[{"use_sim_time":True,"frame_prefix":"uav/",
            "robot_description":ParameterValue(uav_xml,value_type=str)}]),
            Node(package="tf2_ros",executable="static_transform_publisher",name="uav_map_to_odom",
                arguments=["--frame-id","map","--child-frame-id","uav/odom"]),
            aircraft_spawn,
            include(share("uav_px4_bringup")/"launch/px4_sitl.launch.py",{
                "px4_autopilot_dir":str(px4),"moving_base":"true","enable_energy_constraints":"false"}),
            include(share("uav_control")/"launch/uav_control.launch.py",{
                "enable_energy_constraints":"false","enable_sim_velocity_aiding":"true","use_sim_time":"true"})]
    if with_uav or delivery_mode=="ugv_only":
        extra.append(Node(package="campus_regional_delivery",executable="regional_mission_manager",namespace="regional_delivery",
            name="mission_manager",output="screen",parameters=[{"use_sim_time":True,
            "layout_file":value("layout_file"),"delivery_mode":delivery_mode}]))
    spawns=[]
    for r in layout.vehicles.values():
        ns=r["namespace"]
        x,y,z,yaw=r["depot_pose"]
        spawns.append(Node(package="gazebo_ros",executable="spawn_entity.py",name="spawn_"+ns,output="screen",
            arguments=["-topic",f"/{ns}/robot_description","-entity",ns,"-timeout","180",
                       "-x",str(x),"-y",str(y),"-z",str(z),"-Y",str(yaw)]))
    # Finish Gazebo sensor/model creation before bringing up the four Nav2 stacks.
    # Do not let a blocked model insertion starve already-active navigation bonds.
    if with_uav:
        def check_aircraft_insertion(_context):
            if not _context.is_shutdown and not aircraft_inserted[0]:
                return [Shutdown(reason="UAV insertion exceeded 240 wall seconds; inspect Gazebo/PX4 startup logs")]
            return []
        aircraft_start.append(TimerAction(period=240.,actions=[OpaqueFunction(function=check_aircraft_insertion)]))
        def aircraft_spawned(event,_context):
            if _context.is_shutdown:
                return []
            if event.returncode:
                return [Shutdown(reason="Regional UAV model insertion failed")]
            aircraft_inserted[0]=True
            return nav_stages[0]
        entities.append(RegisterEventHandler(OnProcessExit(target_action=aircraft_spawn,on_exit=aircraft_spawned)))
    for index,stage in enumerate(nav_stages):
        def navigation_ready(event,_context,index=index):
            if _context.is_shutdown:
                return []
            if event.returncode:
                return [Shutdown(reason=f"Regional navigation stage {index} did not activate")]
            return nav_stages[index+1] if index+1<len(nav_stages) else extra
        entities.append(RegisterEventHandler(OnProcessExit(target_action=stage[-1],on_exit=navigation_ready)))
    for i,spawn in enumerate(spawns):
        def spawned(event,_context,index=i):
            if _context.is_shutdown:
                return []
            if event.returncode:
                return [Shutdown(reason="Regional UGV spawn failed")]
            return [spawns[index+1]] if index+1<len(spawns) else (aircraft_start if with_uav else nav_stages[0])
        entities.append(RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=spawned)))
    entities.append(spawns[0])
    return entities


def generate_launch_description():
    share=Path(get_package_share_directory("campus_regional_delivery"))
    px4=LaunchConfiguration("px4_autopilot_dir")
    return LaunchDescription([
        DeclareLaunchArgument("layout_file",default_value=str(share/"config/campus_regions.yaml")),
        DeclareLaunchArgument("gui",default_value="true"),DeclareLaunchArgument("rviz",default_value="true"),
        DeclareLaunchArgument("rviz_render_driver",default_value="softpipe",choices=["softpipe","llvmpipe"]),
        DeclareLaunchArgument("uav",default_value="true"),
        DeclareLaunchArgument("delivery_mode",default_value="cooperative"),
        DeclareLaunchArgument("use_composition",default_value="true"),
        DeclareLaunchArgument("rmw_implementation",default_value=EnvironmentVariable(
            "RMW_IMPLEMENTATION",default_value="rmw_cyclonedds_cpp")),
        DeclareLaunchArgument("visualize_sensor_rays",default_value="false"),
        DeclareLaunchArgument("px4_autopilot_dir",default_value=EnvironmentVariable("PX4_AUTOPILOT_DIR",
            default_value="/home/wqi/design_final_artifacts/px4/PX4-Autopilot")),
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE","1"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION",LaunchConfiguration("rmw_implementation")),
        SetEnvironmentVariable("LP_NUM_THREADS",EnvironmentVariable("LP_NUM_THREADS",default_value="2")),
        SetEnvironmentVariable("GAZEBO_PLUGIN_PATH",[px4,"/build/px4_sitl_default/build_gazebo-classic:",
            EnvironmentVariable("GAZEBO_PLUGIN_PATH",default_value="")]),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH",[px4,"/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:",
            EnvironmentVariable("GAZEBO_MODEL_PATH",default_value="")]),
        SetLaunchConfiguration("sigterm_timeout","20.0"),SetLaunchConfiguration("sigkill_timeout","10.0"),
        SetParameter(name="use_sim_time",value=True),SetParameter(name="bond_timeout",value=30.0),
        OpaqueFunction(function=prepare)])
