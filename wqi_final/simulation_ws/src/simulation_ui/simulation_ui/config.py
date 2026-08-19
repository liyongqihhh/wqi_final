from dataclasses import dataclass
from enum import Enum
import os
from pathlib import Path
import shlex


def _find_repository_root(workspace: Path) -> Path:
    for candidate in (workspace, *workspace.parents):
        if (candidate / ".git").exists():
            return candidate
    return workspace.parent.parent


def _discover_workspace() -> Path:
    configured = os.environ.get("WQI_WORKSPACE")
    if configured:
        return Path(configured).expanduser().resolve()

    source_file = Path(__file__).resolve()
    for candidate in source_file.parents:
        if (candidate / "src" / "simulation_ui" / "package.xml").is_file():
            return candidate

    current = Path.cwd().resolve()
    for candidate in (current, *current.parents):
        if (candidate / "src" / "simulation_ui" / "package.xml").is_file():
            return candidate
    return current


def _default_build_root(workspace: Path) -> Path:
    configured = os.environ.get("WQI_BUILD_ROOT")
    if configured:
        return Path(configured).expanduser().resolve()
    repository = _find_repository_root(workspace)
    return repository.parent / f"{repository.name}_artifacts"


UAV_BATTERY_RESERVE_PERCENT = 20
UAV_LOW_ENERGY_WARNING_PERCENT = 40
UAV_RECOMMENDED_TEST_PERCENT = 80
UGV_DRIVE_BATTERY_RESERVE_PERCENT = 20
UGV_CHARGING_BATTERY_RESERVE_PERCENT = 10
MAX_DELIVERY_ITEMS = 10
MAX_UAV_PAYLOAD_KG = 1.0


class ViewerMode(str, Enum):
    RVIZ = "rviz"
    GAZEBO = "gazebo"
    BOTH = "both"


@dataclass(frozen=True)
class BatteryNotice:
    severity: str
    message: str
    requires_confirmation: bool = False


@dataclass(frozen=True)
class Building:
    target_id: str
    label: str
    maximum_floor: int
    default_floor: int
    default_payload_kg: float

    def altitude_for_floor(self, floor: int) -> float:
        if not 1 <= floor <= self.maximum_floor:
            raise ValueError(
                f"Floor {floor} is outside 1..{self.maximum_floor} for "
                f"{self.target_id}"
            )
        return round(1.6 + (floor - 1) * 3.2, 3)


@dataclass(frozen=True)
class DeliveryItem:
    target_id: str
    floor: int
    payload_kg: float


@dataclass(frozen=True)
class ObstacleDensity:
    key: str
    label: str
    obstacle_count: int


OBSTACLE_DENSITIES = (
    ObstacleDensity("none", "无动态障碍（0）", 0),
    ObstacleDensity("low", "低密度（3）", 3),
    ObstacleDensity("medium", "中密度（6）", 6),
    ObstacleDensity("high", "高密度（10）", 10),
)
OBSTACLE_DENSITY_BY_KEY = {
    density.key: density for density in OBSTACLE_DENSITIES
}


BUILDINGS = (
    Building("teaching_building", "教学楼", 8, 3, 0.30),
    Building("laboratory", "实验楼", 9, 4, 0.35),
    Building("library", "图书馆", 7, 3, 0.25),
    Building("innovation_center", "创新中心", 6, 3, 0.30),
    Building("cafeteria", "食堂", 5, 2, 0.25),
    Building("gymnasium", "体育馆", 3, 2, 0.30),
    Building("dormitory_1", "宿舍 1 栋", 10, 3, 0.20),
    Building("dormitory_2", "宿舍 2 栋", 11, 4, 0.20),
    Building("dormitory_3", "宿舍 3 栋", 12, 3, 0.20),
    Building("dormitory_4", "宿舍 4 栋", 13, 4, 0.20),
)
BUILDING_BY_ID = {building.target_id: building for building in BUILDINGS}


@dataclass(frozen=True)
class SimulationMode:
    key: str
    label: str
    uses_uav: bool
    supports_floor: bool
    supports_payload: bool
    supports_battery_input: bool
    has_route_command: bool
    uses_energy_model: bool
    supports_dynamic_obstacles: bool


SIMULATION_MODES = (
    SimulationMode(
        key="indoor_ugv",
        label="1  室内 UGV 标点导航",
        uses_uav=False,
        supports_floor=False,
        supports_payload=False,
        supports_battery_input=False,
        has_route_command=False,
        uses_energy_model=False,
        supports_dynamic_obstacles=False,
    ),
    SimulationMode(
        key="campus_ugv",
        label="2  校园 UGV 自动导航",
        uses_uav=False,
        supports_floor=False,
        supports_payload=False,
        supports_battery_input=False,
        has_route_command=True,
        uses_energy_model=False,
        supports_dynamic_obstacles=False,
    ),
    SimulationMode(
        key="campus_uav",
        label="3  校园 UAV 自动导航",
        uses_uav=True,
        supports_floor=True,
        supports_payload=True,
        supports_battery_input=False,
        has_route_command=True,
        uses_energy_model=False,
        supports_dynamic_obstacles=False,
    ),
    SimulationMode(
        key="cooperative",
        label="4  空地协同自动导航",
        uses_uav=True,
        supports_floor=True,
        supports_payload=True,
        supports_battery_input=False,
        has_route_command=True,
        uses_energy_model=False,
        supports_dynamic_obstacles=False,
    ),
    SimulationMode(
        key="cooperative_energy",
        label="5  空地协同与三电池",
        uses_uav=True,
        supports_floor=True,
        supports_payload=True,
        supports_battery_input=True,
        has_route_command=True,
        uses_energy_model=True,
        supports_dynamic_obstacles=False,
    ),
    SimulationMode(
        key="cooperative_dynamic_energy",
        label="6  动态障碍协同配送",
        uses_uav=True,
        supports_floor=True,
        supports_payload=True,
        supports_battery_input=True,
        has_route_command=True,
        uses_energy_model=True,
        supports_dynamic_obstacles=True,
    ),
)
MODE_BY_KEY = {mode.key: mode for mode in SIMULATION_MODES}


def battery_admission_notice(
    mode_key: str,
    battery_percent: int,
    ugv_drive_battery_percent: int = UAV_RECOMMENDED_TEST_PERCENT,
    ugv_charging_battery_percent: int = UAV_RECOMMENDED_TEST_PERCENT,
) -> BatteryNotice:
    """Return UI guidance without replacing the runtime energy planner."""
    mode = MODE_BY_KEY[mode_key]
    percentage = int(battery_percent)
    drive_percentage = int(ugv_drive_battery_percent)
    charging_percentage = int(ugv_charging_battery_percent)
    if not mode.supports_battery_input:
        return BatteryNotice("normal", "")
    critical = []
    if percentage <= UAV_BATTERY_RESERVE_PERCENT:
        critical.append(f"UAV {percentage}%")
    if drive_percentage <= UGV_DRIVE_BATTERY_RESERVE_PERCENT:
        critical.append(f"UGV 驱动电池 {drive_percentage}%")
    if charging_percentage <= UGV_CHARGING_BATTERY_RESERVE_PERCENT:
        critical.append(f"UGV 充电电池 {charging_percentage}%")
    if critical:
        return BatteryNotice(
            "critical",
            "以下电池不高于安全储备：" + "、".join(critical) + "。"
            "任务会由 UAV 架次规划器和 UGV 驱动规划器联合判断；"
            f"完整流程建议三块电池均使用 {UAV_RECOMMENDED_TEST_PERCENT}%。",
            True,
        )
    low = []
    if percentage < UAV_LOW_ENERGY_WARNING_PERCENT:
        low.append(f"UAV {percentage}%")
    if drive_percentage < UAV_LOW_ENERGY_WARNING_PERCENT:
        low.append(f"UGV 驱动电池 {drive_percentage}%")
    if charging_percentage < UAV_LOW_ENERGY_WARNING_PERCENT:
        low.append(f"UGV 充电电池 {charging_percentage}%")
    if low:
        return BatteryNotice(
            "warning",
            "以下电池电量较低：" + "、".join(low) + "。高楼层、远距离、"
            "重载荷或多任务可能因任务能耗与安全储备不足而被拒绝；"
            f"首次完整测试建议使用 {UAV_RECOMMENDED_TEST_PERCENT}%。",
            True,
        )
    return BatteryNotice(
        "normal",
        f"初始电量：UAV {percentage}%，UGV 驱动 {drive_percentage}%，"
        f"UGV 充电 {charging_percentage}%。发送后会按路线、楼层、载荷"
        "和安全储备执行联合能量准入检查。",
    )


@dataclass(frozen=True)
class CommandSpec:
    label: str
    command: str
    delay_seconds: float = 0.0


class CommandBuilder:
    def __init__(
        self,
        workspace: Path | None = None,
        build_root: Path | None = None,
    ) -> None:
        self.workspace = Path(workspace or _discover_workspace()).resolve()
        self.build_root = Path(
            build_root or _default_build_root(self.workspace)
        ).resolve()

    @property
    def setup_file(self) -> Path:
        return self.build_root / "install" / "setup.bash"

    @staticmethod
    def _flags(viewer: ViewerMode) -> tuple[str, str]:
        gui = viewer in (ViewerMode.GAZEBO, ViewerMode.BOTH)
        rviz = viewer in (ViewerMode.RVIZ, ViewerMode.BOTH)
        return str(gui).lower(), str(rviz).lower()

    def simulation_commands(
        self,
        mode_key: str,
        viewer: ViewerMode,
        battery_percent: int,
        visualize_sensor_rays: bool,
        ugv_drive_battery_percent: int = UAV_RECOMMENDED_TEST_PERCENT,
        ugv_charging_battery_percent: int = UAV_RECOMMENDED_TEST_PERCENT,
        obstacle_density: str = "none",
    ) -> list[CommandSpec]:
        if mode_key not in MODE_BY_KEY:
            raise ValueError(f"Unknown simulation mode: {mode_key}")
        if not 0 <= int(battery_percent) <= 100:
            raise ValueError("Battery percentage must be in the range 0..100")
        if not 0 <= int(ugv_drive_battery_percent) <= 100:
            raise ValueError("UGV drive battery must be in the range 0..100")
        if not 0 <= int(ugv_charging_battery_percent) <= 100:
            raise ValueError("UGV charging battery must be in the range 0..100")
        if obstacle_density not in OBSTACLE_DENSITY_BY_KEY:
            raise ValueError(f"Unknown obstacle density: {obstacle_density}")
        gui, rviz = self._flags(viewer)
        rays = str(bool(visualize_sensor_rays)).lower()
        soc = int(battery_percent) / 100.0
        drive_soc = int(ugv_drive_battery_percent) / 100.0
        charging_soc = int(ugv_charging_battery_percent) / 100.0

        if mode_key == "indoor_ugv":
            return [
                CommandSpec(
                    "室内 Gazebo",
                    "ros2 launch ugvcar_description gazebo_sim.launch.py "
                    f"gui:={gui}",
                ),
                CommandSpec(
                    "室内 Nav2 / RViz",
                    "ros2 launch ugvcar_navigation2 navigation2.launch.py "
                    f"rviz:={rviz}",
                    4.0,
                ),
            ]
        if mode_key == "campus_ugv":
            return [
                CommandSpec(
                    "校园 UGV Gazebo",
                    "ros2 launch ugvcar_description "
                    "campus_delivery_sim.launch.py "
                    f"gui:={gui} visualize_sensor_rays:={rays}",
                ),
                CommandSpec(
                    "校园 UGV Nav2 / RViz",
                    "ros2 launch ugvcar_navigation2 "
                    "campus_navigation.launch.py "
                    f"rviz:={rviz} localization_mode:=ground_truth",
                    5.0,
                ),
            ]
        if mode_key == "campus_uav":
            return [CommandSpec(
                "校园 UAV 仿真",
                "ros2 launch uav_bringup uav_sim.launch.py "
                f"gui:={gui} rviz:={rviz} "
                "enable_energy_constraints:=false "
                "initial_battery_percentage:=1.00 "
                f"visualize_sensor_rays:={rays}",
            )]

        energy_enabled = mode_key in (
            "cooperative_energy",
            "cooperative_dynamic_energy",
        )
        dynamic_enabled = mode_key == "cooperative_dynamic_energy"
        cooperative_soc = soc if energy_enabled else 1.0
        cooperative_drive_soc = drive_soc if energy_enabled else 1.0
        cooperative_charging_soc = charging_soc if energy_enabled else 1.0
        return [CommandSpec(
            "校园协同仿真",
            "ros2 launch cooperative_delivery "
            "cooperative_delivery.launch.py "
            f"gui:={gui} rviz:={rviz} "
            f"initial_battery_percentage:={cooperative_soc:.2f} "
            "initial_ugv_drive_battery_percentage:="
            f"{cooperative_drive_soc:.2f} "
            "initial_ugv_charging_battery_percentage:="
            f"{cooperative_charging_soc:.2f} "
            f"enable_energy_constraints:={str(energy_enabled).lower()} "
            f"enable_dynamic_obstacles:={str(dynamic_enabled).lower()} "
            f"obstacle_density:={obstacle_density if dynamic_enabled else 'none'} "
            f"visualize_sensor_rays:={rays}",
        )]

    def task_command(
        self,
        mode_key: str,
        target_id: str,
        floor: int,
        payload_kg: float,
        return_home: bool,
    ) -> CommandSpec | None:
        return self.delivery_task_command(
            mode_key,
            [DeliveryItem(target_id, floor, payload_kg)],
            return_home,
        )

    def delivery_task_command(
        self,
        mode_key: str,
        items: list[DeliveryItem],
        return_home: bool,
    ) -> CommandSpec | None:
        mode = MODE_BY_KEY[mode_key]
        if not mode.has_route_command:
            return None
        delivery_items = list(items)
        if not delivery_items:
            raise ValueError("At least one delivery item is required")
        if len(delivery_items) > MAX_DELIVERY_ITEMS:
            raise ValueError(
                f"At most {MAX_DELIVERY_ITEMS} delivery items are supported"
            )

        validated = []
        for index, item in enumerate(delivery_items, start=1):
            if item.target_id not in BUILDING_BY_ID:
                raise ValueError(
                    f"Unknown delivery target for item {index}: {item.target_id}"
                )
            building = BUILDING_BY_ID[item.target_id]
            floor = int(item.floor)
            building.altitude_for_floor(floor)
            payload = float(item.payload_kg)
            if not 0.0 <= payload <= MAX_UAV_PAYLOAD_KG:
                raise ValueError(
                    f"Item {index} payload must be in the range "
                    f"0..{MAX_UAV_PAYLOAD_KG:.1f} kg"
                )
            validated.append(DeliveryItem(item.target_id, floor, payload))

        if (
            mode_key == "campus_uav"
            and sum(item.payload_kg for item in validated)
            > MAX_UAV_PAYLOAD_KG + 1e-9
        ):
            raise ValueError(
                "Standalone UAV carries all selected items in one flight; "
                f"their total payload cannot exceed {MAX_UAV_PAYLOAD_KG:.1f} kg"
            )

        target_values = ", ".join(item.target_id for item in validated)
        floor_values = ", ".join(str(item.floor) for item in validated)
        payload_values = ", ".join(
            f"{item.payload_kg:.3f}" for item in validated
        )

        if mode_key == "campus_ugv":
            targets_argument = f'delivery_targets:="[{target_values}]"'
            return CommandSpec(
                "UGV 配送路线",
                "ros2 launch ugvcar_application delivery_task.launch.py "
                f"{targets_argument} wait_duration:=10.0",
            )

        action_type = "uav_interfaces/action/ExecuteDelivery"
        action_name = "/uav/execute_delivery"
        if mode_key in (
            "cooperative",
            "cooperative_energy",
            "cooperative_dynamic_energy",
        ):
            action_type = (
                "cooperative_delivery_interfaces/action/"
                "ExecuteCooperativeDelivery"
            )
            action_name = "/cooperative_delivery/execute_mission"
        goal = (
            f"{{targets: [{target_values}], "
            f"return_home: {str(bool(return_home)).lower()}, "
            f"target_floors: [{floor_values}], "
            f"payload_masses_kg: [{payload_values}]}}"
        )
        return CommandSpec(
            "配送任务",
            f"env PYTHONUNBUFFERED=1 ros2 action send_goal "
            f"{action_name} {action_type} "
            f"{shlex.quote(goal)} --feedback",
        )

    def shell_command(self, command: str) -> str:
        setup = shlex.quote(str(self.setup_file))
        return (
            "source /opt/ros/humble/setup.bash && "
            f"source {setup} && exec {command}"
        )
