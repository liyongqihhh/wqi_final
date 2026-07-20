from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import shlex


DEFAULT_WORKSPACE = Path("/home/wqi/design_final/wqi_final/simulation_ws")
UAV_BATTERY_RESERVE_PERCENT = 20
UAV_LOW_ENERGY_WARNING_PERCENT = 40
UAV_RECOMMENDED_TEST_PERCENT = 80
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


SIMULATION_MODES = (
    SimulationMode(
        "indoor_ugv",
        "1  UGV 房间导航",
        False,
        False,
        False,
        False,
        False,
    ),
    SimulationMode(
        "campus_ugv",
        "2  UGV 校园导航",
        False,
        False,
        False,
        False,
        True,
    ),
    SimulationMode(
        "campus_uav",
        "3  UAV 校园配送",
        True,
        True,
        True,
        True,
        True,
    ),
    SimulationMode(
        "cooperative",
        "4  UGV-UAV 协同",
        True,
        True,
        True,
        False,
        True,
    ),
    SimulationMode(
        "cooperative_energy",
        "5  电量约束协同",
        True,
        True,
        True,
        True,
        True,
    ),
)
MODE_BY_KEY = {mode.key: mode for mode in SIMULATION_MODES}


def battery_admission_notice(
    mode_key: str,
    battery_percent: int,
) -> BatteryNotice:
    """Return UI guidance without replacing the runtime energy planner."""
    mode = MODE_BY_KEY[mode_key]
    percentage = int(battery_percent)
    if not mode.supports_battery_input:
        return BatteryNotice("normal", "")
    if percentage <= UAV_BATTERY_RESERVE_PERCENT:
        if mode_key == "cooperative_energy":
            detail = (
                "只有 UGV 行驶期间充入足够能量后，协同任务才可能通过"
                "起飞准入检查"
            )
        else:
            detail = "独立 UAV 配送会被起飞准入检查拒绝"
        return BatteryNotice(
            "critical",
            f"当前初始电量 {percentage}%，不高于 "
            f"{UAV_BATTERY_RESERVE_PERCENT}% 安全储备；{detail}。"
            f"正常流程建议使用 {UAV_RECOMMENDED_TEST_PERCENT}%。",
            True,
        )
    if percentage < UAV_LOW_ENERGY_WARNING_PERCENT:
        return BatteryNotice(
            "warning",
            f"当前初始电量 {percentage}% 接近能量门槛。高楼层、远距离或"
            "较重载荷仍可能因“任务能耗 + 20% 安全储备”不足而被拒绝；"
            f"首次完整测试建议使用 {UAV_RECOMMENDED_TEST_PERCENT}%。",
            True,
        )
    return BatteryNotice(
        "normal",
        f"当前初始电量 {percentage}%；发送后仍会按路线、楼层、载荷和"
        "安全储备执行实际能量准入检查。",
    )


@dataclass(frozen=True)
class CommandSpec:
    label: str
    command: str
    delay_seconds: float = 0.0


class CommandBuilder:
    def __init__(self, workspace: Path = DEFAULT_WORKSPACE) -> None:
        self.workspace = Path(workspace)

    @property
    def setup_file(self) -> Path:
        return self.workspace / "install" / "setup.bash"

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
    ) -> list[CommandSpec]:
        if mode_key not in MODE_BY_KEY:
            raise ValueError(f"Unknown simulation mode: {mode_key}")
        if not 0 <= int(battery_percent) <= 100:
            raise ValueError("Battery percentage must be in the range 0..100")
        gui, rviz = self._flags(viewer)
        rays = str(bool(visualize_sensor_rays)).lower()
        soc = int(battery_percent) / 100.0

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
                f"initial_battery_percentage:={soc:.2f} "
                f"visualize_sensor_rays:={rays}",
            )]

        cooperative_soc = 1.0 if mode_key == "cooperative" else soc
        return [CommandSpec(
            "校园协同仿真",
            "ros2 launch cooperative_delivery "
            "cooperative_delivery.launch.py "
            f"gui:={gui} rviz:={rviz} "
            f"initial_battery_percentage:={cooperative_soc:.2f} "
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
        if mode_key in ("cooperative", "cooperative_energy"):
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
            f"ros2 action send_goal {action_name} {action_type} "
            f"{shlex.quote(goal)} --feedback",
        )

    def shell_command(self, command: str) -> str:
        setup = shlex.quote(str(self.setup_file))
        return (
            "source /opt/ros/humble/setup.bash && "
            f"source {setup} && exec {command}"
        )
