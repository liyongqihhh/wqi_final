from datetime import datetime
import json
import re

from PyQt5.QtCore import QSettings, Qt, QTimer
from PyQt5.QtGui import QCloseEvent
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QButtonGroup,
    QCheckBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QSpinBox,
    QSplitter,
    QStyle,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from simulation_ui.config import (
    BUILDING_BY_ID,
    MODE_BY_KEY,
    SIMULATION_MODES,
    CommandBuilder,
    DeliveryItem,
    ViewerMode,
    battery_admission_notice,
)
from simulation_ui.cargo_editor import CargoEditor
from simulation_ui.process_manager import ProcessSupervisor


class SimulationDashboard(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("校园物流配送仿真控制台")
        self.setMinimumSize(1080, 720)
        self.resize(1280, 820)
        self.settings = QSettings("wqi_final", "simulation_dashboard")
        self.command_builder = CommandBuilder()
        self.supervisor = ProcessSupervisor(self.command_builder, self)
        self._battery_before_fixed_mode = 80
        self._launched_mode_key = None
        self._launched_battery_percent = None
        self._pending_simulation_commands = None
        self._pending_task = None
        self._task_output_tail = ""
        self._task_failed = False
        self._task_succeeded = False
        self._task_cancel_requested = False
        self._task_message = ""
        self._simulation_start_timer = QTimer(self)
        self._simulation_start_timer.setSingleShot(True)
        self._simulation_start_timer.timeout.connect(
            self._launch_pending_simulation
        )
        self._task_start_timer = QTimer(self)
        self._task_start_timer.setSingleShot(True)
        self._task_start_timer.timeout.connect(self._launch_pending_task)
        self._build_ui()
        self._connect_signals()
        self._restore_settings()
        self._mode_changed(self.mode_list.currentRow())
        self._append_log("控制台", "界面已启动，等待选择仿真模式。")

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        root_layout = QHBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(252)
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(20, 22, 16, 20)
        sidebar_layout.setSpacing(14)
        brand = QLabel("WQI FINAL")
        brand.setObjectName("brand")
        title = QLabel("仿真模式")
        title.setObjectName("sidebarTitle")
        sidebar_layout.addWidget(brand)
        sidebar_layout.addWidget(title)

        self.mode_list = QListWidget()
        self.mode_list.setObjectName("modeList")
        self.mode_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.mode_list.setSpacing(4)
        for mode in SIMULATION_MODES:
            item = QListWidgetItem(mode.label)
            item.setData(Qt.UserRole, mode.key)
            item.setSizeHint(item.sizeHint().expandedTo(item.sizeHint()))
            self.mode_list.addItem(item)
        sidebar_layout.addWidget(self.mode_list, 1)

        workspace_label = QLabel("ROS 2 Humble\nGazebo Classic")
        workspace_label.setObjectName("environmentLabel")
        sidebar_layout.addWidget(workspace_label)
        root_layout.addWidget(sidebar)

        content = QWidget()
        content.setObjectName("content")
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(26, 20, 26, 22)
        content_layout.setSpacing(14)

        header = QHBoxLayout()
        heading_box = QVBoxLayout()
        heading_box.setSpacing(2)
        heading = QLabel("校园物流配送仿真控制台")
        heading.setObjectName("pageTitle")
        self.mode_heading = QLabel()
        self.mode_heading.setObjectName("pageSubtitle")
        heading_box.addWidget(heading)
        heading_box.addWidget(self.mode_heading)
        header.addLayout(heading_box)
        header.addStretch(1)
        self.process_count = QLabel("0 个活动进程")
        self.process_count.setObjectName("processCount")
        self.system_status = QLabel("就绪")
        self.system_status.setObjectName("systemStatus")
        self.system_status.setProperty("state", "ready")
        header.addWidget(self.process_count)
        header.addWidget(self.system_status)
        content_layout.addLayout(header)

        splitter = QSplitter(Qt.Vertical)
        splitter.setChildrenCollapsible(False)
        upper = QWidget()
        upper_layout = QHBoxLayout(upper)
        upper_layout.setContentsMargins(0, 0, 0, 0)
        upper_layout.setSpacing(14)

        configuration = QGroupBox("任务配置")
        configuration.setObjectName("configurationPanel")
        form = QFormLayout(configuration)
        form.setContentsMargins(18, 22, 18, 18)
        form.setHorizontalSpacing(20)
        form.setVerticalSpacing(12)
        form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.cargo_editor = CargoEditor()
        form.addRow("配送货物", self.cargo_editor)

        battery_row = QWidget()
        battery_layout = QHBoxLayout(battery_row)
        battery_layout.setContentsMargins(0, 0, 0, 0)
        battery_layout.setSpacing(10)
        self.battery_slider = QSlider(Qt.Horizontal)
        self.battery_slider.setRange(0, 100)
        self.battery_spin = QSpinBox()
        self.battery_spin.setRange(0, 100)
        self.battery_spin.setSuffix(" %")
        battery_layout.addWidget(self.battery_slider, 1)
        battery_layout.addWidget(self.battery_spin)
        form.addRow("UAV 初始电量", battery_row)

        viewer_row = QWidget()
        viewer_layout = QHBoxLayout(viewer_row)
        viewer_layout.setContentsMargins(0, 0, 0, 0)
        viewer_layout.setSpacing(0)
        self.viewer_group = QButtonGroup(self)
        self.viewer_group.setExclusive(True)
        self.viewer_buttons = {}
        for index, (viewer, text) in enumerate((
            (ViewerMode.RVIZ, "RViz"),
            (ViewerMode.GAZEBO, "Gazebo"),
            (ViewerMode.BOTH, "两者"),
        )):
            button = QToolButton()
            button.setText(text)
            button.setCheckable(True)
            button.setToolButtonStyle(Qt.ToolButtonTextOnly)
            button.setProperty("segment", "middle")
            if index == 0:
                button.setProperty("segment", "left")
            elif index == 2:
                button.setProperty("segment", "right")
            self.viewer_group.addButton(button)
            self.viewer_buttons[viewer] = button
            viewer_layout.addWidget(button)
        self.viewer_buttons[ViewerMode.RVIZ].setChecked(True)
        form.addRow("三维界面", viewer_row)

        self.return_home = QCheckBox("任务完成后返航")
        self.return_home.setChecked(True)
        form.addRow("返航", self.return_home)

        self.sensor_rays = QCheckBox("显示 Gazebo 传感器射线")
        self.sensor_rays.setChecked(False)
        form.addRow("传感器显示", self.sensor_rays)
        upper_layout.addWidget(configuration, 5)

        operation = QGroupBox("运行控制")
        operation.setObjectName("operationPanel")
        operation_layout = QVBoxLayout(operation)
        operation_layout.setContentsMargins(18, 22, 18, 18)
        operation_layout.setSpacing(11)

        self.preview = QPlainTextEdit()
        self.preview.setObjectName("commandPreview")
        self.preview.setReadOnly(True)
        self.preview.setMaximumBlockCount(30)
        self.preview.setPlaceholderText("启动和任务命令")
        operation_layout.addWidget(self.preview, 1)

        button_grid = QGridLayout()
        button_grid.setSpacing(9)
        self.start_button = QPushButton("启动仿真")
        self.start_button.setObjectName("primaryButton")
        self.start_button.setIcon(
            self.style().standardIcon(QStyle.SP_MediaPlay)
        )
        self.start_button.setToolTip("启动当前选择的仿真模式")
        self.run_button = QPushButton("运行配送任务")
        self.run_button.setObjectName("taskButton")
        self.run_button.setIcon(
            self.style().standardIcon(QStyle.SP_DialogApplyButton)
        )
        self.run_button.setToolTip("发送当前配送任务")
        self.stop_task_button = QPushButton("停止任务")
        self.stop_task_button.setIcon(
            self.style().standardIcon(QStyle.SP_BrowserStop)
        )
        self.stop_task_button.setToolTip("停止任务命令，不关闭仿真")
        self.stop_all_button = QPushButton("停止全部")
        self.stop_all_button.setObjectName("dangerButton")
        self.stop_all_button.setIcon(
            self.style().standardIcon(QStyle.SP_DialogCloseButton)
        )
        self.stop_all_button.setToolTip("关闭由控制台启动的全部进程")
        button_grid.addWidget(self.start_button, 0, 0)
        button_grid.addWidget(self.run_button, 0, 1)
        button_grid.addWidget(self.stop_task_button, 1, 0)
        button_grid.addWidget(self.stop_all_button, 1, 1)
        operation_layout.addLayout(button_grid)
        self.stage_note = QLabel()
        self.stage_note.setObjectName("stageNote")
        self.stage_note.setWordWrap(True)
        operation_layout.addWidget(self.stage_note)
        self.task_result = QLabel("任务状态：尚未发送")
        self.task_result.setObjectName("taskResult")
        self.task_result.setProperty("state", "idle")
        self.task_result.setWordWrap(True)
        operation_layout.addWidget(self.task_result)
        upper_layout.addWidget(operation, 4)

        splitter.addWidget(upper)

        log_panel = QGroupBox("运行日志")
        log_layout = QVBoxLayout(log_panel)
        log_layout.setContentsMargins(12, 18, 12, 12)
        self.log_console = QPlainTextEdit()
        self.log_console.setObjectName("logConsole")
        self.log_console.setReadOnly(True)
        self.log_console.setMaximumBlockCount(3000)
        log_layout.addWidget(self.log_console)
        splitter.addWidget(log_panel)
        splitter.setSizes([500, 220])
        content_layout.addWidget(splitter, 1)
        root_layout.addWidget(content, 1)

    def _connect_signals(self) -> None:
        self.mode_list.currentRowChanged.connect(self._mode_changed)
        self.cargo_editor.items_changed.connect(self._configuration_changed)
        self.return_home.toggled.connect(self._configuration_changed)
        self.sensor_rays.toggled.connect(self._configuration_changed)
        self.viewer_group.buttonClicked.connect(self._configuration_changed)
        self.battery_slider.valueChanged.connect(self.battery_spin.setValue)
        self.battery_spin.valueChanged.connect(self.battery_slider.setValue)
        self.battery_spin.valueChanged.connect(self._battery_changed)
        self.start_button.clicked.connect(self._start_simulation)
        self.run_button.clicked.connect(self._run_task)
        self.stop_task_button.clicked.connect(
            self._stop_task
        )
        self.stop_all_button.clicked.connect(self._stop_all)
        self.supervisor.output_received.connect(self._append_process_output)
        self.supervisor.process_changed.connect(self._process_changed)
        self.supervisor.active_count_changed.connect(self._active_count_changed)

    def _restore_settings(self) -> None:
        mode_key = self.settings.value(
            "mode", "cooperative_energy", type=str
        )
        mode_index = next(
            (
                index
                for index, mode in enumerate(SIMULATION_MODES)
                if mode.key == mode_key
            ),
            4,
        )
        self.mode_list.setCurrentRow(mode_index)

        self.cargo_editor.set_items(self._restore_delivery_items())
        battery = self.settings.value("battery", 80, type=int)
        self._battery_before_fixed_mode = battery
        self.battery_spin.setValue(battery)
        viewer_name = self.settings.value("viewer", "rviz", type=str)
        try:
            viewer = ViewerMode(viewer_name)
        except ValueError:
            viewer = ViewerMode.RVIZ
        self.viewer_buttons[viewer].setChecked(True)
        self.return_home.setChecked(
            self.settings.value("return_home", True, type=bool)
        )
        self.sensor_rays.setChecked(
            self.settings.value("sensor_rays", False, type=bool)
        )

    def _save_settings(self) -> None:
        self.settings.setValue("mode", self._current_mode().key)
        items = self.cargo_editor.items()
        self.settings.setValue(
            "delivery_items",
            json.dumps([
                {
                    "target": item.target_id,
                    "floor": item.floor,
                    "payload": item.payload_kg,
                }
                for item in items
            ]),
        )
        first = items[0]
        self.settings.setValue("target", first.target_id)
        self.settings.setValue("floor", first.floor)
        self.settings.setValue("payload", first.payload_kg)
        self.settings.setValue("battery", self._battery_before_fixed_mode)
        self.settings.setValue("viewer", self._current_viewer().value)
        self.settings.setValue("return_home", self.return_home.isChecked())
        self.settings.setValue("sensor_rays", self.sensor_rays.isChecked())

    def _restore_delivery_items(self) -> list[DeliveryItem]:
        serialized = self.settings.value("delivery_items", "", type=str)
        if serialized:
            try:
                values = json.loads(serialized)
                items = [
                    DeliveryItem(
                        str(value["target"]),
                        int(value["floor"]),
                        float(value["payload"]),
                    )
                    for value in values
                ]
                if items and all(
                    item.target_id in BUILDING_BY_ID for item in items
                ):
                    return items
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                pass
        target = self.settings.value(
            "target", "teaching_building", type=str
        )
        if target not in BUILDING_BY_ID:
            target = "teaching_building"
        building = BUILDING_BY_ID[target]
        floor = self.settings.value(
            "floor", building.default_floor, type=int
        )
        payload = self.settings.value(
            "payload", building.default_payload_kg, type=float
        )
        return [DeliveryItem(target, floor, payload)]

    def _current_mode(self):
        item = self.mode_list.currentItem()
        key = item.data(Qt.UserRole) if item else SIMULATION_MODES[0].key
        return MODE_BY_KEY[key]

    def _current_viewer(self) -> ViewerMode:
        for viewer, button in self.viewer_buttons.items():
            if button.isChecked():
                return viewer
        return ViewerMode.RVIZ

    def _mode_changed(self, _row: int) -> None:
        mode = self._current_mode()
        self.mode_heading.setText(mode.label)
        self.cargo_editor.set_capabilities(
            mode.key != "indoor_ugv",
            mode.supports_floor,
            mode.supports_payload,
        )
        self.return_home.setEnabled(mode.has_route_command)

        if mode.key == "cooperative":
            if self.battery_spin.value() != 100:
                self._battery_before_fixed_mode = self.battery_spin.value()
            self.battery_spin.setValue(100)
            self.battery_spin.setEnabled(False)
            self.battery_slider.setEnabled(False)
        elif mode.supports_battery_input:
            self.battery_spin.setEnabled(True)
            self.battery_slider.setEnabled(True)
            if self.battery_spin.value() == 100:
                self.battery_spin.setValue(self._battery_before_fixed_mode)
        else:
            self.battery_spin.setEnabled(False)
            self.battery_slider.setEnabled(False)

        self._configuration_changed()

    def _battery_changed(self, value: int) -> None:
        if self._current_mode().supports_battery_input:
            self._battery_before_fixed_mode = value
        self._configuration_changed()

    def _update_stage_note(self) -> None:
        mode = self._current_mode()
        severity = "normal"
        if mode.key == "indoor_ugv":
            text = "任务输入：在 RViz 中使用 Nav2 Goal 标点。"
        elif mode.key == "campus_ugv":
            text = (
                "UGV 将按精确最短闭环顺序访问所选位置；楼层、载荷和"
                "电量不参与单车阶段计算。"
            )
        elif mode.key == "cooperative":
            text = (
                "初始电量固定为 100%；系统按 UGV 停靠点的最短闭环"
                "顺序配送全部货物。"
            )
        else:
            notice = battery_admission_notice(
                mode.key, self.battery_spin.value()
            )
            severity = notice.severity
            text = (
                "多件货物将按最短路线排序，楼层和载荷随对应货物一起"
                "进入实际 UAV 配送 Action。\n"
                + notice.message
            )
        self.stage_note.setText(text)
        self.stage_note.setProperty("severity", severity)
        self.stage_note.style().unpolish(self.stage_note)
        self.stage_note.style().polish(self.stage_note)

    def _configuration_changed(self, *_args) -> None:
        self._update_stage_note()
        self._update_preview()
        self._refresh_buttons()

    def _simulation_commands(self):
        mode = self._current_mode()
        return self.command_builder.simulation_commands(
            mode.key,
            self._current_viewer(),
            self.battery_spin.value(),
            self.sensor_rays.isChecked(),
        )

    def _task_command(self):
        mode = self._current_mode()
        return self.command_builder.delivery_task_command(
            mode.key,
            self.cargo_editor.items(),
            self.return_home.isChecked(),
        )

    def _commands(self):
        return self._simulation_commands(), self._task_command()

    def _update_preview(self) -> None:
        try:
            commands, task = self._commands()
            lines = []
            for spec in commands:
                suffix = (
                    f"  # 延迟 {spec.delay_seconds:.0f}s"
                    if spec.delay_seconds
                    else ""
                )
                lines.append(f"[启动] {spec.command}{suffix}")
            if task is not None:
                lines.append(f"[任务] {task.command}")
            else:
                lines.append("[任务] RViz Nav2 Goal")
            self.preview.setPlainText("\n\n".join(lines))
        except ValueError as error:
            self.preview.setPlainText(f"参数错误：{error}")

    def _refresh_buttons(self) -> None:
        simulation_active = (
            self.supervisor.active_count("simulation") > 0
            or self.supervisor.has_pending("simulation")
        )
        task_active = self.supervisor.active_count("task") > 0
        self.run_button.setEnabled(
            self._current_mode().has_route_command and simulation_active
        )
        self.stop_task_button.setEnabled(
            task_active or self._task_start_timer.isActive()
        )
        self.stop_all_button.setEnabled(
            self.supervisor.active_count() > 0
            or self._simulation_start_timer.isActive()
            or self._task_start_timer.isActive()
        )

    def _start_simulation(self) -> None:
        if not self.command_builder.setup_file.exists():
            QMessageBox.critical(
                self,
                "工作空间未编译",
                "找不到 simulation_ws/install/setup.bash。请先执行 colcon build。",
            )
            return
        commands = self._simulation_commands()
        self._launched_mode_key = self._current_mode().key
        self._launched_battery_percent = self.battery_spin.value()
        restart = self.supervisor.active_count() > 0
        if restart:
            self.supervisor.stop_all()
            self._append_log("控制台", "正在关闭旧仿真，随后启动当前模式。")
            self._pending_simulation_commands = commands
            self._simulation_start_timer.start(3500)
        else:
            self.supervisor.launch_many(commands, "simulation")
        self._set_status("正在启动", "running")
        self._set_task_result("任务状态：仿真启动中，尚未发送任务", "idle")
        self._append_log(
            "控制台", f"启动模式：{self._current_mode().label}"
        )
        self._save_settings()
        self._refresh_buttons()

    def _run_task(self) -> None:
        try:
            task = self._task_command()
        except ValueError as error:
            QMessageBox.warning(self, "配送参数无效", str(error))
            self._set_task_result(
                f"任务状态：未发送\n{error}", "warning"
            )
            return
        if task is None:
            return
        mode = self._current_mode()
        if self._launched_mode_key != mode.key:
            QMessageBox.warning(
                self,
                "仿真模式不一致",
                "当前运行的仿真不是所选阶段。请先点击“启动仿真”，等待当前"
                "模式启动后再发送任务。",
            )
            self._set_task_result("任务状态：未发送，需先启动当前模式", "warning")
            return
        if (
            mode.supports_battery_input
            and self._launched_battery_percent != self.battery_spin.value()
        ):
            QMessageBox.warning(
                self,
                "初始电量尚未生效",
                f"当前仿真是以 {self._launched_battery_percent}% 电量启动的，"
                f"界面已改为 {self.battery_spin.value()}%。初始电量只在启动时"
                "载入，请点击“启动仿真”重启后再发送任务。",
            )
            self._set_task_result(
                "任务状态：未发送，修改后的电量需要重启仿真才能生效",
                "warning",
            )
            return
        if not self._confirm_low_battery_task():
            self._append_log("控制台", "已取消发送低电量配送任务。")
            self._set_task_result("任务状态：已取消发送，请提高电量并重启仿真", "warning")
            return
        self.supervisor.stop_group("task")
        self._task_output_tail = ""
        self._task_failed = False
        self._task_succeeded = False
        self._task_cancel_requested = False
        self._task_message = ""
        self._pending_task = task
        self._task_start_timer.start(1200)
        items = self.cargo_editor.items()
        total_payload = sum(item.payload_kg for item in items)
        destinations = ", ".join(
            BUILDING_BY_ID[item.target_id].label for item in items
        )
        self._set_task_result("任务状态：等待 Action 服务接收目标", "running")
        self._set_status("任务准备中", "running")
        self._append_log(
            "控制台",
            f"发送 {len(items)} 件货物，总载荷 {total_payload:.2f} kg；"
            f"目标：{destinations}。",
        )
        self._save_settings()
        self._refresh_buttons()

    def _confirm_low_battery_task(self) -> bool:
        mode = self._current_mode()
        notice = battery_admission_notice(
            mode.key, self.battery_spin.value()
        )
        if not notice.requires_confirmation:
            return True
        dialog = QMessageBox(self)
        dialog.setWindowTitle("低电量任务确认")
        dialog.setIcon(QMessageBox.Warning)
        dialog.setText(notice.message)
        dialog.setInformativeText(
            "当前设置可用于验证低电量拒绝和停靠充电逻辑。若要验证完整"
            "配送流程，请先提高初始电量，再点击“启动仿真”重启当前模式。"
        )
        send_button = dialog.addButton(
            "仍然发送低电量任务", QMessageBox.AcceptRole
        )
        cancel_button = dialog.addButton("取消", QMessageBox.RejectRole)
        dialog.setDefaultButton(cancel_button)
        dialog.exec_()
        return dialog.clickedButton() is send_button

    def _launch_pending_simulation(self) -> None:
        commands = self._pending_simulation_commands
        self._pending_simulation_commands = None
        if commands:
            self.supervisor.launch_many(commands, "simulation")

    def _launch_pending_task(self) -> None:
        task = self._pending_task
        self._pending_task = None
        if task is not None:
            self.supervisor.launch_one(task, "task")

    def _stop_task(self) -> None:
        self._task_start_timer.stop()
        self._pending_task = None
        self._task_cancel_requested = True
        self.supervisor.stop_group("task")
        self._set_task_result("任务状态：正在停止", "warning")
        self._refresh_buttons()

    def _stop_all(self) -> None:
        self._simulation_start_timer.stop()
        self._task_start_timer.stop()
        self._pending_simulation_commands = None
        self._pending_task = None
        self._task_cancel_requested = True
        self.supervisor.stop_all()
        self._set_status("正在停止", "stopping")
        self._set_task_result("任务状态：已请求停止", "warning")
        self._append_log("控制台", "已请求停止全部仿真进程。")

    def _append_process_output(self, source: str, text: str) -> None:
        for line in text.rstrip().splitlines():
            if line:
                self._append_log(source, line)
                if source in ("配送任务", "UGV 配送路线"):
                    self._interpret_task_output(line)

    def _interpret_task_output(self, line: str) -> None:
        self._task_output_tail = (self._task_output_tail + "\n" + line)[-6000:]
        lowered = line.lower()
        if "waiting for an action server" in lowered:
            self._set_task_result("任务状态：等待 Action 服务启动", "running")
            return
        if "goal accepted with id" in lowered:
            self._set_task_result("任务状态：目标已接受，正在执行", "running")
            self._set_status("配送执行中", "running")
            return

        rejection = re.search(
            r"REJECT at ([^:]+): takeoff ([0-9.]+) Wh, "
            r"sortie ([0-9.]+) Wh, reserve ([0-9.]+) Wh",
            line,
        )
        if rejection:
            target, takeoff, sortie, reserve = rejection.groups()
            required = float(sortie) + float(reserve)
            self._task_failed = True
            self._task_message = (
                f"电量不足：{target} 起飞时预计 {float(takeoff):.2f} Wh，"
                f"需要 {required:.2f} Wh（任务 {float(sortie):.2f} Wh + "
                f"储备 {float(reserve):.2f} Wh）"
            )
            self._set_task_result(
                "任务状态：被能量规划器拒绝\n" + self._task_message,
                "error",
            )
            self._set_status("电量不足", "error")
            return

        if lowered.strip().startswith("message:"):
            message = line.split(":", 1)[1].strip().strip("'\"")
            self._task_message = message
            if self._task_failed:
                self._set_task_result("任务状态：失败\n" + message, "error")
            return
        if "success: false" in lowered or "status: aborted" in lowered:
            self._task_failed = True
            detail = self._task_message or "Action 返回失败，请查看下方运行日志。"
            self._set_task_result("任务状态：失败\n" + detail, "error")
            self._set_status("任务失败", "error")
            return
        if "goal was rejected" in lowered:
            self._task_failed = True
            self._set_task_result("任务状态：Action 目标被拒绝", "error")
            self._set_status("任务被拒绝", "error")
            return
        if "success: true" in lowered or "status: succeeded" in lowered:
            self._task_succeeded = True
            self._set_task_result("任务状态：配送完成", "success")
            self._set_status("任务完成", "ready")

    def _set_task_result(self, text: str, state: str) -> None:
        self.task_result.setText(text)
        self.task_result.setProperty("state", state)
        self.task_result.style().unpolish(self.task_result)
        self.task_result.style().polish(self.task_result)

    def _append_log(self, source: str, text: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_console.appendPlainText(f"{timestamp}  [{source}]  {text}")

    def _process_changed(self, group: str, label: str, state: str) -> None:
        state_names = {
            "scheduled": "已排队",
            "running": "运行中",
            "stopping": "正在停止",
        }
        if state.startswith("finished"):
            code = state.split(":")[1]
            message = f"已退出，代码 {code}"
        elif state.startswith("error"):
            message = "启动错误"
        else:
            message = state_names.get(state, state)
        self._append_log(label, message)
        if group == "simulation" and state == "running":
            self._set_status("仿真运行中", "running")
        elif group == "task":
            if state == "running":
                self._set_task_result("任务状态：任务命令运行中", "running")
            elif state.startswith("error"):
                self._task_failed = True
                self._set_task_result("任务状态：任务命令启动失败", "error")
                self._set_status("任务启动失败", "error")
            elif state.startswith("finished"):
                code = int(state.split(":")[1])
                if self._task_cancel_requested:
                    self._set_task_result("任务状态：已停止", "warning")
                elif code != 0:
                    self._task_failed = True
                    self._set_task_result(
                        f"任务状态：任务进程异常退出（代码 {code}）",
                        "error",
                    )
                    self._set_status("任务异常退出", "error")
                elif self._task_failed:
                    self._set_status("任务失败", "error")
                elif self._task_succeeded:
                    self._set_task_result("任务状态：配送完成", "success")
                else:
                    self._set_task_result("任务状态：任务命令已结束", "success")
        self._refresh_buttons()

    def _active_count_changed(self, count: int) -> None:
        self.process_count.setText(f"{count} 个活动进程")
        if count == 0 and self.system_status.property("state") != "error":
            self._set_status("就绪", "ready")
        self._refresh_buttons()

    def _set_status(self, text: str, state: str) -> None:
        self.system_status.setText(text)
        self.system_status.setProperty("state", state)
        self.system_status.style().unpolish(self.system_status)
        self.system_status.style().polish(self.system_status)

    def closeEvent(self, event: QCloseEvent) -> None:
        self._save_settings()
        self._simulation_start_timer.stop()
        self._task_start_timer.stop()
        self.supervisor.stop_all()
        event.accept()
