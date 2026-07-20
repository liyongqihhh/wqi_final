from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDoubleSpinBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QSpinBox,
    QTableWidget,
    QVBoxLayout,
    QWidget,
)

from simulation_ui.config import (
    BUILDINGS,
    BUILDING_BY_ID,
    MAX_DELIVERY_ITEMS,
    MAX_UAV_PAYLOAD_KG,
    DeliveryItem,
)


class CargoEditor(QWidget):
    items_changed = pyqtSignal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(7)

        toolbar = QHBoxLayout()
        toolbar.setContentsMargins(0, 0, 0, 0)
        toolbar.addWidget(QLabel("件数"))
        self.count_spin = QSpinBox()
        self.count_spin.setRange(1, MAX_DELIVERY_ITEMS)
        self.count_spin.setSuffix(" 件")
        toolbar.addWidget(self.count_spin)
        toolbar.addStretch(1)
        self.summary = QLabel()
        self.summary.setObjectName("valueHint")
        toolbar.addWidget(self.summary)
        layout.addLayout(toolbar)

        self.table = QTableWidget(0, 3)
        self.table.setObjectName("cargoTable")
        self.table.setHorizontalHeaderLabels(("配送位置", "楼层", "载荷"))
        self.table.verticalHeader().setVisible(True)
        self.table.verticalHeader().setDefaultSectionSize(38)
        self.table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.Stretch
        )
        self.table.horizontalHeader().setSectionResizeMode(
            1, QHeaderView.ResizeToContents
        )
        self.table.horizontalHeader().setSectionResizeMode(
            2, QHeaderView.ResizeToContents
        )
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setMinimumHeight(218)
        layout.addWidget(self.table)

        self._supports_target = True
        self._supports_floor = True
        self._supports_payload = True
        self.count_spin.valueChanged.connect(self._count_changed)
        self.set_items([self._default_item(0)])

    @staticmethod
    def _default_item(row: int) -> DeliveryItem:
        building = BUILDINGS[row % len(BUILDINGS)]
        return DeliveryItem(
            building.target_id,
            building.default_floor,
            building.default_payload_kg,
        )

    def _create_row(self, row: int, item: DeliveryItem) -> None:
        building = BUILDING_BY_ID[item.target_id]
        target = QComboBox()
        for option in BUILDINGS:
            target.addItem(option.label, option.target_id)
        target.setCurrentIndex(target.findData(item.target_id))

        floor = QSpinBox()
        floor.setRange(1, building.maximum_floor)
        floor.setSuffix(" 层")
        floor.setValue(min(max(int(item.floor), 1), building.maximum_floor))

        payload = QDoubleSpinBox()
        payload.setRange(0.0, MAX_UAV_PAYLOAD_KG)
        payload.setSingleStep(0.05)
        payload.setDecimals(2)
        payload.setSuffix(" kg")
        payload.setValue(float(item.payload_kg))

        self.table.setCellWidget(row, 0, target)
        self.table.setCellWidget(row, 1, floor)
        self.table.setCellWidget(row, 2, payload)
        self.table.setVerticalHeaderItem(
            row, self._header_item(str(row + 1))
        )
        target.currentIndexChanged.connect(
            lambda _index, current=target, floor_input=floor,
            payload_input=payload: self._target_changed(
                current, floor_input, payload_input
            )
        )
        floor.valueChanged.connect(lambda _value: self._emit_changed())
        payload.valueChanged.connect(lambda _value: self._emit_changed())
        self._apply_row_capabilities(target, floor, payload)
        self._update_floor_tooltip(target, floor)

    @staticmethod
    def _header_item(text: str):
        from PyQt5.QtWidgets import QTableWidgetItem

        return QTableWidgetItem(text)

    def _target_changed(self, target, floor, payload) -> None:
        building = BUILDING_BY_ID[str(target.currentData())]
        floor.setRange(1, building.maximum_floor)
        floor.setValue(building.default_floor)
        payload.setValue(building.default_payload_kg)
        self._update_floor_tooltip(target, floor)
        self._emit_changed()

    @staticmethod
    def _update_floor_tooltip(target, floor) -> None:
        building = BUILDING_BY_ID[str(target.currentData())]
        altitude = building.altitude_for_floor(floor.value())
        floor.setToolTip(
            f"{building.label}最高 {building.maximum_floor} 层；"
            f"当前悬停高度 {altitude:.1f} m"
        )

    def _count_changed(self, count: int) -> None:
        while self.table.rowCount() < count:
            row = self.table.rowCount()
            self.table.insertRow(row)
            self._create_row(row, self._default_item(row))
        while self.table.rowCount() > count:
            self.table.removeRow(self.table.rowCount() - 1)
        self._emit_changed()

    def _emit_changed(self) -> None:
        for row in range(self.table.rowCount()):
            target = self.table.cellWidget(row, 0)
            floor = self.table.cellWidget(row, 1)
            self._update_floor_tooltip(target, floor)
        items = self.items()
        total_payload = sum(item.payload_kg for item in items)
        unique_targets = len({item.target_id for item in items})
        self.summary.setText(
            f"{unique_targets} 个位置 / 总载荷 {total_payload:.2f} kg"
        )
        self.items_changed.emit()

    def items(self) -> list[DeliveryItem]:
        result = []
        for row in range(self.table.rowCount()):
            target = self.table.cellWidget(row, 0)
            floor = self.table.cellWidget(row, 1)
            payload = self.table.cellWidget(row, 2)
            result.append(DeliveryItem(
                str(target.currentData()),
                int(floor.value()),
                float(payload.value()),
            ))
        return result

    def set_items(self, items: list[DeliveryItem]) -> None:
        values = list(items)[:MAX_DELIVERY_ITEMS]
        if not values:
            values = [self._default_item(0)]
        self.count_spin.blockSignals(True)
        self.table.setRowCount(0)
        for row, item in enumerate(values):
            self.table.insertRow(row)
            self._create_row(row, item)
        self.count_spin.setValue(len(values))
        self.count_spin.blockSignals(False)
        self._emit_changed()

    def _apply_row_capabilities(self, target, floor, payload) -> None:
        target.setEnabled(self._supports_target)
        floor.setEnabled(self._supports_target and self._supports_floor)
        payload.setEnabled(self._supports_target and self._supports_payload)

    def set_capabilities(
        self,
        supports_target: bool,
        supports_floor: bool,
        supports_payload: bool,
    ) -> None:
        self._supports_target = bool(supports_target)
        self._supports_floor = bool(supports_floor)
        self._supports_payload = bool(supports_payload)
        self.count_spin.setEnabled(self._supports_target)
        for row in range(self.table.rowCount()):
            self._apply_row_capabilities(
                self.table.cellWidget(row, 0),
                self.table.cellWidget(row, 1),
                self.table.cellWidget(row, 2),
            )
