import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication  # noqa: E402

from simulation_ui.cargo_editor import CargoEditor  # noqa: E402
from simulation_ui.config import DeliveryItem  # noqa: E402


@pytest.fixture(scope="module")
def qt_app():
    application = QApplication.instance() or QApplication([])
    yield application


def test_item_count_creates_independent_cargo_rows(qt_app):
    editor = CargoEditor()
    editor.count_spin.setValue(3)
    items = editor.items()
    assert len(items) == 3
    assert len({item.target_id for item in items}) == 3


def test_set_items_preserves_each_target_floor_and_payload(qt_app):
    editor = CargoEditor()
    expected = [
        DeliveryItem("library", 5, 0.15),
        DeliveryItem("teaching_building", 2, 0.45),
        DeliveryItem("dormitory_4", 11, 0.25),
    ]
    editor.set_items(expected)
    assert editor.items() == expected
    assert "总载荷 0.85 kg" in editor.summary.text()


def test_ugv_mode_disables_floor_and_payload_but_keeps_targets(qt_app):
    editor = CargoEditor()
    editor.set_capabilities(True, False, False)
    assert editor.table.cellWidget(0, 0).isEnabled()
    assert not editor.table.cellWidget(0, 1).isEnabled()
    assert not editor.table.cellWidget(0, 2).isEnabled()
