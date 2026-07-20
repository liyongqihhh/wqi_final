from pathlib import Path
import sys

from PyQt5.QtWidgets import QApplication

from simulation_ui.main_window import SimulationDashboard


def _stylesheet() -> str:
    source_path = Path(__file__).parents[1] / "resource" / "dashboard.qss"
    if source_path.exists():
        return source_path.read_text(encoding="utf-8")
    try:
        from ament_index_python.packages import get_package_share_directory

        installed_path = (
            Path(get_package_share_directory("simulation_ui"))
            / "resource"
            / "dashboard.qss"
        )
        return installed_path.read_text(encoding="utf-8")
    except (ImportError, FileNotFoundError, LookupError):
        return ""


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("校园物流配送仿真控制台")
    app.setOrganizationName("wqi_final")
    app.setStyle("Fusion")
    app.setStyleSheet(_stylesheet())
    window = SimulationDashboard()
    window.show()
    raise SystemExit(app.exec_())


if __name__ == "__main__":
    main()
