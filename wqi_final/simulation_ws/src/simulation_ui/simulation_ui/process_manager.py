from dataclasses import dataclass
import os
import signal

from PyQt5.QtCore import QObject, QProcess, QTimer, pyqtSignal

from simulation_ui.config import CommandBuilder, CommandSpec


@dataclass
class _ManagedProcess:
    key: str
    group: str
    spec: CommandSpec
    process: QProcess


class ProcessSupervisor(QObject):
    output_received = pyqtSignal(str, str)
    process_changed = pyqtSignal(str, str, str)
    active_count_changed = pyqtSignal(int)

    def __init__(self, command_builder: CommandBuilder, parent=None) -> None:
        super().__init__(parent)
        self.command_builder = command_builder
        self._processes: dict[str, _ManagedProcess] = {}
        self._timers: dict[str, list[QTimer]] = {}
        self._next_id = 0

    def launch_many(self, specs: list[CommandSpec], group: str) -> None:
        self.stop_group(group)
        for spec in specs:
            if spec.delay_seconds <= 0.0:
                self._launch(spec, group)
                continue
            timer = QTimer(self)
            timer.setSingleShot(True)
            timer.timeout.connect(
                lambda item=spec, name=group: self._launch(item, name)
            )
            timer.timeout.connect(
                lambda name=group, current=timer: self._remove_timer(
                    name, current
                )
            )
            self._timers.setdefault(group, []).append(timer)
            timer.start(round(spec.delay_seconds * 1000.0))
            self.process_changed.emit(group, spec.label, "scheduled")

    def launch_one(self, spec: CommandSpec, group: str) -> None:
        self.launch_many([spec], group)

    def _remove_timer(self, group: str, timer: QTimer) -> None:
        timers = self._timers.get(group, [])
        if timer in timers:
            timers.remove(timer)
        if not timers:
            self._timers.pop(group, None)
        timer.deleteLater()

    def _launch(self, spec: CommandSpec, group: str) -> None:
        self._next_id += 1
        key = f"{group}-{self._next_id}"
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.setProgram("/usr/bin/setsid")
        process.setArguments([
            "/bin/bash",
            "-lc",
            self.command_builder.shell_command(spec.command),
        ])
        entry = _ManagedProcess(key, group, spec, process)
        self._processes[key] = entry
        process.readyReadStandardOutput.connect(
            lambda current=key: self._read_output(current)
        )
        process.started.connect(
            lambda current=key: self._started(current)
        )
        process.errorOccurred.connect(
            lambda error, current=key: self._error(current, int(error))
        )
        process.finished.connect(
            lambda code, status, current=key: self._finished(
                current, int(code), int(status)
            )
        )
        process.start()

    def _started(self, key: str) -> None:
        entry = self._processes.get(key)
        if entry is None:
            return
        self.process_changed.emit(entry.group, entry.spec.label, "running")
        self.active_count_changed.emit(self.active_count())

    def _read_output(self, key: str) -> None:
        entry = self._processes.get(key)
        if entry is None:
            return
        data = bytes(entry.process.readAllStandardOutput()).decode(
            "utf-8", errors="replace"
        )
        if data:
            self.output_received.emit(entry.spec.label, data)

    def _error(self, key: str, error: int) -> None:
        entry = self._processes.get(key)
        if entry is not None:
            self.process_changed.emit(
                entry.group, entry.spec.label, f"error:{error}"
            )

    def _finished(self, key: str, code: int, status: int) -> None:
        entry = self._processes.pop(key, None)
        if entry is None:
            return
        remaining = bytes(entry.process.readAllStandardOutput()).decode(
            "utf-8", errors="replace"
        )
        if remaining:
            self.output_received.emit(entry.spec.label, remaining)
        self.process_changed.emit(
            entry.group,
            entry.spec.label,
            f"finished:{code}:{status}",
        )
        entry.process.deleteLater()
        self.active_count_changed.emit(self.active_count())

    def _stop_process(self, entry: _ManagedProcess) -> None:
        process = entry.process
        if process.state() == QProcess.NotRunning:
            return
        pid = int(process.processId())
        try:
            if pid > 0:
                os.killpg(pid, signal.SIGINT)
            else:
                process.terminate()
        except (ProcessLookupError, PermissionError):
            process.terminate()

        def terminate_if_running() -> None:
            if process.state() != QProcess.NotRunning:
                try:
                    current_pid = int(process.processId())
                    if current_pid > 0:
                        os.killpg(current_pid, signal.SIGTERM)
                    else:
                        process.terminate()
                except (ProcessLookupError, PermissionError):
                    process.terminate()

        def kill_if_running() -> None:
            if process.state() != QProcess.NotRunning:
                try:
                    current_pid = int(process.processId())
                    if current_pid > 0:
                        os.killpg(current_pid, signal.SIGKILL)
                    else:
                        process.kill()
                except (ProcessLookupError, PermissionError):
                    process.kill()

        QTimer.singleShot(2500, terminate_if_running)
        QTimer.singleShot(5000, kill_if_running)

    def stop_group(self, group: str) -> None:
        for timer in self._timers.pop(group, []):
            timer.stop()
            timer.deleteLater()
        for entry in list(self._processes.values()):
            if entry.group == group:
                self.process_changed.emit(
                    entry.group, entry.spec.label, "stopping"
                )
                self._stop_process(entry)

    def stop_all(self) -> None:
        groups = set(self._timers)
        groups.update(entry.group for entry in self._processes.values())
        for group in groups:
            self.stop_group(group)

    def active_count(self, group: str | None = None) -> int:
        return sum(
            entry.process.state() != QProcess.NotRunning
            for entry in self._processes.values()
            if group is None or entry.group == group
        )

    def has_pending(self, group: str) -> bool:
        return bool(self._timers.get(group))
