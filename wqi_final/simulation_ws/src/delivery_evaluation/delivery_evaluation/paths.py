import os
from pathlib import Path
from typing import Mapping, Optional


def default_results_directory(
    environment: Optional[Mapping[str, str]] = None,
    home: Optional[Path] = None,
) -> Path:
    env = os.environ if environment is None else environment
    explicit = env.get("WQI_EXPERIMENT_RESULTS_DIR")
    if explicit:
        return Path(explicit).expanduser()

    build_root = env.get("WQI_BUILD_ROOT")
    if build_root:
        return Path(build_root).expanduser() / "experiment_results"

    home_directory = Path.home() if home is None else Path(home)
    return home_directory / ".ros" / "wqi_final" / "experiment_results"
