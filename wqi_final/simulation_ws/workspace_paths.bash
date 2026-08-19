#!/usr/bin/env bash

# Shared path discovery for out-of-tree ROS 2 builds.
_wqi_paths_script_dir="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1
  pwd
)"

export WQI_WORKSPACE="${WQI_WORKSPACE:-${_wqi_paths_script_dir}}"

if _wqi_git_root="$(
  git -C "${WQI_WORKSPACE}" rev-parse --show-toplevel 2>/dev/null
)"; then
  export WQI_REPOSITORY_ROOT="${WQI_REPOSITORY_ROOT:-${_wqi_git_root}}"
else
  export WQI_REPOSITORY_ROOT="${WQI_REPOSITORY_ROOT:-$(
    cd -- "${WQI_WORKSPACE}/../.." >/dev/null 2>&1
    pwd
  )}"
fi

_wqi_default_build_root="$(dirname -- "${WQI_REPOSITORY_ROOT}")/$(
  basename -- "${WQI_REPOSITORY_ROOT}"
)_artifacts"
export WQI_BUILD_ROOT="${WQI_BUILD_ROOT:-${_wqi_default_build_root}}"

unset _wqi_paths_script_dir _wqi_git_root _wqi_default_build_root
