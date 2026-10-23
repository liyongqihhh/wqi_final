#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This file must be sourced: source ./setup_workspace.bash" >&2
  exit 2
fi

_wqi_setup_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${_wqi_setup_dir}/workspace_paths.bash"
export PYTHONDONTWRITEBYTECODE=1

_wqi_install_setup="${WQI_BUILD_ROOT}/install/setup.bash"
if [[ ! -f "${_wqi_install_setup}" ]]; then
  echo "External workspace is not built: ${_wqi_install_setup}" >&2
  echo "Run: bash ${WQI_WORKSPACE}/build_workspace.sh" >&2
  unset _wqi_setup_dir _wqi_install_setup
  return 1
fi

source /opt/ros/humble/setup.bash
_wqi_px4_setup="${WQI_BUILD_ROOT}/px4_ros_ws/install/setup.bash"
if [[ -f "${_wqi_px4_setup}" ]]; then
  source "${_wqi_px4_setup}"
  export PX4_AUTOPILOT_DIR="${WQI_BUILD_ROOT}/px4/PX4-Autopilot"
  for _wqi_venv in virtualenv venv; do
    for _wqi_site in "${WQI_BUILD_ROOT}/px4/${_wqi_venv}"/lib/python*/site-packages; do
      if [[ -d "${_wqi_site}/pymavlink" ]]; then
        export PX4_PYMAVLINK_SITE="${_wqi_site}"
        break 2
      fi
    done
  done
fi
source "${_wqi_install_setup}"
echo "Loaded wqi_final from ${WQI_BUILD_ROOT}/install"

unset _wqi_setup_dir _wqi_install_setup _wqi_px4_setup _wqi_venv _wqi_site
