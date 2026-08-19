#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This file must be sourced: source ./setup_workspace.bash" >&2
  exit 2
fi

_wqi_setup_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${_wqi_setup_dir}/workspace_paths.bash"

_wqi_install_setup="${WQI_BUILD_ROOT}/install/setup.bash"
if [[ ! -f "${_wqi_install_setup}" ]]; then
  echo "External workspace is not built: ${_wqi_install_setup}" >&2
  echo "Run: bash ${WQI_WORKSPACE}/build_workspace.sh" >&2
  unset _wqi_setup_dir _wqi_install_setup
  return 1
fi

source /opt/ros/humble/setup.bash
source "${_wqi_install_setup}"
echo "Loaded wqi_final from ${WQI_BUILD_ROOT}/install"

unset _wqi_setup_dir _wqi_install_setup
