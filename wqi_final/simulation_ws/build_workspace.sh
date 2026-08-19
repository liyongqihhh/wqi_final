#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/workspace_paths.bash"
source /opt/ros/humble/setup.bash

mkdir -p \
  "${WQI_BUILD_ROOT}/build" \
  "${WQI_BUILD_ROOT}/install" \
  "${WQI_BUILD_ROOT}/log"

echo "Workspace: ${WQI_WORKSPACE}"
echo "Artifacts: ${WQI_BUILD_ROOT}"

cd "${WQI_WORKSPACE}"
colcon --log-base "${WQI_BUILD_ROOT}/log" build \
  --base-paths "${WQI_WORKSPACE}/src" \
  --build-base "${WQI_BUILD_ROOT}/build" \
  --install-base "${WQI_BUILD_ROOT}/install" \
  --symlink-install \
  "$@"

echo
echo "Build completed. Load the workspace with:"
echo "  source ${WQI_WORKSPACE}/setup_workspace.bash"
