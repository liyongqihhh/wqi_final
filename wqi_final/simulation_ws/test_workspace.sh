#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/workspace_paths.bash"

install_setup="${WQI_BUILD_ROOT}/install/setup.bash"
if [[ ! -f "${install_setup}" ]]; then
  echo "External workspace is not built: ${install_setup}" >&2
  echo "Run: bash ${WQI_WORKSPACE}/build_workspace.sh" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${install_setup}"

export PYTHONDONTWRITEBYTECODE=1
export PYTEST_ADDOPTS="${PYTEST_ADDOPTS:+${PYTEST_ADDOPTS} }-p no:cacheprovider"

cd "${WQI_WORKSPACE}"
colcon --log-base "${WQI_BUILD_ROOT}/log" test \
  --base-paths "${WQI_WORKSPACE}/src" \
  --build-base "${WQI_BUILD_ROOT}/build" \
  --install-base "${WQI_BUILD_ROOT}/install" \
  --test-result-base "${WQI_BUILD_ROOT}/build" \
  "$@"

colcon --log-base "${WQI_BUILD_ROOT}/log" test-result \
  --test-result-base "${WQI_BUILD_ROOT}/build" \
  --all \
  --verbose
