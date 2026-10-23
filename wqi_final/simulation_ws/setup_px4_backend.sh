#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/workspace_paths.bash"

px4_root="${WQI_BUILD_ROOT}/px4"
px4_dir="${px4_root}/PX4-Autopilot"
ros_ws="${WQI_BUILD_ROOT}/px4_ros_ws"
ros_src="${ros_ws}/src"
jobs="${PX4_BUILD_JOBS:-2}"

if [[ "${1:-}" == "--install-system-deps" ]]; then
  sudo apt-get update
  sudo apt-get install -y \
    build-essential cmake curl git ninja-build python3-dev python3-pip \
    python3-venv protobuf-compiler libeigen3-dev libopencv-dev \
    libgazebo11-dev ros-humble-gazebo-ros-pkgs
fi

mkdir -p "${px4_root}" "${ros_src}"

git -C "${WQI_REPOSITORY_ROOT}" submodule update --init --recursive -- \
  wqi_final/third_party/PX4-Autopilot \
  wqi_final/third_party/px4_msgs \
  wqi_final/third_party/Micro-XRCE-DDS-Agent
python3 "${script_dir}/prepare_px4_sources.py" \
  --source-root "${script_dir}/../third_party" --build-root "${WQI_BUILD_ROOT}"

venv_dir="${px4_root}/venv"
if [[ -f "${px4_root}/virtualenv/bin/activate" ]]; then
  venv_dir="${px4_root}/virtualenv"
elif ! python3 -m venv "${venv_dir}"; then
  echo "python3-venv is unavailable; using user-installed virtualenv."
  if ! python3 -m pip --version >/dev/null 2>&1; then
    get_pip="${px4_root}/get-pip.py"
    curl -fsSL https://bootstrap.pypa.io/get-pip.py -o "${get_pip}"
    python3 "${get_pip}" --user
  fi
  python3 -m pip install --user virtualenv
  venv_dir="${px4_root}/virtualenv"
  python3 -m virtualenv "${venv_dir}"
fi
source "${venv_dir}/bin/activate"
python -m pip install --upgrade pip wheel
python -m pip install -r "${px4_dir}/Tools/setup/requirements.txt"
python -m pip install pymavlink==2.4.49

gazebo_dir="${px4_dir}/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
bash "${script_dir}/src/uav_px4_bringup/scripts/refresh_gazebo_imu.sh" \
  "${px4_dir}" --apply-only
battery_patch="${script_dir}/src/uav_px4_bringup/patches/px4_external_battery_time.patch"
if git -C "${px4_dir}" apply --reverse --check "${battery_patch}" 2>/dev/null; then
  echo "PX4 external battery flight-time compatibility patch already applied."
else
  git -C "${px4_dir}" apply --check "${battery_patch}"
  git -C "${px4_dir}" apply "${battery_patch}"
fi
make -C "${px4_dir}" -j"${jobs}" px4_sitl_default
cmake -S "${gazebo_dir}" \
  -B "${px4_dir}/build/px4_sitl_default/build_gazebo-classic" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PROJECT_INCLUDE="${script_dir}/px4_gazebo_options.cmake" \
  -DBUILD_GSTREAMER_PLUGIN=OFF \
  -DSEND_ODOMETRY_DATA=ON \
  -DGENERATE_ROS_MODELS=ON
export DONT_RUN=1
make -C "${px4_dir}" -j"${jobs}" px4_sitl_default gazebo-classic_iris
unset DONT_RUN

iris_template="${gazebo_dir}/models/iris/iris.sdf.jinja"
iris_sdf="${gazebo_dir}/models/iris/iris.sdf"
ROS_VERSION=2 python "${gazebo_dir}/scripts/jinja_gen.py" \
  "${iris_template}" "${gazebo_dir}" \
  --generate_ros_models true --output-file "${iris_sdf}"
mkdir -p "${px4_dir}/build/px4_sitl_default/rootfs"

deactivate
set +u
source /opt/ros/humble/setup.bash
set -u
colcon --log-base "${ros_ws}/log" build \
  --base-paths "${ros_src}" \
  --build-base "${ros_ws}/build" \
  --install-base "${ros_ws}/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "PX4 backend installed outside the source tree:"
echo "  PX4: ${px4_dir}"
echo "  ROS: ${ros_ws}/install"
echo "Now run: bash ./build_workspace.sh && source ./setup_workspace.bash"
