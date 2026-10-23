#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
patch_file="${script_dir}/../patches/gazebo_contact_imu.patch"
antialias_patch="${script_dir}/../patches/gazebo_contact_imu_antialias.patch"
px4_dir="${1:?Usage: bash refresh_gazebo_imu.sh PX4_SOURCE [--apply-only]}"
gazebo_dir="${px4_dir}/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
if git -C "${gazebo_dir}" apply --reverse --check "${antialias_patch}" 2>/dev/null; then
  echo "Gazebo contact-aware anti-aliased IMU patch already applied."
else
  if ! git -C "${gazebo_dir}" apply --reverse --check "${patch_file}" 2>/dev/null; then
    git -C "${gazebo_dir}" apply --check "${patch_file}"
    git -C "${gazebo_dir}" apply "${patch_file}"
  fi
  git -C "${gazebo_dir}" apply --check "${antialias_patch}"
  git -C "${gazebo_dir}" apply "${antialias_patch}"
fi
if [[ "${2:-}" != "--apply-only" ]]; then
  cmake --build "${px4_dir}/build/px4_sitl_default/build_gazebo-classic" \
    --target gazebo_imu_plugin --parallel "${PX4_BUILD_JOBS:-2}"
fi
