#!/usr/bin/env bash
# Source this file from bash: source /home/hong/SLAM/env.sh
_slam_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="${_slam_root}/third_party/install${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${_slam_root}/third_party/install/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
source "${_slam_root}/ws/install/local_setup.bash"
unset _slam_root
