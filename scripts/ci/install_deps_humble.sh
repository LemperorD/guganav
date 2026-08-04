#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-humble}"
RUN_APT="${RUN_APT:-1}"
RUN_ROSDEP="${RUN_ROSDEP:-1}"
USE_ROSDEPC="${USE_ROSDEPC:-0}"
INSTALL_ACADOS="${INSTALL_ACADOS:-1}"
INSTALL_ACADOS_PYTHON="${INSTALL_ACADOS_PYTHON:-1}"
INSTALL_SMALL_GICP="${INSTALL_SMALL_GICP:-1}"
ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-${HOME}/tools/acados}"
ACADOS_REPO="${ACADOS_REPO:-https://github.com/acados/acados.git}"
ACADOS_CLONE_DEPTH="${ACADOS_CLONE_DEPTH:-1}"
ACADOS_JOBS="${ACADOS_JOBS:-$(nproc)}"
SMALL_GICP_SOURCE_DIR="${SMALL_GICP_SOURCE_DIR:-${HOME}/tools/small_gicp}"
SMALL_GICP_REPO="${SMALL_GICP_REPO:-https://github.com/koide3/small_gicp.git}"
SMALL_GICP_CLONE_DEPTH="${SMALL_GICP_CLONE_DEPTH:-1}"
SMALL_GICP_JOBS="${SMALL_GICP_JOBS:-$(nproc)}"
ROSDEP_SKIP_KEYS="${ROSDEP_SKIP_KEYS:-pb2025_sentry_nav pb_teleop_twist_joy}"
DEBIAN_FRONTEND="${DEBIAN_FRONTEND:-noninteractive}"

export DEBIAN_FRONTEND

run_root() {
  if [ "${EUID}" -eq 0 ]; then
    "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "$@"
  else
    echo "This command needs root privileges, but sudo is not installed: $*" >&2
    exit 1
  fi
}

source_ros() {
  local setup_file="/opt/ros/${ROS_DISTRO}/setup.bash"
  if [ ! -f "${setup_file}" ]; then
    echo "ROS setup file not found: ${setup_file}" >&2
    exit 1
  fi
  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  set -u
}

install_apt_dependencies() {
  if [ "${RUN_APT}" != "1" ]; then
    echo "Skipping apt dependencies because RUN_APT=${RUN_APT}"
    return
  fi

  local ros_prefix="ros-${ROS_DISTRO}"
  local packages=(
    ca-certificates
    curl
    git
    gnupg
    lsb-release
    wget
    build-essential
    cmake
    gcc
    g++
    make
    pkg-config
    python3-colcon-common-extensions
    python3-dev
    python3-pip
    python3-rosdep
    python3-setuptools
    python3-venv
    python3-wheel
    libboost-dev
    libeigen3-dev
    libfmt-dev
    libgoogle-glog-dev
    nlohmann-json3-dev
    libpcl-dev
    libtbb-dev
    libunwind-dev
    ignition-fortress
    libignition-cmake2-dev
    libignition-gazebo6-dev
    libignition-msgs8-dev
    libignition-transport11-dev
    "${ros_prefix}-behaviortree-cpp-v3"
    "${ros_prefix}-camera-info-manager"
    "${ros_prefix}-generate-parameter-library"
    "${ros_prefix}-image-transport"
    "${ros_prefix}-laser-geometry"
    "${ros_prefix}-navigation2"
    "${ros_prefix}-nav2-bringup"
    "${ros_prefix}-pcl-conversions"
    "${ros_prefix}-pcl-ros"
    "${ros_prefix}-ros-gz"
    "${ros_prefix}-rviz2"
    "${ros_prefix}-slam-toolbox"
  )

  if [ -n "${APT_EXTRA_PACKAGES:-}" ]; then
    # shellcheck disable=SC2206
    packages+=(${APT_EXTRA_PACKAGES})
  fi

  run_root apt-get update
  run_root apt-get install -y "${packages[@]}"
}

install_rosdep_dependencies() {
  if [ "${RUN_ROSDEP}" != "1" ]; then
    echo "Skipping rosdep because RUN_ROSDEP=${RUN_ROSDEP}"
    return
  fi

  source_ros
  cd "${REPO_ROOT}"

  if [ "${USE_ROSDEPC}" = "1" ]; then
    python3 -m pip install rosdepc -i https://pypi.tuna.tsinghua.edu.cn/simple
    rosdepc init || true
    rosdepc update
    local rosdepc_cmd=(
      rosdepc install
      --from-paths src
      --ignore-src
      --rosdistro "${ROS_DISTRO}"
      -r -y
    )
    if [ -n "${ROSDEP_SKIP_KEYS}" ]; then
      rosdepc_cmd+=(--skip-keys "${ROSDEP_SKIP_KEYS}")
    fi
    "${rosdepc_cmd[@]}"
  else
    rosdep init || true
    rosdep update --rosdistro "${ROS_DISTRO}"
    local rosdep_cmd=(
      rosdep install
      --from-paths src
      --ignore-src
      --rosdistro "${ROS_DISTRO}"
      -r -y
    )
    if [ -n "${ROSDEP_SKIP_KEYS}" ]; then
      rosdep_cmd+=(--skip-keys "${ROSDEP_SKIP_KEYS}")
    fi
    "${rosdep_cmd[@]}"
  fi
}

install_acados() {
  if [ "${INSTALL_ACADOS}" != "1" ]; then
    echo "Skipping acados because INSTALL_ACADOS=${INSTALL_ACADOS}"
    return
  fi

  export ACADOS_SOURCE_DIR
  export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH:-}"

  mkdir -p "$(dirname "${ACADOS_SOURCE_DIR}")"

  if [ ! -d "${ACADOS_SOURCE_DIR}/.git" ]; then
    git clone --depth "${ACADOS_CLONE_DEPTH}" "${ACADOS_REPO}" "${ACADOS_SOURCE_DIR}"
  fi

  cd "${ACADOS_SOURCE_DIR}"
  git submodule update --recursive --init --depth "${ACADOS_CLONE_DEPTH}"

  mkdir -p build
  cd build
  cmake ..
  make -j"${ACADOS_JOBS}"
  make install

  if [ "${INSTALL_ACADOS_PYTHON}" = "1" ]; then
    python3 -m pip install -e "${ACADOS_SOURCE_DIR}/interfaces/acados_template"
  fi

  if [ -n "${GITHUB_ENV:-}" ]; then
    {
      echo "ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"
      echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}"
    } >> "${GITHUB_ENV}"
  fi
}

install_small_gicp() {
  if [ "${INSTALL_SMALL_GICP}" != "1" ]; then
    echo "Skipping small_gicp because INSTALL_SMALL_GICP=${INSTALL_SMALL_GICP}"
    return
  fi

  mkdir -p "$(dirname "${SMALL_GICP_SOURCE_DIR}")"

  if [ ! -d "${SMALL_GICP_SOURCE_DIR}/.git" ]; then
    git clone --depth "${SMALL_GICP_CLONE_DEPTH}" "${SMALL_GICP_REPO}" "${SMALL_GICP_SOURCE_DIR}"
  fi

  cd "${SMALL_GICP_SOURCE_DIR}"
  mkdir -p build
  cd build
  cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_BENCHMARKS=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF
  make -j"${SMALL_GICP_JOBS}"
  run_root make install
}

install_apt_dependencies
install_rosdep_dependencies
install_small_gicp
install_acados

cat <<EOF
Dependency installation finished.

Generate the MPC solver before the first workspace build:
  bash ${REPO_ROOT}/scripts/ci/generate_acados_mpc.sh

Use this before building:
  source ${REPO_ROOT}/scripts/ci/env_humble.sh
EOF
