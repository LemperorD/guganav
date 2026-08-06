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
INSTALL_HIK_MVS="${INSTALL_HIK_MVS:-0}"
ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-${HOME}/tools/acados}"
ACADOS_REPO="${ACADOS_REPO:-https://github.com/acados/acados.git}"
# Current upstream main commit used as the known-good CI baseline.
ACADOS_REF="${ACADOS_REF:-5874c96bee93935cf0084458db1d135597a0c568}"
ACADOS_CLONE_DEPTH="${ACADOS_CLONE_DEPTH:-1}"
ACADOS_JOBS="${ACADOS_JOBS:-$(nproc)}"
SMALL_GICP_SOURCE_DIR="${SMALL_GICP_SOURCE_DIR:-${HOME}/tools/small_gicp}"
SMALL_GICP_REPO="${SMALL_GICP_REPO:-https://github.com/koide3/small_gicp.git}"
# Current upstream master commit used as the known-good CI baseline.
SMALL_GICP_REF="${SMALL_GICP_REF:-8a2d3734f699c042db74ae61d295b0b928163526}"
SMALL_GICP_CLONE_DEPTH="${SMALL_GICP_CLONE_DEPTH:-1}"
SMALL_GICP_JOBS="${SMALL_GICP_JOBS:-$(nproc)}"
SMALL_GICP_INSTALL_PREFIX="${SMALL_GICP_INSTALL_PREFIX:-/usr/local}"
XMACRO_VERSION="${XMACRO_VERSION:-1.2.1}"
HIK_MVS_ROOT="${HIK_MVS_ROOT:-/opt/MVS}"
HIK_MVS_SOURCE_DIR="${HIK_MVS_SOURCE_DIR:-}"
ROSDEP_SKIP_KEYS="${ROSDEP_SKIP_KEYS:-}"
DEBIAN_FRONTEND="${DEBIAN_FRONTEND:-noninteractive}"

export DEBIAN_FRONTEND

write_github_env() {
  local name="$1"
  local value="$2"
  if [ -n "${GITHUB_ENV:-}" ] && ! grep -Fqx "${name}=${value}" "${GITHUB_ENV}" 2>/dev/null; then
    printf '%s=%s\n' "${name}" "${value}" >> "${GITHUB_ENV}"
  fi
}

if [ "${ROS_DISTRO}" != "humble" ]; then
  echo "This installer targets ROS Humble; got ROS_DISTRO=${ROS_DISTRO}" >&2
  exit 1
fi

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

ensure_git_ref() {
  local repo="$1"
  local ref="$2"
  local source_dir="$3"
  local clone_depth="$4"
  local current_remote
  local normalized_remote
  local normalized_repo

  mkdir -p "$(dirname "${source_dir}")"

  if [ -e "${source_dir}" ] && [ ! -d "${source_dir}/.git" ]; then
    echo "Dependency directory exists but is not a git repository: ${source_dir}" >&2
    exit 1
  fi

  if [ ! -d "${source_dir}/.git" ]; then
    git clone --depth "${clone_depth}" "${repo}" "${source_dir}"
  fi

  cd "${source_dir}"
  current_remote="$(git remote get-url origin)"
  normalized_remote="${current_remote%.git}"
  normalized_repo="${repo%.git}"
  if [ "${normalized_remote}" != "${normalized_repo}" ]; then
    echo "Dependency repository mismatch at ${source_dir}: ${current_remote}" >&2
    exit 1
  fi

  if [ -n "$(git status --porcelain)" ]; then
    echo "Dependency repository has local changes; refusing to replace it: ${source_dir}" >&2
    exit 1
  fi

  if ! git cat-file -e "${ref}^{commit}" 2>/dev/null; then
    git fetch --depth "${clone_depth}" origin "${ref}"
  fi
  git checkout --detach "${ref}"
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
    "${ros_prefix}-pangolin"
    "${ros_prefix}-ros-gz"
    "${ros_prefix}-rviz2"
    "${ros_prefix}-slam-toolbox"
  )

  if [ -n "${APT_EXTRA_PACKAGES:-}" ]; then
    # shellcheck disable=SC2206
    packages+=(${APT_EXTRA_PACKAGES})
  fi

  run_root apt-get update
  run_root apt-get install -y software-properties-common
  run_root add-apt-repository -y universe
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
    python3 -m pip install rosdepc==1.1.0 -i https://pypi.tuna.tsinghua.edu.cn/simple
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
      run_root rosdepc init
    fi
    rosdepc update
    local rosdepc_cmd=(
      rosdepc install
      --from-paths src
      --ignore-src
      --rosdistro "${ROS_DISTRO}"
      -y
    )
    if [ -n "${ROSDEP_SKIP_KEYS}" ]; then
      rosdepc_cmd+=(--skip-keys "${ROSDEP_SKIP_KEYS}")
    fi
    "${rosdepc_cmd[@]}"
  else
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
      run_root rosdep init
    fi
    rosdep update --rosdistro "${ROS_DISTRO}"
    local rosdep_cmd=(
      rosdep install
      --from-paths src
      --ignore-src
      --rosdistro "${ROS_DISTRO}"
      -y
    )
    if [ -n "${ROSDEP_SKIP_KEYS}" ]; then
      rosdep_cmd+=(--skip-keys "${ROSDEP_SKIP_KEYS}")
    fi
    "${rosdep_cmd[@]}"
  fi
}

install_python_dependencies() {
  python3 -m pip install "xmacro==${XMACRO_VERSION}"
}

install_acados() {
  if [ "${INSTALL_ACADOS}" != "1" ]; then
    echo "Skipping acados because INSTALL_ACADOS=${INSTALL_ACADOS}"
    return
  fi

  export ACADOS_SOURCE_DIR
  export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH:-}"

  mkdir -p "$(dirname "${ACADOS_SOURCE_DIR}")"

  ensure_git_ref "${ACADOS_REPO}" "${ACADOS_REF}" "${ACADOS_SOURCE_DIR}" "${ACADOS_CLONE_DEPTH}"
  git submodule update --recursive --init --depth "${ACADOS_CLONE_DEPTH}"

  local build_dir="${ACADOS_BUILD_DIR:-${ACADOS_SOURCE_DIR}/../guganav-build/acados-${ACADOS_REF:0:12}}"
  local stamp_file="${build_dir}/.guganav-installed-ref"
  if [ -f "${stamp_file}" ] &&
     [ "$(<"${stamp_file}")" = "${ACADOS_REF}" ] &&
     [ -f "${ACADOS_SOURCE_DIR}/lib/libacados.so" ]; then
    echo "acados ${ACADOS_REF} is already installed"
  else
    cmake -S "${ACADOS_SOURCE_DIR}" -B "${build_dir}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="${ACADOS_SOURCE_DIR}"
    cmake --build "${build_dir}" --parallel "${ACADOS_JOBS}"
    cmake --install "${build_dir}"
    printf '%s\n' "${ACADOS_REF}" > "${stamp_file}"
  fi

  if [ "${INSTALL_ACADOS_PYTHON}" = "1" ]; then
    local acados_python_dir="${ACADOS_SOURCE_DIR}/interfaces/acados_template"
    if ! python3 -c \
      'import pathlib, sys, acados_template; module = pathlib.Path(acados_template.__file__).resolve(); root = pathlib.Path(sys.argv[1]).resolve(); sys.exit(0 if module.is_relative_to(root) else 1)' \
      "${acados_python_dir}" >/dev/null 2>&1; then
      python3 -m pip install -e "${ACADOS_SOURCE_DIR}/interfaces/acados_template"
    fi
    python3 -c "from acados_template import get_tera; get_tera()"
  fi

  if [ -n "${GITHUB_ENV:-}" ]; then
    write_github_env ACADOS_SOURCE_DIR "${ACADOS_SOURCE_DIR}"
    write_github_env LD_LIBRARY_PATH "${LD_LIBRARY_PATH:-}"
    write_github_env SMALL_GICP_INSTALL_PREFIX "${SMALL_GICP_INSTALL_PREFIX}"
  fi
}

install_small_gicp() {
  if [ "${INSTALL_SMALL_GICP}" != "1" ]; then
    echo "Skipping small_gicp because INSTALL_SMALL_GICP=${INSTALL_SMALL_GICP}"
    return
  fi

  mkdir -p "$(dirname "${SMALL_GICP_SOURCE_DIR}")"

  ensure_git_ref "${SMALL_GICP_REPO}" "${SMALL_GICP_REF}" \
    "${SMALL_GICP_SOURCE_DIR}" "${SMALL_GICP_CLONE_DEPTH}"

  local build_dir="${SMALL_GICP_BUILD_DIR:-${SMALL_GICP_SOURCE_DIR}/../guganav-build/small_gicp-${SMALL_GICP_REF:0:12}}"
  local stamp_file="${build_dir}/.guganav-installed-ref"
  if [ -f "${stamp_file}" ] &&
     [ "$(<"${stamp_file}")" = "${SMALL_GICP_REF}" ] &&
     [ -d "${SMALL_GICP_INSTALL_PREFIX}/include/small_gicp" ] &&
     [ -f "${SMALL_GICP_INSTALL_PREFIX}/lib/libsmall_gicp.so" ] &&
     [ -f "${SMALL_GICP_INSTALL_PREFIX}/lib/cmake/small_gicp/small_gicp-config.cmake" ]; then
    echo "small_gicp ${SMALL_GICP_REF} is already installed"
    return
  fi

  cmake -S "${SMALL_GICP_SOURCE_DIR}" -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${SMALL_GICP_INSTALL_PREFIX}" \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_BENCHMARKS=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF
  cmake --build "${build_dir}" --parallel "${SMALL_GICP_JOBS}"
  mkdir -p "${SMALL_GICP_INSTALL_PREFIX}"
  if [ -w "${SMALL_GICP_INSTALL_PREFIX}" ]; then
    cmake --install "${build_dir}"
  else
    run_root cmake --install "${build_dir}"
  fi
  printf '%s\n' "${SMALL_GICP_REF}" > "${stamp_file}"
  write_github_env SMALL_GICP_INSTALL_PREFIX "${SMALL_GICP_INSTALL_PREFIX}"
}

hik_mvs_available() {
  local root="$1"
  local library
  local libraries=(
    MvCameraControl
    FormatConversion
    MediaProcess
    MVRender
    MvUsb3vTL
  )

  [ -f "${root}/include/MvCameraControl.h" ] || return 1
  for library in "${libraries[@]}"; do
    [ -e "${root}/lib/64/lib${library}.so" ] || return 1
  done
}

install_hik_mvs() {
  if hik_mvs_available "${HIK_MVS_ROOT}"; then
    echo "Hikrobot MVS SDK found at ${HIK_MVS_ROOT}"
    write_github_env HIK_MVS_AVAILABLE 1
    return
  fi

  if [ "${INSTALL_HIK_MVS}" != "1" ]; then
    echo "Skipping Hikrobot MVS SDK because INSTALL_HIK_MVS=${INSTALL_HIK_MVS}"
    write_github_env HIK_MVS_AVAILABLE 0
    return
  fi

  if [ -z "${HIK_MVS_SOURCE_DIR}" ]; then
    cat >&2 <<EOF
Hikrobot MVS SDK is not available at ${HIK_MVS_ROOT}.
Download the Linux SDK from the official Hikrobot download center, install or
extract it locally, then rerun with:
  INSTALL_HIK_MVS=1 HIK_MVS_SOURCE_DIR=/path/to/MVS bash scripts/ci/install_deps_humble.sh
EOF
    exit 1
  fi

  if ! hik_mvs_available "${HIK_MVS_SOURCE_DIR}"; then
    echo "HIK_MVS_SOURCE_DIR does not look like a valid MVS SDK root: ${HIK_MVS_SOURCE_DIR}" >&2
    exit 1
  fi

  if [ -L "${HIK_MVS_ROOT}" ]; then
    run_root ln -sfn "${HIK_MVS_SOURCE_DIR}" "${HIK_MVS_ROOT}"
  elif [ ! -e "${HIK_MVS_ROOT}" ]; then
    run_root mkdir -p "$(dirname "${HIK_MVS_ROOT}")"
    run_root ln -s "${HIK_MVS_SOURCE_DIR}" "${HIK_MVS_ROOT}"
  fi

  if ! hik_mvs_available "${HIK_MVS_ROOT}"; then
    echo "Hikrobot MVS SDK is present at ${HIK_MVS_SOURCE_DIR}, but ${HIK_MVS_ROOT} does not expose the expected files." >&2
    exit 1
  fi

  echo "Hikrobot MVS SDK available at ${HIK_MVS_ROOT}"
  write_github_env HIK_MVS_AVAILABLE 1
}

install_apt_dependencies
install_rosdep_dependencies
install_python_dependencies
install_small_gicp
install_hik_mvs
install_acados

cat <<EOF
Dependency installation finished.

Pinned external dependencies:
  acados=${ACADOS_REF}
  small_gicp=${SMALL_GICP_REF}
  xmacro=${XMACRO_VERSION}

Generate the MPC solver before the first workspace build:
  bash ${REPO_ROOT}/scripts/ci/generate_acados_mpc.sh

Use this before building:
  source ${REPO_ROOT}/scripts/ci/env_humble.sh
EOF
