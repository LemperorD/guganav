#!/usr/bin/env bash

_guganav_ros_distro="${ROS_DISTRO:-humble}"
_guganav_ros_setup="/opt/ros/${_guganav_ros_distro}/setup.bash"

if [ ! -f "${_guganav_ros_setup}" ]; then
  echo "ROS setup file not found: ${_guganav_ros_setup}" >&2
  return 1 2>/dev/null || exit 1
fi

# shellcheck source=/dev/null
source "${_guganav_ros_setup}"

export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-${HOME}/tools/acados}"

case ":${LD_LIBRARY_PATH:-}:" in
  *":${ACADOS_SOURCE_DIR}/lib:"*) ;;
  *)
    export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH:-}"
    ;;
esac

unset _guganav_ros_distro
unset _guganav_ros_setup
