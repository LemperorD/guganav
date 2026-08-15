#!/bin/bash
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)
TERRAIN_ROOT="$WS/src/guga_perception/terrain_analysis"

source_setup() {
  local setup_file=$1
  if [ -f "$setup_file" ]; then
    set +u
    source "$setup_file"
    set -u
  fi
}

if [ -z "${ROS_DISTRO:-}" ]; then
  source_setup /opt/ros/humble/setup.bash
fi
cd "$WS"

rm -rf build/terrain_analysis build/terrain_analysis_ext \
  install/terrain_analysis install/terrain_analysis_ext \
  log/latest_build/terrain_analysis log/latest_build/terrain_analysis_ext

colcon build \
  --base-paths "$TERRAIN_ROOT" \
  --symlink-install \
  --parallel-workers 1 \
  --allow-overriding terrain_analysis \
  --packages-select terrain_analysis terrain_analysis_ext \
  --event-handlers console_direct+ \
  --cmake-clean-cache \
  --cmake-args \
    -DBUILD_TESTING=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source_setup "$WS/install/setup.bash"

cd "$WS/build/terrain_analysis"
for t in test_terrain_analysis test_state_ingest test_algorithm; do
  ./$t >/dev/null 2>&1 || { echo "FAILED: $t"; exit 1; }
done
echo "terrain_analysis: all 3 test suites OK"
echo "terrain_analysis_ext: build OK"
