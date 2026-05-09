#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/rog/nav2_ws/install/setup.bash
cd ~/guganav

echo "=== Clean previous build data ==="
rm -rf build/simple_decision

echo "=== Build simple_decision with coverage flags ==="
colcon build --packages-select simple_decision \
  --cmake-args \
    -DBUILD_TESTING=ON \
    -DCMAKE_CXX_FLAGS="--coverage -O0" \
    -DCMAKE_C_FLAGS="--coverage -O0" \
    -DCMAKE_EXE_LINKER_FLAGS="--coverage" \
    -DCMAKE_SHARED_LINKER_FLAGS="--coverage"

. ~/guganav/install/setup.bash

echo "=== Run tests ==="
cd ~/guganav/build/simple_decision
for test_bin in test_transform test_environment_context test_decision test_simple_decision; do
  echo "--- $test_bin ---"
  ./$test_bin
done

echo "=== Generate coverage report ==="
cd ~/guganav
FILTER_BASE='src/guga_decision/simple_decision'
gcovr \
  --root . \
  --object-directory build/simple_decision \
  --filter "${FILTER_BASE}/src/.*\\.cpp" \
  --filter "${FILTER_BASE}/include/.*\\.hpp" \
  --exclude '.*test.*' \
  --gcov-ignore-errors=source_not_found \
  --html --html-details \
  -o build/simple_decision/coverage.html

echo ""
echo "=== Generate lcov info (for VSCode Coverage Gutters) ==="
gcovr \
  --root . \
  --object-directory build/simple_decision \
  --filter "${FILTER_BASE}/src/.*\\.cpp" \
  --filter "${FILTER_BASE}/include/.*\\.hpp" \
  --exclude '.*test.*' \
  --gcov-ignore-errors=source_not_found \
  --lcov \
  -o lcov.info

echo ""
echo "=== Coverage Summary ==="
gcovr \
  --root . \
  --object-directory build/simple_decision \
  --filter "${FILTER_BASE}/src/.*\\.cpp" \
  --filter "${FILTER_BASE}/include/.*\\.hpp" \
  --exclude '.*test.*' \
  --gcov-ignore-errors=source_not_found

echo ""
echo "=== Done ==="
echo "lcov info:    lcov.info"
