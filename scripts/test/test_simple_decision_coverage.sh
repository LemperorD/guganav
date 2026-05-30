#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/rog/nav2_ws/install/setup.bash
cd ~/guganav

RESULT_FILE=~/guganav/test_result.ans
rm -f "$RESULT_FILE"

echo "=== Clean previous build data ===" | tee -a "$RESULT_FILE"
rm -rf build/simple_decision

echo "=== Build simple_decision with coverage flags ===" | tee -a "$RESULT_FILE"
colcon build --packages-select simple_decision \
  --event-handlers console_direct+ \
  --cmake-args \
    -DBUILD_TESTING=ON \
    -DCMAKE_CXX_FLAGS="--coverage -O0" \
    -DCMAKE_C_FLAGS="--coverage -O0" \
    -DCMAKE_EXE_LINKER_FLAGS="--coverage" \
    -DCMAKE_SHARED_LINKER_FLAGS="--coverage" \
  2>&1 | tee -a "$RESULT_FILE"

. ~/guganav/install/setup.bash

echo "=== Run tests ===" | tee -a "$RESULT_FILE"
cd ~/guganav/build/simple_decision
for test_bin in test_transform test_environment_context test_decision test_simple_decision; do
  echo "--- $test_bin ---" | tee -a "$RESULT_FILE"
  GTEST_COLOR=yes ./$test_bin 2>&1 | tee -a "$RESULT_FILE"
done

echo "=== Generate coverage report ===" | tee -a "$RESULT_FILE"
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
  -o build/simple_decision/coverage.html 2>&1 | tee -a "$RESULT_FILE"

echo "" | tee -a "$RESULT_FILE"
echo "=== Generate lcov info ===" | tee -a "$RESULT_FILE"
gcovr \
  --root . \
  --object-directory build/simple_decision \
  --filter "${FILTER_BASE}/src/.*\\.cpp" \
  --filter "${FILTER_BASE}/include/.*\\.hpp" \
  --exclude '.*test.*' \
  --gcov-ignore-errors=source_not_found \
  --lcov \
  -o lcov.info 2>&1 | tee -a "$RESULT_FILE"

echo "" | tee -a "$RESULT_FILE"
echo "=== Coverage Summary ===" | tee -a "$RESULT_FILE"
gcovr \
  --root . \
  --object-directory build/simple_decision \
  --filter "${FILTER_BASE}/src/.*\\.cpp" \
  --filter "${FILTER_BASE}/include/.*\\.hpp" \
  --exclude '.*test.*' \
  --gcov-ignore-errors=source_not_found 2>&1 | tee -a "$RESULT_FILE"

echo "" | tee -a "$RESULT_FILE"
echo "=== Done ===" | tee -a "$RESULT_FILE"
echo "lcov info:    lcov.info" | tee -a "$RESULT_FILE"
