#!/bin/bash
cd ~/guganav
MAKEFLAGS="-j4" CMAKE_BUILD_PARALLEL_LEVEL=1 \
colcon build \
  --symlink-install \
  --parallel 1\
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-O2 -g -fno-omit-frame-pointer -DNDEBUG"
