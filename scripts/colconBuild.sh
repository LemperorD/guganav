#!/bin/bash
# guganav构建脚本

cd ~/guganav
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
sudo ln -s ~/guganav/build/compile_commands.json ~/guganav/compile_commands.json