#!/bin/bash
# guganav构建脚本

cd ~/guganav
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
if [ ! -L /home/ld/guganav/compile_commands.json ]; then
  ln -s  ~/guganav/build/compile_commands.json ~/guganav/compile_commands.json
  echo "已创建compile_commands.json的软链接, 请设置vscode的c_cpp_properties.json以启用代码补全和跳转功能。"
fi