#!/bin/bash
# guganav构建脚本

show_help() {
cat << EOF
Usage:
    <workspace>/scripts/colconBuild.sh [OPTIONS]

Options:
    -h / --help          显示帮助
    -c / --clean         清理build目录
    -r / --release       Release编译
    -d / --debug         Debug编译
EOF
}

# 默认参数
CLEAN=false
BUILD_TYPE=Release

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h | --help)
            show_help
            exit 0
            ;;
        -c | --clean)
            CLEAN=true
            shift
            ;;
        -r | --release)
            BUILD_TYPE=Release
            shift
            ;;
        -d | --debug)
            BUILD_TYPE=Debug
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage."
            exit 1
            ;;
    esac
done

echo "Build type : $BUILD_TYPE"
echo "Clean      : $CLEAN"

if $CLEAN; then
  echo "[warn]maybe need ur password"
  sudo rm -rf build/ log/ install/
  echo "Cleaning..."
fi

echo "Building..."

cd ~/guganav
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
if [ ! -L /home/ld/guganav/compile_commands.json ]; then
  ln -s  ~/guganav/build/compile_commands.json ~/guganav/compile_commands.json
  echo "已创建compile_commands.json的软链接, 请设置vscode的c_cpp_properties.json以启用代码补全和跳转功能。"
fi