"""
启动 guga_ui_pangolin 独立进程。

guga_ui_pangolin 不依赖 ROS2 运行时（仅通过 POSIX 共享内存读取数据），
因此使用 ExecuteProcess 直接启动可执行文件，而非 ROS2 Node。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    启动 Pangolin UI 进程。

    可配置参数:
      - shm_name: 共享内存名称 (默认 "guga_shm")
      - workspace_root: 工作空间根路径 (默认通过 ament_index 推断)
    """
    shm_name = LaunchConfiguration(
        "shm_name", default="guga_shm"
    )

    # 推断 install 目录下 guga_ui 可执行文件的路径。
    # 安装后位于 <install_prefix>/lib/guga_ui_pangolin/guga_ui
    ui_share_dir = get_package_share_directory("guga_ui_pangolin")
    # share 如: <workspace>/install/guga_ui_pangolin/share/guga_ui_pangolin
    # 回退两层到 install 根, 再进入 lib/guga_ui_pangolin/
    install_base = os.path.normpath(
        os.path.join(ui_share_dir, "..", "..")
    )
    ui_executable = os.path.join(
        install_base, "lib", "guga_ui_pangolin", "guga_ui"
    )

    ui_process = ExecuteProcess(
        cmd=[ui_executable, shm_name],
        name="guga_ui_pangolin",
        output="screen",
        # UI 进程崩溃不影响导航栈
        respawn=False,
        # UI 接收 SIGINT 后自行退出 (main.cpp 已注册 handler)
        sigkill_timeout=2.0,
    )

    return [
        LogInfo(
            msg=["[guga_ui_launch] starting Pangolin UI: ", ui_executable,
                 " shm=", shm_name]
        ),
        ui_process,
    ]
