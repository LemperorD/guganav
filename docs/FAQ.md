# 常见问题

## 找不到 acados

确认 `ACADOS_SOURCE_DIR` 指向实际安装目录，并且 `LD_LIBRARY_PATH` 包含 acados 的 `lib` 目录：

```bash
export ACADOS_SOURCE_DIR=$HOME/tools/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}
```

## `mpc_controller` 找不到生成的 solver

先生成 omni solver，再重新构建：

```bash
cd ~/guganav/src/guga_controller/mpc_controller/test/py_sim
ACADOS_SOURCE_DIR=$HOME/tools/acados \
LD_LIBRARY_PATH=$HOME/tools/acados/lib:${LD_LIBRARY_PATH:-} \
PYTHONPATH=$PWD \
python3 c_codegen/c_codegen_omni.py
```

## 构建产物是否需要提交

不需要。`build/`、`install/`、`log/` 是本地构建产物，不应提交。`compile_commands.json` 通常链接到
`build/compile_commands.json`，供 clangd/VS Code 使用。

## 实车启动前要检查什么

确认雷达、串口、相机 SDK、地图、PCD 和参数文件路径均与当前机器一致。实车参数主要位于
`src/guga_bringup/config/reality/`。

## rviz因未知原因打不开
将你的互联网设置为buaa mobile试试.

