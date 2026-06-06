# 北航Transistor战队27赛季哨兵工作空间

致力于打造咕咕嘎嘎也能学会的导航代码仓库！

<p align="center">
	<img src="docs/img/Transistor算法组.jpg" alt="Transistor算法组" width="460" />
</p>

## 提交前检查 (pre-commit)

安装 pre-commit 工具：

```bash
pip install pre-commit
```

注册到本仓库（在项目根目录执行一次）：

```bash
pre-commit install
```

之后每次 `git commit` 自动运行：

| 检查项                | 触发条件                                                                    | 行为                 |
| --------------------- | --------------------------------------------------------------------------- | -------------------- |
| clang-format          | 修改了核心包 C++ 文件                                                       | 自动格式化后拒绝提交 |
| terrain_analysis 测试 | 修改了 `src/guga_perception/terrain_analysis/` 或对应测试脚本               | 测试失败则拒绝提交   |
| PID 控制器测试        | 修改了 `src/guga_controller/pb_omni_pid_pursuit_controller/` 或对应测试脚本 | 测试失败则拒绝提交   |
| simple_decision 测试  | 修改了 `src/guga_decision/simple_decision/` 或对应测试脚本                  | 测试失败则拒绝提交   |

