# mpc_controller

## 0. tree

```bash
ld@D:~/guganav/src/guga_controller/mpc_controller$ tre -l 3
.
├── package.xml
├── README.md
├── generated               # acados 生成的求解器代码 (COLCON_IGNORE)
│   └── omni
├── mpc_controller.xml      # nav2_core 插件描述
├── CMakeLists.txt
├── src
│   ├── mpc_wrapper.cpp     # acados 求解器封装 (MpcWrapper)
│   └── mpc_controller_node.cpp  # ROS2 控制器节点
├── include
│   └── mpc_controller
│       ├── mpc_wrapper.hpp
│       ├── mpc_controller_node.hpp
│       └── backward.hpp
├── test
│   ├── test_model_cc       # 单测 (COLCON_IGNORE)
│   │   └── test_unicycle_solver.cc
│   └── py_sim              # python 仿真 + acados 代码生成
│       ├── c_codegen
│       │   ├── c_codegen_omni.py
│       │   └── c_codegen_unicycle.py
│       ├── model
│       └── test_model
└── .gitignore
```

> 注：MPC 节点直接输出 body 系（base_footprint）速度，TES 自旋速度叠加与
> nonrotating 坐标系适配由 `nonrotating_vel_transform` 统一完成。
