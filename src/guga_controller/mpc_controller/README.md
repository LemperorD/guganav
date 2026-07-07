# mpc_controller

## 0. tree

```bash
ld@D:~/guganav/src/guga_controller/mpc_controller$ tre -l 3
.
├── package.xml
├── README.md
├── generated
│   ├── .gitkeep
│   └── COLCON_IGNORE
├── mpc_controller.xml
├── CMakeLists.txt
├── src
│   ├── core
│   │   ├── path_handler.cpp
│   │   ├── omni_kinematics.cpp
│   │   └── mpc_solver.cpp
│   ├── node
│   │   └── mpc_controller_node.cpp
│   └── trajectory
│       ├── bspline_generator.cpp
│       ├── minco_generator.cpp
│       └── discrete_generator.cpp
├── include
│   └── mpc_controller
│       ├── core
│       └── node
├── test
│   ├── test_model_cc
│   │   ├── COLCON_IGNORE
│   │   ├── test_unicycle_solver.cc
│   │   └── CMakeLists.txt
│   └── py_sim
│       ├── model
│       ├── .gitignore
│       ├── c_codegen.py
│       └── test_model.py
└── .gitignore
```
