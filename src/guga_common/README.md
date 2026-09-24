# guga_common

通用模块，实现共享内存、自定义消息类型、类型别名、通用数学函数等通用功能

## TODOLIST

- [ ] 共享内存
- [ ] 自定义消息
- [ ] 类型别名
- [ ] 通用数学函数

## 现有内容

- `include/guga_common/shm/`：共享内存通信（`shm_types.hpp` 定义 slot 与 POD 布局、
  reader/writer），构建为 `guga_shm` 库。
- `include/guga_common/common_libs.hpp` + `src/common_libs.cpp`：通用小工具（终端颜色、角度解缠），
  构建为 `common_libs` 库。
- `include/guga_common/filter.hpp`：`slidingWindowFilter` 等通用滤波模板（头文件）。
- `include/guga_common/geometry.hpp`：几何基础类型。`guga_common::Point3d` 是三个 double 的
  xyz 位置（不含姿态），供各包共用，避免每处各自定义一个"位置"结构体。
- `msg/`：本项目自定义消息（MincoTrajectory、裁判系统与敌方目标相关）。
