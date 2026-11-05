# guga_common

通用模块，实现共享内存、自定义消息类型、类型别名、通用数学函数等通用功能

## TODOLIST

- [ ] 共享内存
- [ ] 自定义消息
- [ ] 类型别名
- [ ] 通用数学函数

## 现有内容

- `include/shm/`：共享内存通信（`shm_types.hpp` 定义 slot 与 POD 布局、
  reader/writer），构建为 `guga_shm` 库。
- `include/filter.hpp`：`slidingWindowFilter` 等通用滤波模板（头文件）。
- `include/text_color.hpp`:字体颜色库。
- `include/geometry.hpp`：部分常用几何类型及方法。
