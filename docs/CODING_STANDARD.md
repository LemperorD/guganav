# 编码规范


## 通用规范

以下不区分模式，全项目统一。

### 命名

| 元素               | 约定                                                        |
| ------------------ | ----------------------------------------------------------- |
| class/struct       | PascalCase                                                  |
| 函数/方法          | camelCase                                                   |
| compile-time 常量  | UPPER_CASE `static constexpr int TERRAIN_VOXEL_WIDTH = 21;` |
| 变量（局部/参数）  | snake_case，全拼不简写                                      |
| 私有成员（模式 B） | 尾随 `_`：`config_`, `node_`                                |
| enum class 值      | UPPER_CASE：`UNINITIALIZED`, `RECORDING`                    |
| bool               | `is_`/`has_`/`should_`/`can_` 前缀                          |

### 类型

| 场景           | 使用类型                                      |
| -------------- | --------------------------------------------- |
| 坐标/距离/阈值 | `double`（不用 `float`）                      |
| PCL 点坐标     | `float`（PCL 库要求）                         |
| 数组索引       | `size_t`，非负                                |
| 网格坐标       | `int`，可为负                                 |
| 枚举底层类型   | `uint8_t`（值域小）                           |
| 时间相关       | raw nanoseconds: `int64_t`，seconds: `double` |

### 代码组织

| 规则               | 说明                                                              |
| ------------------ | ----------------------------------------------------------------- |
| `#pragma once`     | 所有头文件                                                        |
| 范围for            | 遍历点云、vector 优先用 `for (const auto& point : cloud->points)` |
| 局部变量提取       | 避免 `state.vehicle_x` 在循环内重复读取，提到循环外               |
| `static constexpr` | 编译期可确定的常量放在 struct 或函数内                            |
| 值初始化           | 用 `{}` 统一初始化：`int x{};` / `double t{};` / `bool flag{};`。禁止 `= 0` / `= 0.0` / `= false` |
| 结构体默认值       | 字段默认值用 `{}`（值初始化）或 `{value}`（带初值）              |
| `auto`             | 类型从上下文明显可推断时用，否则显式写类型                        |
| 显式 cast          | 符号/窄化转换显式 `static_cast<T>()`，不隐式                      |
| `[[nodiscard]]`    | 返回值的查询方法标记                                              |

### 禁止

- 全局变量
- 逗号连续声明 `int x, y, z;` — 每个变量独占一行
- 魔法数字 — `0`/`1`/`-1` 以外的字面量提为命名常量 `constexpr` / `static constexpr`

### 注释

- 默认不写注释，只在原因非明显时添加
- 不使用 Doxygen `///`，不使用 `// ...` 分区注释段
