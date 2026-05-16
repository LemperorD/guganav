# 编码规范

两类模式，分别在两个包中形成范例。新功能根据场景选择其中一种。

## 模式 A：函数式数据流（terrain_analysis）

**适用场景：单帧处理，算法无跨帧状态，输入→处理→输出为纯数据变换, 管道设计模式**

| 维度         | 规范                                                                      |
| ------------ | ------------------------------------------------------------------------- |
| **Config**   | `struct` 公开所有字段，含 `static constexpr` 编译期常量 + 运行期参数      |
| **State**    | `struct` 公开所有可变状态，点云、数组、时间戳全透明                       |
| **算法**     | 纯静态方法类 `class TerrainAlgorithm { static void run(config, state); }` |
| **参数传递** | `(const Config&, State&)` 显式传递，不藏隐式依赖                          |
| **Helper**   | 匿名 namespace 自由函数，按功能域组合                                     |
| **测试**     | 直接构造 Config/State struct，设字段 → 调函数 → 断言字段                  |
| **抽象层数** | 2 层：Config+State struct → Algorithm func                                |
| **适用判定** | 算法不需要跨帧记忆、"输入点云→输出点云"类管线                             |

**范例**：`src/guga_perception/terrainanalysis/terrain_analysis/`

## 模式 B：对象式封装（simple_decision）

**适用场景：有跨帧状态管理、多线程读写、需要不可变快照的决策系统**

| 维度         | 规范                                                                                 |
| ------------ | ------------------------------------------------------------------------------------ |
| **Config**   | `struct` 公开字段，构造时传入 **不可变对象** 内部存储为 `const Config config_`       |
| **State**    | 私有成员，藏在 `Context` 内部，外部只通过 `Snapshot` 只读快照访问                    |
| **Snapshot** | 每次 tick 从 Context 构建的值类型，传给决策函数，构造后不应变                        |
| **算法**     | 实例方法类 `class Decision { DecisionAction computeAction(const Snapshot&) const; }` |
| **参数传递** | Config 构造时绑定，State 通过 Snapshot 隔离，不直接暴露                              |
| **Helper**   | 优先私有成员方法（`supplyAction`/`attackAction`），仅纯工具函数放匿名 namespace      |
| **测试**     | 构造 Config → 构造 Context → 调方法 → 从 Context 取值验证，Snapshot 提供可控输入     |
| **抽象层数** | 3 层：Config → Context(State→Snapshot) → Decision                                    |
| **适用判定** | 算法有跨帧记忆（门状态、冷却）、多线程读写、需要外部不可变性保证                     |

**范例**：`src/guga_decision/simple_decision/`

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

- 默认不写注释，只在 WHY 非明显时添加
- 不使用 Doxygen `///`，不使用 `// ...` 分区注释段
