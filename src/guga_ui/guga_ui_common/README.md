# guga_ui_common — 共享内存通信库

基于 POSIX 共享内存的 **lock-free** 数据桥梁，连接 ROS2 算法模块与独立 UI 进程。

**核心特性**：无 ROS2 依赖（header-only 库）、lock-free 读写、64 字节缓存行对齐、固定 8 槽位。

---

## 1. 快速开始

### 1.1 声明依赖

在你的 `package.xml` 中添加：

```xml
<depend>guga_ui_common</depend>
```

CMakeLists.txt 中使用 `ament_auto` 方式（常见旧包）：

```cmake
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
```

或手动方式（新包）：

```cmake
find_package(guga_ui_common REQUIRED)
# ...
target_link_libraries(your_target guga_ui_common::guga_ui_common)
```

### 1.2 包含头文件

```cpp
#include "guga_ui_common/shm_writer.hpp"   // 写入端
#include "guga_ui_common/shm_reader.hpp"   // 读取端
#include "guga_ui_common/ui_types.hpp"     // 数据类型定义
```

---

## 2. 架构总览

```
算法模块 (serial_driver / simple_decision / point_lio / jps_planner)
    │  ShmWriter::write(&data, sizeof(data))
    ▼
┌─────────────────────────────────────────────────┐
│  POSIX shared memory "guga_shm"                 │
│ ┌──────────┬─────────────────┬────────────────┐ │
│ │ShmHeader │ ShmSlot[0..15]  |Slot Data[0..15]| │
│ │ (64B)    │  (64B × 16)     | (64B × 16)     | │
│ └──────────┴─────────────────┴────────────────┘ │
│  共 2112 字节，lock-free seq 协议                 │
└─────────────────────────────────────────────────┘
    │  ShmReader::read(slot_id, &out)
    ▼
Pangolin UI 进程 (guga_ui_pangolin)
```

### 2.1 槽位分配表

| 槽位 | 枚举值 | 数据类型 | 大小 | 写入端 | 说明 |
|------|--------|----------|------|--------|------|
| 0 | `ROBOT_STATUS` | `UiRobotStatus` | 64B | `serial_driver` | 血量/热量/金币 |
| 1 | `GAME_STATUS` | `UiGameStatus` | 64B | `serial_driver` | 比赛阶段/剩余时间 |
| 2 | `RFID_STATUS` | `UiRfidStatus` | 64B | `serial_driver` | 增益点状态 |
| 3 | `DECISION` | `UiDecision` | 64B | `simple_decision` | 决策状态/导航目标 |
| 4 | `ENEMY` | `UiEnemy` | 64B | `simple_decision` | 敌方检测信息 |
| 5 | `ODOM` | `UiOdom` | 64B | `point_lio` | 里程计位姿 |
| 6 | `YAW` | `UiYaw` | 64B | `serial_driver` | 云台偏航角 |
| 7 | **`PATH`** | `UiPath` | 64B+ | **`jps_planner`** | **导航规划路径** |

> **注意**：新增槽位必须追加在末尾，不可删除或重排已有槽位，以保证向后兼容。

---

## 3. 写入端 API (ShmWriter)

### 3.1 核心步骤

```cpp
#include "guga_ui_common/shm_writer.hpp"
#include "guga_ui_common/ui_types.hpp"

// ── 步骤 1: 在模块类中声明成员 ──
class MyPlanner {
private:
  guga_ui::ShmWriter shm_writer_;
  // ...
};

// ── 步骤 2: 初始化（通常在 configure / activate / 构造函数中） ──
bool ok = shm_writer_.init("guga_shm", guga_ui::UiSlotId::PATH);
if (!ok) {
  RCLCPP_ERROR(logger_, "ShmWriter init failed, UI path display unavailable");
}

// ── 步骤 3: 在数据更新处写入 ──
guga_ui::UiPath path_data{};
path_data.stamp_sec = node->now().seconds();
path_data.count = std::min(plan.poses.size(), guga_ui::UI_PATH_MAX_POINTS);
for (size_t i = 0; i < path_data.count; ++i) {
  path_data.x[i] = plan.poses[i].pose.position.x;
  path_data.y[i] = plan.poses[i].pose.position.y;
}
shm_writer_.write(&path_data, sizeof(path_data));
```

### 3.2 关键 API

| 方法 | 说明 |
|------|------|
| `init(name, slot_id)` | 打开或创建共享内存。首次调用创建 shm + 写入 header，后续调用校验 magic/version。失败返回 false。 |
| `write(data, size)` | 将数据写入对应槽位。如果 `size > 64`（如 `UiPath`），会**截断为 64 字节**。 |

### 3.3 UiPath 特别说明

`UiPath` 结构体超过 64 字节（约 4160 字节），但 `ShmWriter::write()` 默认只复制前 64 字节（`SHM_SLOT_DATA_SIZE`）以避免破坏其他槽位。

**对于 UiPath 这种大结构体，写入时需要直接操作该槽位的数据区**。`guga_ui_common` 目前每个槽位固定 64 字节数据区——要对 PATH 槽位传完整体积，需要确认 shm 布局已为该槽分配了足够空间。如果数据区仍为 64 字节，你有两个选择：

1. **降采样**：只写入前 7 个路径点（恰好拟合 64 字节：count(4B) + stamp_sec(8B) + x[7](56B) + y[7](56B) = 124B → 超出）。实际上 64 字节只够存 3 个点。
2. **扩展 PATH 槽位的数据区**：修改 `shm_layout.hpp` 中 `SHM_SLOT_DATA_SIZE` 为更大的值（如 4160），或者给特定槽分配不同大小的数据区。

> 关于 PATH 槽位数据区容量，建议联系 @ld 在当前分支确认 shm_layout 是否需要为 PATH 单独扩展。README 中按需降采样的方案先作为参考。

### 3.4 Lock-free 写入协议

写入过程分三步，保证读取端永远不会看到半写入的数据：

```
1. seq |= 1  (奇数，表示"写入进行中")
2. memcpy 数据区
3. seq += 2  (偶数，表示"写入完成")
```

---

## 4. 读取端 API (ShmReader)

### 4.1 核心步骤

```cpp
#include "guga_ui_common/shm_reader.hpp"
#include "guga_ui_common/ui_types.hpp"

// ── 步骤 1: 构造 & 打开 ──
guga_ui::ShmReader reader;
bool ok = reader.open("guga_shm");
if (!ok) {
  // shm 尚未创建（没有写入端在运行），UI 降级显示 "no data"
  return;
}

// ── 步骤 2: 每帧检查是否有新数据 ──
if (reader.checkFresh(guga_ui::UiSlotId::PATH)) {
  // seq 已递增，有新数据
}

// ── 步骤 3: 读取数据 ──
guga_ui::UiPath path_data{};
bool success = reader.read(guga_ui::UiSlotId::PATH,
                           &path_data, sizeof(path_data));
if (success) {
  for (uint32_t i = 0; i < path_data.count; ++i) {
    // 使用 path_data.x[i], path_data.y[i]
  }
}
```

### 4.2 关键 API

| 方法 | 说明 |
|------|------|
| `open(name)` | 打开已有共享内存（O_RDONLY）。如果 shm 不存在，返回 false。校验 magic/version。 |
| `isValid()` | 查询 shm 是否已打开且校验通过。 |
| `checkFresh(slot_id)` | 判断自上次 `read()` 以来该槽位是否有新数据。 |
| `read(slot_id, &out, size)` | 读取数据。使用 double-check seq 模式保证一致性，最多重试 3 次。返回 false 表示数据不完整（写入竞争）。 |
| `getSeq(slot_id)` | 获取当前 seq 号，用于自定义比较。 |
| `slotCount()` | 获取 shm 中注册的槽位总数。 |

### 4.3 完整读取循环（参考用）

```cpp
void update() {
  if (!reader_.isValid()) return;

  // 轮询所有槽位，只读取有更新的
  for (uint32_t i = 0; i < static_cast<uint32_t>(guga_ui::UiSlotId::COUNT); ++i) {
    auto slot = static_cast<guga_ui::UiSlotId>(i);
    if (!reader_.checkFresh(slot)) continue;

    switch (slot) {
      case guga_ui::UiSlotId::ROBOT_STATUS:
        reader_.read(slot, &ui_data_.robot_status, sizeof(ui_data_.robot_status));
        break;
      case guga_ui::UiSlotId::PATH:
        reader_.read(slot, &ui_data_.path, sizeof(ui_data_.path));
        break;
      // ... 其余槽位类似
      default: break;
    }
  }
}
```

---

## 5. 路径规划器集成示例 (JPS → UI)

将 JPS 规划的路径写入共享内存，供 UI 渲染。

### 5.1 在 JPSPlanner 中添加 ShmWriter

```cpp
// jps_planner.hpp — 添加私有成员
#include "guga_ui_common/shm_writer.hpp"

private:
  guga_ui::ShmWriter shm_writer_;
  bool shm_ready_{false};
```

### 5.2 在 configure() 中初始化

```cpp
// jps_planner.cpp — JPSPlanner::configure() 末尾
shm_ready_ = shm_writer_.init("guga_shm", guga_ui::UiSlotId::PATH);
if (shm_ready_) {
  RCLCPP_INFO(logger_, "JPSPlanner: shm writer ready for PATH slot");
} else {
  RCLCPP_WARN(logger_, "JPSPlanner: shm writer init failed, UI path unavailable");
}
```

### 5.3 在 createPlan() 中写入路径

```cpp
// jps_planner.cpp — JPSPlanner::createPlan() 在 return plan 之前

if (shm_ready_ && !plan.poses.empty()) {
  guga_ui::UiPath ui_path{};
  ui_path.stamp_sec = clock_->now().seconds();
  ui_path.count = std::min(plan.poses.size(), guga_ui::UI_PATH_MAX_POINTS);
  for (size_t i = 0; i < ui_path.count; ++i) {
    ui_path.x[i] = plan.poses[i].pose.position.x;
    ui_path.y[i] = plan.poses[i].pose.position.y;
  }
  shm_writer_.write(&ui_path, sizeof(ui_path));
}
```

### 5.4 在 cleanup() 中清理

`ShmWriter` 析构函数会自动 `munmap`，无需手动清理。但不主动 `shm_unlink`（因为其他进程可能还在使用）。

---

## 6. Lock-free 协议细节

| 状态 | seq 值 | 读取端行为 |
|------|--------|-----------|
| 初始（未写入） | 0（偶数） | 可读（全零数据） |
| 写入进行中 | 奇数（seq \| 1） | 跳过，等待重试 |
| 写入完成 | 偶数（seq + 2） | 读数据，前后 seq 一致 → 有效 |
| 多次写入后 | 2, 4, 6, ... | 每次递增 2 |

读取端的 double-check 流程：

```
1. seq1 = slot->seq.load(acquire)   // 必须为偶数
2. memcpy(out, shm_data, size)
3. seq2 = slot->seq.load(acquire)
4. if seq1 == seq2 → 数据完整，更新 last_seqs_ 缓存
   else           → 重试（最多 3 次）
```

---

## 7. 注意事项

1. **共享内存名称一致性**：写入端和读取端必须使用相同的 shm 名称（默认为 `"guga_shm"`，定义在 `shm_layout.hpp` 的 `SHM_DEFAULT_NAME`）。

2. **启动顺序**：写入端必须先于读取端启动（或至少有一个写入端先创建了 shm）。UI 进程在 shm 不存在时会降级显示 "no data"，不会崩溃。

3. **SIGINT 安全**：`ShmWriter`/`ShmReader` 的析构函数在进程退出时自动回收资源。但如果进程被 SIGKILL 杀掉，shm 会残留——推荐写入端定期检查或由启动脚本管理 `/dev/shm/` 清理。

4. **线程安全**：`write()` 和 `read()` 各自是线程安全的（通过 atomic seq 实现），但多个进程写入同一槽位会导致数据交叉覆盖。**每个槽位只能有一个写入者**。

5. **数据大小**：单个槽位数据区默认 **64 字节**。写入超过此大小的结构体会被截断。对于 `UiPath` 这类大结构体，需确认数据区容量——参见第 3.3 节。

6. **UiPath 降采样参考代码**（当 PATH 槽数据区为 64 字节时）：

```cpp
// 只传前 3 个点（拟合 64B 限制）
guga_ui::UiPath ui_path{};
ui_path.count = std::min(plan.poses.size(), 3UL);
for (size_t i = 0; i < ui_path.count; ++i) {
  ui_path.x[i] = plan.poses[i].pose.position.x;
  ui_path.y[i] = plan.poses[i].pose.position.y;
}
shm_writer_.write(&ui_path, sizeof(ui_path));
```

7. **依赖声明**：使用 `guga_ui_common` 的包必须在 `package.xml` 中添加 `<depend>guga_ui_common</depend>`。

---

## 8. 文件索引

| 文件 | 说明 |
|------|------|
| `include/guga_ui_common/shm_layout.hpp` | 共享内存布局常量、ShmHeader/ShmSlot 结构体、偏移计算工具函数 |
| `include/guga_ui_common/shm_writer.hpp` | 写入端 API（`init`, `write`） |
| `include/guga_ui_common/shm_reader.hpp` | 读取端 API（`open`, `checkFresh`, `read`, `getSeq`） |
| `include/guga_ui_common/ui_types.hpp` | 8 个 POD 数据类型定义、`UiSlotId` 槽位枚举 |
| `test/test_shm_write_read.cpp` | 写入/读取集成测试（参考用例） |
| `test/test_ui_types.cpp` | 类型 POD 断言测试 |
| `test/test_shm_layout.cpp` | 布局偏移计算测试 |
