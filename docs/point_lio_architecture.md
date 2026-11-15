# Point-LIO 架构与文件目录

本文记录 `src/guga_localization/LIO/point_lio` 的分层结构、文件职责和数据流。
最后更新于 2026 年 11 月 13 日。

## 分层结构

代码分为四层，依赖方向从上到下，没有反向引用。

节点层是 `LaserMappingNode`，负责 ROS2 边界：读取参数、建立订阅和发布器、驱动主循环。

调度层是 `FrameProcessor`，负责一帧的完整流程：配时间、传播、量测更新、写结果。

数据层是 `DataHandler` 和 `MeasureGroup`。`DataHandler` 放配置和跨帧输出，`MeasureGroup` 放 `Synchronizer` 产出的单帧输入。

模块层包括 `Lidar`、`Imu`、`Synchronizer`、`ManifoldEskf`。`Lidar` 和 `Imu` 只处理纯数据，不依赖任何 ROS 消息类型。

## 文件清单

### 节点头文件

| 文件                               | 职责                           |
| ---------------------------------- | ------------------------------ |
| `include/point_lio/laserMapping.h` | 节点类声明：参数、成员、主循环 |

### 节点头与源文件

| 文件                   | 职责                                               |
| ---------------------- | -------------------------------------------------- |
| `src/laserMapping.cpp` | 节点实现：参数读取、接口创建、主循环调度、帧尾发布 |

### 核心头文件

| 文件                     | 职责                                               |
| ------------------------ | -------------------------------------------------- |
| `core/common_lib.h`      | 类型别名、状态定义、`MeasureGroup`、`FilterOutput` |
| `core/parameters.h`      | 全部参数结构体                                     |
| `core/PointTypes.h`      | PCL 点类型                                         |
| `core/DataHandler.h`     | 配置与跨帧输出仓库                                 |
| `core/ProcessingState.h` | 主循环工作区，含降采样滤波器和初始化点云           |
| `core/preprocess.h`      | 点云预处理类与切帧参数                             |

### 调度层

| 文件                          | 职责                                     |
| ----------------------------- | ---------------------------------------- |
| `core/FrameProcessor.h`       | 帧调度类声明                             |
| `src/core/FrameProcessor.cpp` | 帧调度实现：配时间、传播、量测、地图更新 |

### 数据输出层

| 文件                                  | 职责                                 |
| ------------------------------------- | ------------------------------------ |
| `core/FramePublisher.h`               | 点云转消息并发布                     |
| `src/core/FramePublisher.cpp`         | 世界系、机体系、地图点云发布         |
| `core/ProjectOutputPublisher.h`       | odom 系点云与里程计发布              |
| `src/core/ProjectOutputPublisher.cpp` | camera_init 到 odom 的坐标转换与发布 |
| `core/OdomProcessor.h`                | camera_init 到 odom 的固定变换       |
| `src/core/OdomProcessor.cpp`          | 变换解析：参数或 TF 查询             |

### 地图持久化层

| 文件                           | 职责                                        |
| ------------------------------ | ------------------------------------------- |
| `core/PriorMapManager.h`       | 先验地图累积、写盘、GICP 状态响应、重载请求 |
| `src/core/PriorMapManager.cpp` | PCD 路径生成、原子写盘、增量更新            |

### 传感器接入层

| 文件                               | 职责                          |
| ---------------------------------- | ----------------------------- |
| `core/SensorSubscriptions.h`       | 传感器订阅与消息转换          |
| `src/core/SensorSubscriptions.cpp` | 点云和 IMU 消息回调，切帧路径 |

### 模块层

| 文件                        | 职责                     |
| --------------------------- | ------------------------ |
| `core/Lidar.h`              | 点云缓冲、合帧、切帧入队 |
| `src/core/Lidar.cpp`        | 点云缓冲实现             |
| `core/Imu.h`                | IMU 样本缓冲             |
| `src/core/Imu.cpp`          | IMU 缓冲实现、静止初始化 |
| `core/Synchronizer.h`       | 雷达与 IMU 配时间        |
| `src/core/Synchronizer.cpp` | 配时间实现               |
| `core/ManifoldEskf.h`       | 30 维误差状态滤波器      |
| `src/core/ManifoldEskf.cpp` | 传播、协方差、量测更新   |

## 数据流

输入方向：ROS 消息进入 `SensorSubscriptions`，`Preprocess` 转成 `PointCloudXYZI` 或 `ImuMeasurement`，经 `FrameProcessor::onLidarFrame` 和 `onImu` 交给 `Lidar`、`Imu` 的缓冲。

处理方向：`FrameProcessor::processIteration()` 调 `Synchronizer` 从缓冲取一帧，填 `MeasureGroup`，交给 `ManifoldEskf` 做传播和量测更新。

输出方向：`ManifoldEskf` 的结果写进 `DataHandler`，节点从 `DataHandler` 取出来，经 `FramePublisher` 和 `ProjectOutputPublisher` 发布。

地图方向：`PriorMapManager` 从 `DataHandler` 取世界系点云，按 GICP 状态决定是否累积，按间隔或触发条件写盘。

## 依赖关系

`FrameProcessor` 依赖 `Lidar`、`Imu`、`ManifoldEskf`、`Synchronizer`、`DataHandler`。

`PriorMapManager` 依赖 `OdomProcessor`，不依赖任何发布类。

`ProjectOutputPublisher` 依赖 `OdomProcessor`。

`FrameProcessor` 不持有任何 `rclcpp::Publisher`，也不接收发布回调。

## 配置与生命周期

参数在 `LaserMappingNode::initializeParameters` 里读取，写入 `DataHandler.config`。

`Lidar` 和 `Imu` 持有 `DataHandler.config` 对应子结构的只读指针，动态调参改配置后模块能直接看到新值。

`FrameProcessor::initialize` 一次性配置 `Lidar`、`Imu`、`Synchronizer`、`ivox`、外参、降采样滤波器和滤波器初始协方差。
