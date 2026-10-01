# 已解决问题归档

这里记录**已经定位并修掉**的问题：现象、原因、解决方案、验证依据。未解决或仍在观察的条目不留在此处，它们在 [[TODOLIST.md]]里。

新条目追加到对应分节的末尾。一条只写一个问题，并且把"当时是怎么发现的、怎么验证的"写清楚——这两项比结论更值得复用，因为同一个结论在新场景下未必成立。提交号都可用 `git show <号>` 查到。

本文可以用ai形成初稿,但需要人工进行修改,确保各位咕咕嘎嘎们真的理解并定位到问题的本质,为自己与后来的咕咕嘎嘎们提供可观的帮助。


## 一.实车感知与建图

### 1. 低矮障碍物无法识别

- **现象**：实车低矮障碍物识别不出来。
- **涉及文件**: `terrain_analysis`
- **原因**：在 terrain 的高度判据链条上,有几处会把低矮点丢掉: 逐格点数门限 `minBlockPointNum`、`minRelZ`/`maxRelZ` 的裁剪带、输出上界 `ceilingClearance`，加上坐标基准不一致， 导致离地高度算错（`groundFloorZ` 按错误推算取到 −0.45，`lidar_z` 被当成车高）。这几条当时分散在 odom 绝对、车辆相对、局部地面三个参考系里，只看现象无法判断是哪一条生效。
- **解决方案**：重构，判据统一到"距局部地面的高度"，坐标基准按实车实测改正（odom 与 base_footprint 重合、平地地面 z ≈ 0），地面带与高度带改为 ROS 参数（`minObstacleHeight` 0.04、`ceilingClearance` 0.62、`groundFloorZ` −0.2），之后按实车调参。
- **验证**：实车确认低矮障碍物可被识别。
- **提交**：terrain 重构链条，含基准改正 `56003b9`、配置改为构造注入 `87b61d1`、删除失效的近距豁免 `79542df` 。

### 2. 建图模式无法清除伪静态障碍物，全模式存在鬼点问题

- **现象**：建图模式下动态目标的运动轨迹被当作静态地图保留，清除不掉，时常产生鬼点，影响规划器。
- **涉及文件**: `terrain_analysis`，`obstacle_layer`
- **原因**：地图由 slam_toolbox 维护且没有消退机制；输入是 terrain 的累计点云，已经离开的目标的历史点仍在其中，被累积成静态结构。
- **解决方案**：改用 nav2 的射线追踪清障：对当帧返回的雷达射线从传感器原点画射线清除沿途标记，移走的目标不再留下记忆。terrain 侧因此新增当帧返回的雷达射线云 `terrain_returns_current`（见条目 2），代价地图的清除由它承担。同时带来额外的好处:视线外障碍物消失的 bug 也被修复。
- **验证**：仿真及实车确认。
- **提交**：`131bc7e`、`6b61f78`


## 二.规划与导航

### 1. 多点导航发不出去，日志刷 base_link 不存在

- **现象**：RViz 的 Nav2 面板发单个目标正常；发多点目标（`navigate_through_poses`）时 `bt_navigator` 反复报 `No Transform available Error looking up target frame: "base_link" passed to lookupTransform argument source_frame does not exist`，行为树同时不断清全局/局部代价地图并执行 `BackUp`，车不动。
- **发现方式**：先误判过两次——把 `tf2_echo map base_link` 的输出当成了"TF 树缺 `map` 帧"（实际那只是该进程没收到 TF，空 buffer 会报同样的话），也怀疑过是 RViz 给目标点打了错坐标系。真正有用的一条线索是"单点正常、多点失败"这个差异：两棵行为树只差一个节点。
- **原因**：`RemovePassedGoals`（只在多点行为树里）的 `robot_base_frame` 端口默认值是硬编码的 `base_link`，**不继承** `bt_navigator` 的 `robot_base_frame` 参数（上游 `humble` 分支 `remove_passed_goals_action.hpp` 的两个端口默认值为 `map` 与 `base_link`，`getInput` 直接从 XML 读，不做命名空间解析）。本项目 TF 帧名是全局的 `base_footprint`，树里没有 `base_link`，于是该节点每轮 tick 取机器人位姿都失败，整棵树的规划分支随之失败并进恢复动作。
- **解决方案**：在 `behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml` 里给该节点显式写 `robot_base_frame="base_footprint"` 与 `global_frame="map"`，并加注释说明不能依赖端口默认值。上游同类报告 [navigation2#4508](https://github.com/ros-navigation/navigation2/issues/4508)，维护者按"缺少配置"关闭。
- **验证**：核对了本地库 `libnav2_remove_passed_goals_action_bt_node.so` 的端口表与默认字符串、上游源码、XML 解析通过；**整车重测尚未做**，重测判据是日志不再出现 `transformPoseInTargetFrame` 报错、行为树不再进入清图与 `BackUp` 循环。
- **注意**：帧名在此项目的约定是全局唯一、不带命名空间（话题与服务才随命名空间解析）。若将来 TF 帧名改为带前缀（多机场景），XML 里这两处硬编码会失效，报错形式与本次相同；届时把帧名改为启动参数、由 launch 生成改写后的行为树。
- **提交**：`94c48ec`；相关的坐标系统一改动为 `037c384`（`simple_decision`、`nonrotating_vel_transform` 里的 `base_link` 默认值改为 `base_footprint`）。
