# guga_decision_common

决策方案共用的 ROS 输入层，包含：

- `DecisionInputHub`：只订阅一次机器人、比赛、RFID 和视觉汇总消息，提供线程安全
  快照与 TF 位姿查询。
- `Ros2InputNode`：BehaviorTree.CPP 输入节点，从 `DecisionInputHub` 读取数据，
  写入由行为树显式映射的输出端口。

节点不绑定固定黑板键。树上按需映射端口，例如：

```xml
<Ros2Input HP="{HP}"
           ammo="{ammo}"
           enemy_count="{enemy_count}"
           game_progress="{game_progress}"
           rfid_supply="{rfid_supply}"
           rfid_center="{rfid_center}"
           robot_x="{robot_x}"
           robot_y="{robot_y}"
           pose_valid="{pose_valid}"/>
```

机器人状态是必需输入；未收到时节点返回 `RUNNING`。比赛、RFID 和视觉数据可以
稍后到达，对应的 `*_received` 端口用于区分“未收到”和“真实值为 false/0”。
