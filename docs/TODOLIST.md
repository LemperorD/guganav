# TODOLIST

- 感知
  - [ ] 坡面识别：让哨兵在上坡的时候对准坡面法线，可以参考川大开源
  - [ ] Point-LIO重构：不是很急，主要是Point-LIO代码太乱了，这个任务可以先放放

  ```
  感觉可以当黑盒用 --LHZ
  ```

-  规划
```
已处理 --LHZ
```

- 控制

  - [ ] ~~调研并实验 MPPI~~ + GPU 方案


- 驱动
  - [ ] 相机驱动：已重构，未测试，如果今年还是双上位机则需求不大

- 杂项
  - [ ] 共享内存：已经在guga_common中写好了，整理出哪些部分需要写入共享内存并放入对应功能包
  - [ ] UI：百废待兴，但可能需求不大
  - [x] 仓库总Readme撰写

- 重构
  - [ ] src/guga_ui_old/guga_ui_common/include/guga_ui_common/ui_types.hpp 里全是魔法数字,待修复.
  - [ ] [pointlio] curvature 字段语义化: PointType(pcl::PointXYZINormal) 的 curvature 被复用为帧内时间偏移(ms), 全项目约70处使用, 语义不清. 待选方案: A=访问器 point_time()/ranges投影(零风险, 改动小) vs B=自定义结构体 + union{curvature,time} + POINT_CLOUD_REGISTER_POINT_STRUCT(可写 p.time, 需改~70处并重新编译验证). 注: 1) normal_x/y/z 为死代码(只写0不读); 2) 发布点云走 pcl::toROSMsg, 改结构体必须保留字段注册, 否则丢字段; 3) 缘由: PCL 基础点类型无时间戳字段, LOAM 家族(FAST-LIO/LIO-SAM等)均复用 float 字段存时间, 属行业惯例
