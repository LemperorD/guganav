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
