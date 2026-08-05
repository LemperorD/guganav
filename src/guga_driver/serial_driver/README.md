# serial_driver

## 0. preview

串口通信模块

## 1. tree

```bash
├── CMakeLists.txt
├── config
│   └── serial_driver.yaml
├── include
│   └── serial_driver
│       ├── ros_serial_bridge.hpp
│       ├── serial_driver_main.hpp
│       └── serial_driver_node.hpp
├── launch
│   └── communication.launch.py
├── package.xml
├── README.md
└── src
    ├── serial_driver_main.cpp
    └── serial_driver_node.cpp
```

> 该包是重构后的串口通信入口，旧版 `communication_OLD` 已废弃。

## 2. 设计模式简述

main类：对js标准输入文件进行只读操作，读取的文件由参数配置，是main类构造函数中的变量，也是成员变量
node类：调用main类api，进行ros2层封装，输出速度话题
