# serial_driver

## 0. preview

串口通信模块

## 1. tree

```bash
├── CMakeLists.txt
├── config
│   └── communication.yaml
├── include
│   └── serial_driver
│       ├── bridge.hpp
│       ├── Com.hpp
│       ├── ros_serial_bridge.hpp
│       └── termcolor.hpp
├── launch
│   └── communication.launch.py
├── package.xml
├── README.md
└── src
    ├── bridge.cpp
    └── Com.cpp
```

> 因为是从之前的项目copy过来的，目前代码内命名空间不规范，亟待修改

## 2. 设计模式简述

main类：对js标准输入文件进行只读操作，读取的文件由参数配置，是main类构造函数中的变量，也是成员变量
node类：调用main类api，进行ros2层封装，输出速度话题
