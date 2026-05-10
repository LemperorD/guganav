# usbjs_driver

## 0. preview

使用可以被linux识别的usb joystick设备（连接pc后被识别为标准输入设备）驱动仿真中的车

> 仅为测试用功能包，严禁在任何reality的launch中出现

## 1. tree

```bash
├── CMakeLists.txt             # cmake构建文件
├── config
│   └── usbjs.yaml             # 参数
├── include
│   └── usbjs_driver
│       ├── usb_joystick_main.hpp # main类hpp
│       └── usb_joystick_node.hpp # node类hpp
├── launch
│   └── usbjs_driver.launch.py # launch脚本
├── package.xml
├── README.md
└── src
    ├── usb_joystick_main.cpp     # main类cpp
    └── usb_joystick_node.cpp     # node类cpp
```

## 2. 设计模式简述

main类：对js标准输入文件进行只读操作，读取的文件由参数配置，是main类构造函数中的变量，也是成员变量
node类：调用main类api，进行ros2层封装，输出速度话题
