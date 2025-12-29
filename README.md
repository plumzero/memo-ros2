
参考:
- [Foxy](https://docs.ros.org/en/foxy/index.html)
- [Foxy Ubuntu 安装](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [ros_tutorials foxy-devel 分支](https://github.com/ros/ros_tutorials)
- [Eclipse Cyclone DDS](https://projects.eclipse.org/projects/iot.cyclonedds)

### DDS 更换

```s
  sudo apt install ros-foxy-rmw-cyclonedds-cpp
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 内容

- [命令行](ch01)
  - [turtlesim和rqt](ch01/01_turtlesim和rqt.md)
  - [节点](ch01/02_节点.md)
  - [主题](ch01/03_主题.md)
  - [服务](ch01/04_服务.md)
  - [参数](ch01/05_参数.md)
  - [动作](ch01/06_动作.md)
  - [rqt_console](ch01/07_rqt_console.md)
  - [启动多节点](ch01/08_启动多节点.md)
  - [数据记录和回放](ch01/09_数据记录和回放.md)
- [基本通信模式](ch02)
  - [发布者和订阅者](ch02/01_发布者和订阅者.md)
  - [发送端和响应端](ch02/02_发送端和响应端.md)
  - [主题和服务](ch02/03_主题和服务.md)
- [数据录制](ch03)
  - [主题数据](ch03/01_主题数据.md)
  - [自定义数据](ch03/02_自定义数据.md)
  - [外部数据](ch03/03_外部数据.md)
- [概念与特性](ch04)
  - [域ID](ch04/01_ROS_DOMAIN_ID.md)
  - [QoS服务质量](ch04/02_QoS服务质量.md)
  - [进程内通信](ch04/03_进程内通信.md)
  - [了解实时编程](ch04/04_了解实时编程.md)
