
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

- [基本通信模式](ch01)
  - [发布者和订阅者](ch01/发布者和订阅者.md)
  - [发送端和响应端](ch01/发送端和响应端.md)
  - [主题和服务](ch01/主题和服务.md)
- [数据录制](ch02)
  - [主题数据](ch02/主题数据.md)
  - [定时器数据](ch02/定时器数据.md)
  - [外部数据](ch02/外部数据.md)