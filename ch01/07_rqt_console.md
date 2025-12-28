
参考:
- [Using rqt_console to view logs](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html)

rqt_console 是一个用于在 ROS2 中查看日志消息的图形用户界面工具。通常，日志消息会显示在终端中。使用 rqt_console，用户可以收集一段时间内的日志消息，以更清晰、更有条理的方式查看它们，进行筛选，保存它们，甚至可以重新加载已保存的文件以便在其他时间查看。

目标:
- 能够通过`ros2 run rqt_console rqt_console`命令运行 rqt_console 窗口。

日志级别一共 5 个级别:
```s
Fatal
Error
Warn
Info
Debug
```
默认级别为 Info 。

可以在运行节点时指定日志级别:
```s
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
```

执行下面的命令(海龟撞墙)在 rqt_console 中查看日志信息的输出:
```s
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
```
