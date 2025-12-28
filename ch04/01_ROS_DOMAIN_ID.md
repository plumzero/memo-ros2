
参考:
- [ROS_DOMAIN_ID](https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html)
- [Configuring environment](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
- [Statically configure a Firewall to let OMG DDS Traffic through](https://community.rti.com/content/forum-topic/statically-configure-firewall-let-omg-dds-traffic-through)

ROS 2 默认的通信中间件是 DDS。在 DDS 中，不同逻辑网络共享物理网络的主要机制称为域 ID。同一域中的 ROS 2 节点可以自由地相互发现和发送消息，而不同域中的 ROS 2 节点则不能。所有 ROS 2 节点默认使用域 ID 0(环境变量 ROS_DOMAIN_ID 为空)。为了避免在同一网络上运行 ROS 2 的不同计算机组之间相互干扰，应该为每个计算机组设置不同的域 ID。

接下来进行一组测试。

默认域 ID 下在两个终端中分别执行如下两条命令:
```s
ros2 run demo_nodes_cpp listener
```

```s
ros2 run demo_nodes_cpp talker
```
可以看到，是能够正常发送和接收的。

现在将 talker 停掉，并在此终端上通过`export ROS_DOMAIN_ID=10`，将域 ID 修改为 10，再执行启动 talker 的命令，将发现 listener 终端将不再有输出。

之后将 listener 停掉，在此终端上同样将域 ID 修改为 10，执行启动 listener 的命令，会发现 listener 终端又可以接收到消息并打印输出了。

不同的域 ID 会使用不同的端口，可以在[域名 ID 到 UDP 端口计算器](https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html#domain-id-to-udp-port-calculator)这里计算出域名 ID 和端口之间的关系。