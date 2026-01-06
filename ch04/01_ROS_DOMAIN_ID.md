
参考:
- [ROS_DOMAIN_ID](https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html)
- [Configuring environment](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
- [Statically configure a Firewall to let OMG DDS Traffic through](https://community.rti.com/content/forum-topic/statically-configure-firewall-let-omg-dds-traffic-through)

ROS2 默认的通信中间件是 DDS。在 DDS 中，不同逻辑网络共享物理网络的主要机制称为域 ID。同一域中的 ROS2 节点可以自由地相互发现和发送消息，而不同域中的 ROS2 节点则不能。所有 ROS2 节点默认使用域 ID 0(环境变量 ROS_DOMAIN_ID 为空)。为了避免在同一网络上运行 ROS2 的不同计算机组之间相互干扰，应该为每个计算机组设置不同的域 ID。

DDS 实现(例如 RTI Connext DDS)使用的端口(和组播地址)由[OMG DDS 互操作性线路协议规范(DDS-RTPS)](https://www.omg.org/spec/DDSI-RTPS/2.5/PDF)规定。默认情况下，每个域参与者(DomainParticipant)会打开 4 个 UDP/IP 端口: 其中两个是组播端口，同一 DomainId 下的所有域参与者共享；另外两个是单播端口，同一台计算机内的每个参与者的单播端口都不同。用于发现的组播 IP 地址默认为 239.255.0.1，这与 DDS-RTPS 规范一致。

> 一台计算机上最多可以创建 120 个 ROS 2 进程，超过此数量就会进入临时端口范围(同下一个域的端口产生重叠)。

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
