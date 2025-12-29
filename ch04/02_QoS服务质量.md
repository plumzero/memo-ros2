
参考:
- [About Quality of Service settings](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html)
- [rosbag2: Overriding QoS Policies](https://docs.ros.org/en/foxy/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html)

ROS2 提供了丰富的服务质量(QoS)策略，允许您调整节点间的通信。通过合适的 QoS 策略，ROS2 可以像 TCP 一样可靠(reliable)，也可以像 UDP 一样尽力而为(best effort)，两者之间还有许多其他可能的状态。与主要仅支持 TCP 的 ROS1 不同，ROS2 受益于底层 DDS​​ 传输的灵活性，在丢包率较高的无线网络环境中，尽力而为的策略更为适用；在实时计算系统中，需要合适的 QoS 配置来满足截止时间要求，ROS2 也同样适用。

一系列 QoS 策略组合构成一个 QoS 配置文件。鉴于为特定场景选择合适的 QoS 策略的复杂性，ROS2 提供了一组预定义的 QoS 配置文件，用于常见的用例(例如传感器数据）。同时，开发者也可以灵活地控制 QoS 配置文件中的特定策略。

可以为发布者、订阅者、服务服务器和客户端指定 QoS 配置文件。QoS 配置文件可以独立应用于上述每个实体实例，但如果使用不同的配置文件，则可能存在不兼容的情况，从而导致消息无法送达。

### QoS 策略

基本 QoS 配置文件目前包含以下策略的设置:

- history(历史记录)
  - keep last: 最多只存储 N 个样本，可通过 queue depth 选项进行配置。
  - keep all: 存储所有样本，但须遵守底层中间件配置的资源限制。
- depth(深度)
  - queue size: 仅当 history 策略设置为"keep last"时才有效。
- reliability(可靠性)
  - best effort: 尝试递送样品，但如果网络不稳定，可能会丢失样品。
  - reliable: 保证样品送达，可多次尝试。
- durability(持久性)
  - transient local: 发布者负责为"late-joining”的订阅者保存样本。
  - volatile: 不尝试保存样品。
- deadline(截止时间)
  - duration: 向同一主题发布后续消息之间预期的最大时间间隔。
- lifespan(生命周期):
  - duration: 消息发布到接收之间的最长时间，在此时间内消息不会被视为过期或失效(过期消息会被静默丢弃，实际上永远不会被接收)。
- liveliness(活跃度)
  - automatic: 当节点上的任何发布者发布消息时，系统将认为该节点的所有发布者在另一个"lease duration"内仍然存活。
  - manual by topic: 如果系统手动断言发布者仍然存活(通过调用发布者 API)，则系统会将发布者视为在另一个"lease duration"内仍然存活。
- lease duration(租约期限)
  - duration: 发布者必须表明其仍然活跃的最长时间，之后系统会认为它已失去活跃状态(失去活跃状态可能表明出现故障)。

对于所有不是 duration 类型的策略，都存在"system default"选项，该选项使用底层中间件的默认值。对于所有是 duration 类型的策略，也存在"default"选项，该选项表示持续时间未指定，底层中间件通常会将其解释为无限长的持续时间。

### 与 ROS1 的比较

ROS2 中的 history 和 depth 策略结合起来，提供了类似于 ROS1 中队列大小的功能。

ROS2 中的 reliability 策略类似于使用 UDPROS(仅在 ROS1 中roscpp)实现"best effort"，或使用 TCPROS(ROS1 默认)实现"reliable"。但需要注意的是，即使是 ROS2 中的 reliability 策略也是使用 UDP 实现的，这使得在适当情况下可以进行多播。

durability 策略的"transient local"结合任意 depth，可提供类似于"latching"发布者的功能。ROS2 中的其余策略与 ROS1 中的任何策略都截然不同，这意味着 ROS2 在这方面比 ROS1 功能更丰富。未来，ROS2 中可能会提供更多 QoS 策略。

### QoS 配置文件

QoS 配置文件使开发人员能够专注于应用程序本身，而无需担心所有可能的 QoS 设置。QoS 配置文件定义了一组策略，这些策略预期能够针对特定用例协同工作。

当前定义的 QoS 配置文件有:

1.发布和订阅的默认 QoS 设置

为了使从 ROS1 到 ROS2 的过渡更加顺畅，采用类似的网络行为是可取的。默认情况下，ROS2 中的发布者和订阅者使用"keep last"的历史记录机制(queue size 为 10）、"reliable"的可靠性机制、"volatile"的持久性机制以及"system default"的活跃度机制。截止时间、生命周期和租约期限也都设置为"default"。

2.服务(services)

与发布和订阅类似，服务也需要具备"reliable"。对于服务而言，使用"volatile"持久性尤为重要，否则服务服务器重启后可能会收到过时的请求。虽然客户端不会收到重复的响应，但服务器却无法避免收到过时请求所带来的副作用。

3.传感器数据

对于传感器数据而言，在大多数情况下，及时接收读数比确保所有读数都到达更重要。也就是说，开发人员希望在数据采集完成后立即获得最新样本，即使可能会丢失一些数据也在所不惜。因此，传感器数据配置文件采用"best effort"的可靠性策略和较小的 queue size。

4.参数

ROS2 中的参数基于服务(services)，因此具有类似的特性。不同之处在于，参数使用更大的 queue depth，这样即使参数客户端无法连接到参数服务服务器，请求也不会丢失。

5.系统默认设置(system default)

这将使用 RMW 实现的所有策略的默认值。不同的 RMW 实现可能具有不同的默认值。

点击[此处](https://github.com/ros2/rmw/blob/foxy/rmw/include/rmw/qos_profiles.h)查看上述配置文件的具体策略。这些配置文件中的设置可能会根据社区反馈进行进一步调整。

### QoS 兼容性

> 本段内容涉及发布者和订阅者，但同样适用于服务服务器和客户端。

QoS 配置文件可以分别针对发布者和订阅者进行配置。只有当发布者和订阅者的 QoS 配置文件兼容时，才会建立二者之间的连接。

QoS 配置文件兼容性基于"Request vs Offered"模型确定。订阅者请求的 QoS 配置文件是其愿意接受的"minimum quality"标准，而发布者提供的 QoS 配置文件是其能够提供的最高质量标准。只有当请求的 QoS 配置文件中的每一项策略都不比提供的 QoS 配置文件中的策略更严格时，连接才会建立。即使多个订阅者请求的 QoS 配置文件不同，它们也可以同时连接到同一个发布者。发布者和订阅者之间的兼容性不受其他发布者和订阅者的影响。

下表显示了不同策略设置的兼容性及结果:

reliability 策略的兼容性:

| 发布者       | 订阅者       | 兼容性 |
|:------------|:------------|:------|
| best effort | best effort | yes   |
| best effort | reliable    | no    |
| reliable    | best effort | yes   |
| reliable    | reliable    | yes   |

durability 策略的兼容性:

| 发布者           | 订阅者           | 兼容性 |
|:----------------|:----------------|:------|
| volatile        | volatile        | Yes   |
| volatile        | transient local | No    |
| transient local | volatile        | Yes   |
| transient local | transient local | Yes   |

deadline 策略的兼容性(假设 x 和 y 是任意有效的 duration 值):

| 发布者   | 订阅者   | 兼容性 |
|:--------|:--------|:------|
| default | default | Yes   |
| default | x       | No    |
| x       | default | Yes   |
| x       | x       | Yes   |
| x       | y(y>x)  | Yes   |
| x       | y(y<x)  | No    |

liveliness 策略的兼容性:

| 发布者           | 订阅者           | 兼容性 |
|:----------------|:----------------|:------|
| automatic       | automatic       | Yes   |
| automatic       | manual by topic | No    |
| manual by topic | automatic       | Yes   |
| manual by topic | manual by topic | Yes   |

lease duration 策略的兼容性(假设 x 和 y 是任意有效的 duration 值):

| 发布者   | 订阅者   | 兼容性 |
|:--------|:--------|:------|
| default | default | Yes   |
| default | x       | No    |
| x       | default | Yes   |
| x       | x       | Yes   |
| x       | y(y>x)  | Yes   |
| x       | y(y<x)  | No    |

要建立连接，所有影响兼容性的策略都必须兼容。例如，即使 Request 的 QoS 配置文件和 Offered 的 QoS 配置文件具有兼容的 reliability 策略，但如果它们的 durability 策略不兼容，则仍然无法建立连接。

### QoS 事件

某些 QoS 策略可能涉及相关事件。开发者可以为每个发布者和订阅者提供回调函数，这些函数由 QoS 事件触发，开发者可以根据需要处理这些事件，类似于处理主题上收到的消息的方式。

开发者可以订阅与发布者关联的以下 QoS 事件:
- 错过截止日期: 发布者未在服务质量策略规定的截止日期前发布消息。
- 失去活力(liveliness): 发布者未能表明其在 lease duration 期内的有效性。
- 提供的服务质量不兼容: 发布者遇到一个关于同一主题的订阅，该订阅请求的 QoS 配置文件无法满足发布者提供的 QoS 配置文件的要求，导致发布者与该订阅之间没有连接。

开发者可以订阅以下与订阅关联的 QoS 事件:
- 错过要求的截止日期: 订阅者未在服务质量策略规定的截止日期前收到消息。
- 活力变化: 订阅者注意到，订阅主题的一个或多个发布者未在 lease duration 期内表明其活跃状态。
- 请求的服务质量不兼容: 订阅者遇到同一主题的发布者，该发布者提供的 QoS 配置文件不符合所请求的 QoS 配置文件，导致订阅者与该发布者之间无法建立连接。
