
参考:
- [What is DDS?](https://fast-dds.docs.eprosima.com/en/latest/fastdds/getting_started/definitions.html)

### DDS

数据分发服务(DDS)是一种以数据为中心的通信协议，用于分布式软件应用程序通信。它描述了通信应用程序编程接口(API)和通信语义，从而实现数据提供者和数据消费者之间的通信。

由于它是一种数据中心发布订阅(DCPS)模型，因此在其实现中定义了三个关键应用程序实体:
- 发布实体，用于定义信息生成对象及其属性
- 订阅实体，用于定义信息消费对象及其属性
- 配置实体，用于定义作为主题传输的信息类型，并创建具有服务质量(QoS)属性的发布者和订阅者，以确保上述实体的正确性能。

### DCPS 概念模型

在 DCPS 模型中，定义了用于开发通信应用系统的四个基本要素。

- 发布者。它是负责创建和配置其所实现的 DataWriter 的 DCPS 实体。DataWriter 负责实际发布消息。每个 DataWriter 都会被分配一个主题，消息将发布在该主题下。查看更多[发布者信息](https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/publisher/publisher.html#dds-layer-publisher)。
- 订阅者。它是负责接收其订阅主题下发布的数据的 DCPS 实体。它为一个或多个 DataReader 对象提供服务，这些对象负责将新数据的可用性通知应用程序。查看更多[订阅者信息](https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/subscriber/subscriber.html#dds-layer-subscriber)。
- 主题(Topic)是连接出版物和订阅的实体，在 DDS 域中是唯一的。通过主题描述(TopicDescription)，它确保了发布数据类型和订阅数据类型的统一性。查看更多[主题信息](https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/topic/topic.html#dds-layer-topic)。
- 域。这是用于连接属于一个或多个应用程序的所有发布者和订阅者的概念，这些应用程序在不同的主题下交换数据。参与域的各个应用程序称为域参与者(DomainParticipant)。DDS 域由域 ID(domainId)标识。域参与者定义域 ID 以指定其所属的 DDS 域。具有不同 ID 的两个域参与者在网络中彼此不感知。因此，可以创建多个通信通道。这适用于涉及多个 DDS 应用程序的场景，这些应用程序各自的域参与者彼此通信，但这些应用程序不得相互干扰。域参与者充当其他 DCPS 实体的容器，充当发布者、订阅者和主题实体的工厂 ，并在域中提供管理服务。查看更多[域信息](https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domain.html#dds-layer-domain)。

![](img/dds_domain.svg)

### RTPS

实时发布/订阅(RTPS)协议是为支持 DDS 应用而开发的，它是一种基于尽力而为传输协议(例如 UDP/IP)的发布/订阅通信中间件，旨在支持单播和多播通信。

> Fast DDS 还支持 TCP 和共享内存(SHM)传输协议。

在 RTPS 的顶层，继承自 DDS 的域(Domain)定义了一个独立的通信平面。多个域可以同时独立存在。一个域包含任意数量的 RTPSParticipant，即能够发送和接收数据的元素。为此，RTPSParticipant 使用它们的端点(Endpoints):
- RTPSWriter: 能够发送数据的端点。
- RTPSReader: 能够接收数据的端点。

RTPSParticipant 可以有任意数量的写入端点和读取端点。

![](img/rtps_domain.svg)

通信围绕主题展开，主题定义并标记正在交换的数据。主题不属于任何特定参与者。参与者通过 RTPSWriter 修改主题下发布的数据，并通过 RTPSReader 接收与其订阅的主题关联的数据。通信单元称为"Change"，它表示对主题下写入的数据的更新。RTPSReader/RTPSWriter 将这些变更记录到其历史记录中，历史记录是一种用作最近变更缓存的数据结构。
