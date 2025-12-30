
### DomainParticipant

- 标准 DDS 实体: 是 DDS API 的核心入口点，符合 OMG DDS 标准规范
- 应用层接口: 为应用程序提供创建 Publisher、Subscriber、Topic、DataWriter、DataReader 等实体的工厂
- 域(Domain)概念: 属于特定的 DDS 域(通过 DomainId 标识)，同一域内的实体才能互相发现和通信
- 角色: 逻辑实体，管理数据读写、QoS 策略、主题匹配等高层功能

### RTPSParticipant

- 传输层实体: 实现 RTPS(Real-Time Publish-Subscribe)协议的端点，是 FastDDS 的内部实现细节
- 网络通信实体: 负责实际的网络通信，包括 UDP/TCP 连接、端口管理、发现协议执行等
- 物理网络代表: 每个 RTPSParticipant 对应一个网络节点，有独立的 GUID（全局唯一标识符）
- 角色: 处理发现、连接建立、消息序列化/反序列化、可靠性传输等底层协议细节

### 关键区别

| 方面 | DomainParticipant | RTPSParticipant |
|------|-------------------|-----------------|
| 标准性 | DDS 标准定义 | RTPS 协议实现 |
| 抽象层次 | 应用层/逻辑层 | 传输层/网络层 |
| 主要职责 | 数据模型、QoS、主题管理 | 网络通信、发现、连接管理 |
| 可见性 | 对应用程序可见 | 通常对应用程序透明 |
| 数量关系 | 1个 DomainParticipant | 对应 1个或多个 RTPSParticipant |

### 关系说明

在 FastDDS 中:
- 1.创建 DomainParticipant 时，内部会自动创建至少一个 RTPSParticipant。
- 2.当需要跨网络接口通信时(如同时使用以太网和共享内存)，一个 DomainParticipant 可能对应多个 RTPSParticipant
- 3.RTPSParticipant 是 DomainParticipant 的网络化身，负责将逻辑通信需求转换为实际网络操作

### 示例代码视角

```cpp
// 应用程序看到的 DDS 接口
DomainParticipant* participant = 
    DomainParticipantFactory::get_instance()->create_participant(domain_id);

// 内部实现：每个 DomainParticipant 包含 RTPSParticipant
// RTPSParticipant* rtps_participant = participant->impl_->rtps_participant_;
```

### 使用建议

- 应用程序开发者: 主要操作 DomainParticipant，不需要直接接触 RTPSParticipant
- 高级调优: 需要配置网络特性(如多网卡绑定、自定义端口)时，才需要通过 DomainParticipant 获取 RTPSParticipant 进行设置
