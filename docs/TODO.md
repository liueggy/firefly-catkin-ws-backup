# 待办事项

## Tailscale 远程 ROSBridge（4G）

**状态：待取得实车后安装与联调**

目标：小车只使用 SIM 卡/4G 上网时，通过 Tailscale 加入受控的虚拟网络；Qt 端选择“ Tailscale（ROSBridge）”连接预设后，仍通过现有 ROSBridge WebSocket 协议访问车端 TCP 9090。Tailscale 只承担网络可达性，不新增 ROS 中转节点，不改变 mission、话题或 JSON 契约。

### 实车前置检查

1. 确认 4G 网卡已联网，DNS、HTTPS 和 UDP 出站可用；记录 `ip addr`、`ip route` 和运营商分配地址。
2. 确认板端存在 `/dev/net/tun`，系统架构为 `aarch64`，并核对 Ubuntu 20.04/内核环境可运行当前 Tailscale ARM64 客户端。
3. 检查 4G 网卡是否也使用 `100.64.0.0/10`；若与 Tailscale IPv4 地址段冲突，先制定 IPv6/MagicDNS 或路由规避方案。
4. 确认 `/rosbridge_websocket` 正常监听 TCP 9090，并保持现有心跳、重连、`cmd_vel` 仲裁和底盘超时停车策略。

### 部署原则

1. 使用一次性、预批准、带 `tag:robot` 的认证密钥注册 `eggy-001`；密钥不得写入仓库、日志、launch 或 systemd 文件。
2. `tailscaled` 设置为开机启动并在 4G 网络恢复后自动重连；不配置 exit node，不发布无关子网路由。
3. Tailnet 权限仅允许指定操作员/设备访问 `tag:robot` 的 `tcp:9090`；如需维护，单独授权 `tcp:22`。
4. 不在 4G 公网做 9090 端口映射；主机防火墙只允许 `tailscale0` 方向访问 ROSBridge。
5. Qt 保存 Tailscale 地址或 MagicDNS 名称，不把 `100.x` 地址硬编码进板端源码。

### 联调与验收

1. Windows 与小车登录同一 Tailnet，使用 `tailscale ping eggy-001` 确认直连或 DERP 状态。
2. Windows 验证 `Test-NetConnection <tailscale-address> -Port 9090`，Qt 使用“Tailscale（ROSBridge）”预设连接。
3. 验证连接、断线、4G 切网和 `tailscaled` 重启后 Qt 状态与自动重连；确认断线不会留下持续速度指令。
4. 分别验证普通导航和 AI 巡检 mission 的请求、状态、结果与取消链路，确保网络方式不改变业务行为。
5. 测量 `/map`、`/scan`、costmap、路径和相机同时开启时的 RTT、丢包与流量；确认 DERP 回退时仍可操作。
6. 增加“4G 低带宽”参数验收：相机按需开启并降低帧率/质量，高频显示消息采用节流或 latest-value 策略。
7. 记录最终 Tailscale 客户端版本、节点名、ACL/Grants 策略、直连/DERP 结果和回滚步骤，但不记录认证密钥。

### 完成条件

- Qt 能在局域网 ROSBridge 与 Tailscale ROSBridge 两套预设间切换，并分别保留地址和端口。
- 小车在没有 Wi-Fi、只有 4G 的情况下可稳定接收 mission，并返回状态和结果。
- 4G/Tailscale/Qt 任一链路断开后，板端本地导航安全策略和速度超时停车仍然有效。
- TCP 9090 未暴露到公网，非授权 Tailnet 身份无法访问小车。
- 已形成实车联调记录，且可一键停用 Tailscale、恢复原局域网 ROSBridge。

## 危险状态远程通知（4G）

**状态：待实车确认后开发**

目标：小车检测到严重危险状态时，先在板端执行安全停车，再通过 4G 向指定联系人发出通知；语音拨号作为可选升级能力。

### 已有基础

- `/inspection/alert` 已发布巡检异常事件。
- `eggy_robot_agent.py` 已生成里程计断流、雷达断流、低电量、地图失效和前方近障碍等告警。
- 4G 网络状态节点已通过 `mmcli` 获取运营商与联网状态。

### 实施顺序

1. 实车确认 4G 模块型号、AT 控制串口、SIM 卡类型，以及 VoLTE/语音通话能力。
2. 新增独立告警通知节点：订阅既有告警源，按告警等级去重、限频和记录。
3. 对紧急告警先执行本地安全停车，再上报云端；网络通知不能替代实体急停。
4. 先接入数据网络可用的通知方式：云端推送、短信或云语音呼叫。
5. 若模块和 SIM 均支持语音，接入 AT 拨号：按联系人顺序呼叫、超时挂断、失败重试。
6. 在 Qt 端展示告警级别、通知发送状态及最近一次通知结果。

### 验收条件

- 严重告警发生后，底盘在本地进入安全停止状态。
- 同一告警在冷却时间内不重复骚扰联系人。
- 通知包含机器人编号、时间、告警类型和当前任务/点位。
- 无语音能力时自动降级为云端推送或短信，不影响安全停车。
- 断网、拨号失败和短信失败均记录本地日志，并可在 Qt 端查看。
