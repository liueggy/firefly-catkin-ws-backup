# 待办事项

## Tailscale 远程 ROSBridge（4G）

**状态：Wi-Fi 链路已部署并验证；4G 链路因 SIM 未识别暂未实测（2026-07-15）**

目标：小车只使用 SIM 卡/4G 上网时，通过 Tailscale 加入受控的虚拟网络；Qt 端选择“ Tailscale（ROSBridge）”连接预设后，仍通过现有 ROSBridge WebSocket 协议访问车端 TCP 9090。Tailscale 只承担网络可达性，不新增 ROS 中转节点，不改变 mission、话题或 JSON 契约。

### 实车前置检查

1. 确认 4G 网卡已联网，DNS、HTTPS 和 UDP 出站可用；记录 `ip addr`、`ip route` 和运营商分配地址。
2. 确认板端存在 `/dev/net/tun`，系统架构为 `aarch64`，并核对 Ubuntu 20.04/内核环境可运行当前 Tailscale ARM64 客户端。
3. 检查 4G 网卡是否也使用 `100.64.0.0/10`；若与 Tailscale IPv4 地址段冲突，先制定 IPv6/MagicDNS 或路由规避方案。
4. 确认 `/rosbridge_websocket` 正常监听 TCP 9090，并保持现有心跳、重连、`cmd_vel` 仲裁和底盘超时停车策略。

### 2026-07-15 部署记录

- 板端为 Ubuntu 20.04/aarch64，已安装官方 Tailscale `1.98.9`，节点名为
  `eggy-001`；`tailscaled` 已启用开机启动。
- 当前 4.19.232 内核没有 `/dev/net/tun`，且策略路由不可用，因此不能创建标准
  `tailscale0`。已采用官方用户态网络模式，并使用持久化 Tailscale Serve 将尾网内
  TCP 9090 转发至本机 `127.0.0.1:9090`；未配置出口节点、子网路由、Tailscale DNS
  或 Tailscale SSH。
- Windows 到小车在 Wi-Fi 出口下为直连，`tailscale ping` RTT 约 12 ms；TCP 9090、
  WebSocket 握手及 `/rosapi/get_time` 服务调用均通过。重启 `tailscaled` 后登录状态和
  9090 转发保持正常。
- 4G 模组服务能够识别 Quectel EC200A 和 USB 网卡，但模组返回 `+CME ERROR: 10`
  （未检测到 SIM），运营商状态为 `+COPS: 0`，4G 网卡因此没有地址和默认路由。需要
  断电后检查 SIM 卡方向、卡座接触和套餐状态，再进行“仅 4G、关闭 Wi-Fi”切换测试。
- 用户态 TCP 转发不会改变 ROSBridge 消息、mission、话题或 JSON 契约；但地图、雷达、
  costmap 与相机并发时的吞吐和 CPU 占用仍需在 4G 恢复后测量。
- 当前使用个人账号交互授权完成首次联调。量产前仍需改为一次性预批准的
  `tag:robot` 认证，并在 Tailnet Grants/ACL 中仅授权指定操作员访问 TCP 9090。

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

## RK3568 NPU、相机与 Kimi 图像链优化

**状态：待取得实车后建立基线，再按 P0/P1/P2 逐项开发**

目标：在不牺牲水表/压力表检出率和读数质量的前提下，修正当前 RKNN 推理契约、图像几何和同帧关联问题，减少无效解码、缩放、拷贝与 JPEG 重编码，并确保原始流、带框流和 Kimi 分析流职责清晰、没有重复发布或反馈环路。

### 当前源码基线（尚未按本次版本在实车复测）

- `eggy_system.launch` 请求 Orbbec UVC 相机以 1280×720、10 fps、MJPG 工作，并将相机 JPEG 字节直通发布到 `/camera/front/image_source/compressed`；实际协商出的 V4L2 分辨率、帧率、格式和相机内部 JPEG 质量必须在实车读取，不能只相信请求参数。
- `meter_rknn_detect_cpp` 解码源 JPEG，将 1280×720 直接拉伸为 960×960 RGB/NHWC/UINT8 送入 RK3568 NPU；随后在 CPU 执行 DFL、Sigmoid、NMS，在原尺寸图像上画框并以 JPEG q85 发布 `/camera/front/image/compressed`，检测 JSON 发布到 `/meter/detection`。
- 默认主链路中，原始源流由相机独占发布，带框流由 RKNN 节点独占发布；`use_meter_rknn=false` 且显式启用兼容 relay 时才转发源流。默认不发布 `/camera/front/image_raw`，当前设计不需要额外的视频中转节点。
- 主系统中的 Kimi bridge 订阅 `/camera/front/image_source/compressed`，缓存“最新一帧”并按请求上传原始 JPEG；服务端默认最长边 1280、重新编码为 JPEG q92 后再发送给 Kimi。对于 1280×720 输入通常不缩放，但仍发生一次解码和有损重编码。
- `inspection_mission.launch` 的 Kimi 图像默认值仍是 `/camera/front/image/compressed`，可能把带框且已 q85 重编码的图像再次交给 Kimi，需与主 profile 统一。
- `inspection_reading_auto_node.py` 若被额外手工启动，也可能发布 `/kimi_inspection/result`；实车启动图中必须确认该结果话题只有一个权威发布者。

### 实车基线采集（任何优化前必须完成）

- [ ] 记录板端 Git SHA、RKNN 模型 SHA256、模型契约清单、RKNN SDK/runtime/driver 版本和实际启动参数，保留可回滚版本。
- [ ] 使用 `v4l2-ctl --all`、`--list-formats-ext` 和运行日志记录相机实际协商的宽高、帧率、像素格式、曝光及可用 MJPEG 模式。
- [ ] 对 `/camera/front/image_source/compressed`、`/camera/front/image/compressed`、`/meter/detection` 执行 `rostopic info/hz/bw`，记录发布者、订阅者、频率、带宽和是否存在重复发布者。
- [ ] 采集固定测试集，覆盖水表/压力表的远近、倾斜、遮挡、反光、明暗和运动模糊；保存原始 JPEG、检测框、Kimi 请求图和最终读数用于前后对比。
- [ ] 分段测量 JPEG 解码、预处理、`inputs_set`、NPU run、`outputs_get`、后处理、画框、JPEG 编码及端到端耗时，同时记录 CPU、内存、NPU 利用率和丢帧。
- [ ] 建立当前模型的 Precision/Recall、误检漏检、Kimi 读数准确率及失败样例基线；没有基线时不调整阈值或量化方案。

### P0：正确性、图像质量与话题契约

- [x] `rknn_run` 非零返回时立即停止本帧处理并发布明确错误，禁止继续获取输出或发布可能陈旧的检测结果。（2026-07-15，实机编译与正常推理链验证通过）
- [x] 对每次关键 `rknn_query` 检查返回值；启动时强校验运行时输入为 960×960、RGB、UINT8、NHWC，量化模型侧输入为 INT8/NHWC，输出为 3 个 NCHW raw-head，通道数 66，空间尺寸为 120×120、60×60、30×30。任何模型契约不匹配都应 fail-fast。（2026-07-15，实机 `rknn_query` 元数据验证通过）
- [x] 将 1280×720 到 960×960 的直接拉伸改为保持宽高比的 letterbox，并保存缩放与 padding 参数；检测框必须精确逆映射到原图坐标。（2026-07-15，横屏/竖屏/奇数尺寸/边界几何测试及实机 1280×720→960×540、`pad_top=210`、原尺寸 overlay 验证通过；固定仪表样本的召回率对比仍归入基线任务）
- [ ] 检测结果携带源图 `stamp`、`frame_id` 和稳定的帧标识；巡检 runner 必须让 Kimi 读取触发该检测的同一帧，不能取请求到达时的任意“最新帧”。
- [ ] Kimi 默认使用无叠加框的原始源图，并同时提交带适量边距的目标 ROI 与必要的全景上下文；禁止把 UI 调试用带框图作为读数主输入。
- [ ] 统一 `eggy_system.launch`、`inspection_profile.launch` 和 `inspection_mission.launch` 的 Kimi 源话题为 `/camera/front/image_source/compressed`。
- [ ] 当输入 JPEG 最长边不超过 Kimi 限制且无需裁剪时，复用原始 JPEG 字节，避免无意义的 q92 再编码；需要 ROI/缩放时再做一次可配置的高质量编码。
- [x] 启动时拒绝 RKNN 图像输入话题与 overlay 输出话题相同，增加 launch 契约测试，防止自订阅反馈环路和原始帧/带框帧混流。（2026-07-15，默认 launch 测试及实机同话题拒绝启动验证通过）
- [ ] 确认 `/kimi_inspection/result` 只有一个权威发布者；旧自动读数节点若保留，必须改名、禁用或由统一 launch 明确互斥。

### P1：减少无效工作并补齐可观测性

- [ ] 校验并实际调用现有 `keep_detection()` 几何过滤；若实测证明无价值则删除，避免保留未生效的“伪过滤”代码。
- [ ] 使用固定测试集和 PR 曲线重新标定置信度、面积、边界和 NMS 阈值；NMS 前增加可配置 top-K，避免对大量低价值候选做排序和抑制。
- [ ] overlay 无订阅者时跳过画框与 JPEG q85 编码；原始检测 JSON 仍应正常发布。
- [ ] 使用 RAII 确保 RKNN outputs 在所有成功和异常路径尽早释放，避免错误分支泄漏或长期占用 runtime buffer。
- [ ] 为 decode、preprocess、NPU、postprocess、overlay encode、Kimi prepare/upload/response 和完整任务增加统一时延指标，并记录源帧年龄、分辨率、JPEG 质量、模型版本和 request_id。
- [ ] 相机节点启动后读取并记录 V4L2 实际协商值；OpenCV fallback 也必须读取 `CAP_PROP_*` 实际值，避免日志只显示请求值。
- [ ] 复核 `meter_visual_servo`/巡检对准参数是否仍沿用 640×480 标定；所有像素阈值和面积比例需针对 1280×720 实车重新标定或改成归一化量。
- [ ] 采用单槽 latest-frame 工作队列和单一 RKNN context/推理线程；新帧覆盖旧待处理帧，禁止积压导致巡检使用过期画面。

### P2：NPU/内存优化与工程化

- [ ] 复用 OpenCV Mat、输入缓冲、候选框 vector 和 RKNN input/output 描述数组，减少每帧堆分配与内存拷贝。
- [ ] 在保持检测精度的条件下评估 `want_float=0` 的量化输出和 CPU 端反量化；记录相对 FP32 输出的带宽、延迟和精度差异后再决定是否启用。
- [ ] 评估 RK3568 RGA 完成 resize/letterbox/颜色转换，以及 RKNN tensor memory/零拷贝输入；只有实测端到端收益明确且回滚路径完整时才替换 OpenCV 路径。
- [ ] 通过 CMake `find_path`/`find_library` 检测 RKNN SDK，不把板端头文件和库路径散落硬编码。
- [ ] 为模型增加 SHA256、输入输出契约 manifest、类别名称、量化方式和兼容 SDK/runtime 版本；启动日志必须输出并校验这些信息。
- [ ] 格式化当前单行密集的 C++ 识别节点，并为 letterbox/逆映射、DFL、Sigmoid、NMS、话题相等拒绝和错误分支添加离线测试；重构前先锁定现有行为。

### 完成条件

- `rknn_run` 或模型契约异常不会发布陈旧检测，节点会给出可定位的错误阶段和版本信息。
- letterbox 与逆映射通过离线几何测试，并在实车固定测试集上不降低检测召回率。
- RKNN 检测、ROI 和 Kimi 分析严格关联同一源帧；Kimi 收到无框、清晰、可追踪分辨率与压缩参数的图像。
- 默认启动图中原始流、带框流和 Kimi 结果均只有一个权威发布者，不存在图像反馈环路或无必要的 Python 视频搬运。
- 优化前后有可复现的检测/Kimi 准确率、时延、CPU、内存、NPU 和 ROS 带宽对比；性能提升不得以明显降低识别质量为代价。
- 每一阶段都有独立提交、实车验收记录和明确回滚命令；P0 未通过前不进入 P2 硬件加速改造。

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
