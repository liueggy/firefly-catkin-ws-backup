# 自动建图协议与安全闭环

自动建图仅在 `mapping` profile 启用。Qt 继续使用统一命令中心，板端命令中心负责校验
profile 并转发给自动建图管理器。

## 命令

- Qt：`/eggy/command/request`，命令为
  `auto_mapping_start/pause/resume/stop/status`。
- 板端内部：`/eggy/auto_mapping/request`，JSON 包含
  `schema_version/request_id/command/options`。
- `start` 可配置 `max_duration_sec`、`max_linear_speed`、`return_home`、
  `save_draft_on_abort` 和 `min_frontier_cells`。
- 暂停、继续和停止必须携带当前任务的同一 `request_id`。

## 状态与结果

- `/eggy/auto_mapping/status`：2 Hz，latched；阶段包括 `preflight/planning/navigating/
  observing/returning/final_scan/saving/paused` 及终态。
- `/eggy/auto_mapping/result`：一次性最终结果，包含保存的 `map_id` 和路径。
- `/auto_explore/status`：兼容旧客户端的同内容状态。
- `/eggy/command/status.auto_mapping`：Qt 面板所用的聚合状态。

## 速度与停车

`move_base` 只发布到 `/cmd_vel/mapping_raw`。`auto_mapping_safety` 同时检查雷达、里程计、
管理器心跳、急停和 `/base/flag_stop`，通过后发布 `/cmd_vel/mapping`，再由统一仲裁器输出
最终 `/cmd_vel`。任何数据过期都按失效安全策略输出零速度；人工遥控优先级更高并使任务暂停。

## 地图保存

完成时可先返回起点，再执行一圈低速最终扫描。地图先写入 staging 目录，确认 YAML/PGM
存在后原子提交到 `/root/catkin_ws/maps/library/<map_id>/`，同时生成 `metadata.json`。
用户主动停止默认保存 `draft`；急停或传感器异常只停车并报告原因，不自动恢复。
