# eggy_future_sensors

该包用于预留后续传感器接入，不在当前阶段启动真实驱动。

计划话题约定：

| 传感器 | 推荐话题 | ROS 消息 |
|---|---|---|
| 激光雷达 | `/scan` | `sensor_msgs/LaserScan` |
| 温度传感器 | `/temperature` | `sensor_msgs/Temperature` |
| GPS | `/fix` | `sensor_msgs/NavSatFix` |
| GPS 速度 | `/fix_velocity` | `geometry_msgs/TwistStamped` |

坐标系预留：

- `laser_link`
- `gps_link`
- 温度传感器如需空间位置，后续增加 `temperature_link`
