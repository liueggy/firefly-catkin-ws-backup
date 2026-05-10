# eggy_gui_tools

后续图形化调试界面预留包。

建议分阶段开发：

1. 当前阶段：使用 `rqt_graph`、`rqt_plot`、`rostopic echo` 可视化 `/imu/data_raw`、`/battery/voltage`、`/odom`。
2. 第二阶段：开发 rqt 插件，集中显示：电压、串口状态、IMU 曲线、cmd_vel 控制按钮。
3. 第三阶段：加入激光雷达地图、温度、GPS 状态页。

推荐调试命令：

```bash
rqt_graph
rqt_plot /imu/data_raw/angular_velocity/z /battery/voltage
```
