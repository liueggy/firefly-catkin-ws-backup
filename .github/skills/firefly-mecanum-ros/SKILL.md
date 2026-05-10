---
name: firefly-mecanum-ros
user-invocable: true
description: "Use when working on the Firefly mecanum-wheel ROS Noetic robot project, including STM32 serial protocol, eggy_base_driver, bringup, URDF/TF, IMU, odom, battery, and launch/debug workflow."
---

# Firefly 麦克纳姆轮 ROS 开发 Skill

适用场景：对 Firefly 控制主板上的 ROS Noetic 工程进行开发、联调、排障和文档补充，尤其是基于 C++ roscpp 的底盘串口驱动、IMU、里程计、启动流程和后续传感器扩展。

## 已知项目背景

- 主机：Firefly 控制主板
- 系统：Ubuntu 20.04 / ROS Noetic
- 工作空间：/root/catkin_ws
- 底盘控制板：STM32F407ZG，FreeRTOS V9.0.0
- STM32 固件仓库：ros_stm32f4
- 串口设备：/dev/ttyACM0
- 波特率：115200

## 当前 ROS 包

- eggy_base_driver：STM32 串口底盘驱动
- eggy_robot_description：URDF 和 TF
- eggy_bringup：整车启动入口
- eggy_future_sensors：激光雷达、温度、GPS 预留
- eggy_gui_tools：图形化调试界面预留

## 当前驱动实现

- 主驱动语言：C++
- ROS 客户端库：roscpp
- 主节点：stm32_base_driver_node
- 入口文件：src/stm32_base_driver_node.cpp
- 业务实现：src/base_driver.cpp
- 串口与协议封装：src/serial_port.cpp、src/stm32_protocol.cpp
- 头文件：include/eggy_base_driver/
- Python 脚本：scripts/stm32_base_driver.py，保留为参考或调试用，不是当前主线驱动

## 底盘协议速记

### 下行控制帧

- 长度：11 字节
- 帧头：0x7B
- 帧尾：0x7D
- 模式：Byte1 = 0x00
- 校验：前 9 字节 XOR
- 速度单位：X/Y 为 mm/s，Z 为 rad/s × 1000

### 上行状态帧

- 长度：24 字节
- 帧头：0x7B
- 帧尾：0x7D
- 包含：停止标志、X/Y/Z 速度、三轴加速度、三轴陀螺仪、电池电压
- 校验：前 22 字节 XOR

## 开发时优先遵守的约定

- 默认从 /root/catkin_ws 出发定位问题。
- 先确认串口设备是 /dev/ttyACM0，再检查配置文件中的 port。
- /cmd_vel 是底盘控制入口，修改运动逻辑时优先检查该话题的订阅、限幅和超时停车。
- 发送控制帧时要保持周期发送，建议 20Hz 到 50Hz，避免只发一次就停止。
- 重点关注以下输出话题：/imu/data_raw、/odom、/battery/voltage、/base/flag_stop、/diagnostics、/tf。
- 修改 C++ 代码后以 catkin_make 重新编译，并在当前终端重新 source /root/catkin_ws/devel/setup.bash。

## 常用验证命令

```bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
roslaunch eggy_bringup robot.launch port:=/dev/ttyACM0
```

```bash
rostopic list
rostopic echo /battery/voltage
rostopic echo /imu/data_raw
rostopic hz /imu/data_raw
rqt_graph
rqt_plot /imu/data_raw/angular_velocity/z /battery/voltage
```

## 运动测试模板

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"
```

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{}"
```

## 代码修改时的检查点

- 串口参数是否仍匹配当前设备名和波特率。
- 控制帧打包和校验是否仍符合 STM32 协议。
- 里程计和 IMU 的单位换算是否一致。
- URDF 是否还存在 root link inertia 警告。
- 安全限幅是否保留在合理范围内。
- 新增传感器是否需要在 bringup 中同步启动。
- C++ 节点名、launch 文件和 CMakeLists.txt 是否保持一致。
- Python 旧脚本是否只作为辅助，不再被 launch 误启动。

## 适合这个 skill 的任务

- 修复 eggy_base_driver 的串口读写、协议解析、速度控制和超时停车。
- 修复或扩展 eggy_base_driver 的 C++ roscpp 主驱动、串口读写、协议解析、速度控制和超时停车。
- 调整 URDF、TF、里程计和 IMU 发布逻辑。
- 增加激光雷达、温度传感器或 GPS 的 ROS 接入。
- 编写或更新 launch、config、README 和项目推进记录。
- 为实车联调准备更明确的测试步骤和安全限制。
