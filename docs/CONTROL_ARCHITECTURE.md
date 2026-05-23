# Eggy Robot Control Architecture

This repository contains the Firefly ROS workspace for the robot.

## Main runtime entry points

- `src/eggy_bringup/launch/robot.launch`: main robot bringup.
- `src/eggy_base_driver`: STM32 chassis driver, odometry, serial protocol.
- `src/rplidar_ros`: RPLIDAR driver.
- `src/eggy_external_imu`: external IMU serial driver.
- `src/eggy_bringup/scripts/eggy_external_imu_odom_fuser.py`: fuses wheel odom with external IMU yaw and publishes `/odom` + TF.
- `src/eggy_bringup/scripts/eggy_simple_navigator.py`: simple click-to-go navigator.
- `src/eggy_bringup/config/nav`: move_base / costmap / local planner configs.

## Cloud/web control agent

The deployed robot-web bridge is tracked here as:

- `src/eggy_bringup/scripts/eggy_robot_agent.py`
- deployed path: `/usr/local/bin/eggy-robot-agent`
- service file: `src/eggy_bringup/systemd/eggy-robot-agent.service`

The agent connects to the cloud relay websocket, publishes robot state/map preview/lidar points, and forwards web commands into ROS topics.

## Sync policy

After modifying this repo remotely, pull on the robot:

```bash
cd /root/catkin_ws
git pull --ff-only
catkin_make
systemctl restart eggy-robot-agent
# restart ROS bringup as needed
```

Avoid editing deployed files directly without also updating this repo. If deployed files are edited directly, copy them back into the repo and commit.
