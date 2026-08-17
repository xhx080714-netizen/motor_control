# 耙沙捡球车设备 Topic 登记表

## 1. 文档说明

本文档记录真实设备及系统节点当前实际使用的 ROS 2 接口。

原则：

- 已实测确认的信息填写真实值。
- 尚未由对应负责人确认的信息填写 `TBD`。
- 当前暂停或无法集成的模块标记为 `BLOCKED`。
- 本文档记录实际系统状态，不替代 `icd_v0.1.md` 中的接口设计约束。

当前阶段：Day 5

---

## 2. 控制与底盘

| Item | Node | Topic | Type | Direction | frame_id | Frequency | Owner | Status |
|---|---|---|---|---|---|---|---|---|
| Keyboard command | `teleop_node` | `/teleop/cmd_vel` | `geometry_msgs/msg/Twist` | Publish | N/A | 20 Hz | B | PASS |
| Safety input | `safety_controller` | `/teleop/cmd_vel` | `geometry_msgs/msg/Twist` | Subscribe | N/A | 20 Hz input | B | PASS |
| Safety event | `safety_controller` | `/safety/event` | `sand_rake_interfaces/msg/SafetyEvent` | Subscribe | N/A | Event-driven | B | PASS |
| Safe velocity output | `safety_controller` | `/safety/cmd_vel` | `geometry_msgs/msg/Twist` | Publish | N/A | 20 Hz | B | PASS |
| Mock wheel command | `chassis_controller` | `/chassis/wheel_rpm_cmd` | `sand_rake_control/msg/WheelRpm` | Publish | N/A | 50 Hz | B | PASS |
| Mock chassis state | `chassis_mock` | `/mock/chassis_state` | `nav_msgs/msg/Odometry` | Publish | `odom` | 50 Hz | B | PASS |
| Real chassis command | TBD | `/safety/cmd_vel` | `geometry_msgs/msg/Twist` | Subscribe | N/A | TBD | B | BLOCKED |
| Real chassis feedback | TBD | TBD | TBD | Publish | TBD | TBD | B | BLOCKED |

---

## 3. Camera

| Item | Node | Topic | Type | Direction | frame_id | Frequency | Owner | Status |
|---|---|---|---|---|---|---|---|---|
| Front camera image | `/front_camera/camera_adapter_node` | `/front_camera/image_raw` | `sensor_msgs/msg/Image` | Publish | `front_camera_optical_frame` | Target 15 Hz | D | INTERFACE FROZEN |
| Front camera info | `/front_camera/camera_adapter_node` | `/front_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Publish | `front_camera_optical_frame` | Target 15 Hz | D | INTERFACE FROZEN |
| Rear camera image | `/rear_camera/camera_adapter_node` | `/rear_camera/image_raw` | `sensor_msgs/msg/Image` | Publish | `rear_camera_optical_frame` | Target 15 Hz | D | INTERFACE FROZEN |
| Rear camera info | `/rear_camera/camera_adapter_node` | `/rear_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Publish | `rear_camera_optical_frame` | Target 15 Hz | D | INTERFACE FROZEN |

## 4. LiDAR

| Item | Node | Topic | Type | Direction | frame_id | Frequency | Owner | Status |
|---|---|---|---|---|---|---|---|---|
| Real LiDAR data | TBD | TBD | TBD | Publish | TBD | TBD | E | BLOCKED |

Current note: LiDAR-related work is temporarily deferred by the team.

---

## 5. Current integration status

- ROS 2 Teleop → Safety control interface has been verified.
- Real chassis driver is still under debugging by B.
- Camera interface has been frozen; runtime integration is waiting for D to deliver camera_adapter.
- LiDAR integration is currently deferred.
