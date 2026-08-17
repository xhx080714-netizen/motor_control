# 耙沙捡球车 ROS 2 接口控制文档（ICD）v0.1

## 1. 文档目的

本文档定义耙沙捡球车各 ROS 2 模块之间的通信接口，
作为 B、D、E 模块开发以及 A 系统集成的统一接口基准。

当前版本：v0.1  
目标平台：Ubuntu 22.04 + ROS 2 Humble  
系统架构：RK3576 arm64 + PC

---

## 2. 接口设计原则

1. Topic、Node、Service、Message 字段统一使用英文。
2. Topic 使用绝对名称，例如 `/teleop/cmd_vel`。
3. 所有模块不得绕过安全裁决节点直接向底盘发送运动命令。
4. 未经接口负责人确认，不得自行修改已冻结的 Topic 名称和消息类型。
5. 尚未由计划或硬件确定的参数使用 `TBD` 标记，不自行假设。

---

## 3. Topic 接口表

| Topic                     | Type                                   | Provider               | Consumer                       | Unit                        | Frequency    | Timeout | frame_id                    |
| ------------------------- | -------------------------------------- | ---------------------- | ------------------------------ | --------------------------- | ------------ | ------- | --------------------------- |
| `/teleop/cmd_vel`         | `geometry_msgs/msg/Twist`              | B / Teleop             | B / Safety Arbitration         | linear: m/s, angular: rad/s | 20 Hz        | 500 ms  | N/A                         |
| `/safety/cmd_vel`         | `geometry_msgs/msg/Twist`              | B / Safety Arbitration | B / Chassis                    | linear: m/s, angular: rad/s | 20 Hz        | 500 ms  | N/A                         |
| `/mock/chassis_state`     | `nav_msgs/msg/Odometry`                | B / Mock Chassis       | A / Integration Test           | m, m/s, rad                 | 50 Hz        | 500 ms  | `odom`                      |
| `/chassis/wheel_rpm_cmd`  | `sand_rake_control/msg/WheelRpm`       | B / Chassis Control    | B / Mock Chassis               | motor rpm                   | 50 Hz        | 500 ms  | N/A                         |
| `/mock/wheel_rpm_feedback` | `sand_rake_control/msg/WheelRpm`      | B / Mock Chassis       | A / Integration Test           | motor rpm                   | 50 Hz        | 500 ms  | N/A                         |
| `/mock/lidar_points`      | `sensor_msgs/msg/PointCloud2`          | E / Mock LiDAR         | E / ROI Processing             | m                           | TBD          | TBD     | `laser_link`                |
| `/safety/event`           | `sand_rake_interfaces/msg/SafetyEvent` | E / Obstacle Detection | B / Safety Arbitration         | N/A                         | Event-driven | N/A     | N/A                         |
| `/front_camera/image_raw` | `sensor_msgs/msg/Image`                | D / Camera_adapter     | D / Vision Inference; A / RViz | pixel                       | TBD          | TBD     | TBD (camera optical frame)  |
| `/rear_camera/image_raw`  | `sensor_msgs/msg/Image`                | D / Camera_adapter     | D / Vision Inference; A / RViz | pixel                       | TBD          | TBD     | TBD (camera optical frame)  |
| `/vision/ball_detections` | `vision_msgs/msg/Detection2DArray`     | D / Vision Inference   | A / RViz                       | pixel                       | TBD          | TBD     | Same as `/camera/image_raw` |

补充：500ms意味着，当遥控命令超过500ms仍未更新，则必须将遥控命令判定为失效，然后执行停车指令！检查者均为对应topic的consumer。
      
