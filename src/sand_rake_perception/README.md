# sand_rake_perception

## 负责人

同学 E — 激光障碍检测

## 功能定位

本包负责处理激光雷达数据，在车辆前方建立虚拟保险杠区域，检测障碍物和激光数据超时，并向安全控制模块发送停车事件。

## 主要职责

- 接入 Mock 或真实激光雷达数据。
- 按车辆坐标系和 YAML 参数裁剪虚拟保险杠 ROI。
- 通过点数阈值和连续帧阈值过滤单点噪声并判定障碍物。
- 发布激光障碍和激光 Topic 超时安全事件。
- 发布 ROI 边界与触发点 Marker，支持 RViz 核对方向和位置。
- 维护室内、草地等不同场景的感知参数。
- 使用 rosbag 执行离线回归，验证障碍触发和空场误刹情况。

## 接口说明

以下内容均为三周计划中的计划接口，Day 1 阶段尚未实现：

- 输入：`/mock/lidar_points`、`/lidar/points` 或 `/scan`。
- 输出：`/safety/event`、`/perception/roi_marker`。
- 安全事件原因：`LASER` 和 `TIMEOUT`。

最终的 Topic 名称、消息类型、频率、超时、`frame_id` 和参数单位以 `docs/icd_v0.1.md` 为准。

## 当前状态

Day 1：包骨架已初始化。

