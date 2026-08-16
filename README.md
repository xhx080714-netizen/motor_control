# motor_control

ROS 2 Humble 小车底盘工程。本目录只保留小车有效源码、配置、测试和交接
记录；`build/`、`install/`、`log/` 均为可重建产物，不应提交 Git。

## 工程内容

- `src/sand_rake_control`：遥控、安全裁决、小车运动学以及不连接真车的 Mock 仿真。
- `src/sand_rake_hw_tools`：受限的小车双板双电机实车诊断/点动工具。
- `docs/小车双电机驱动板寄存器读写映射码表_follow_iou.md`：唯一寄存器与帧映射依据。
- `docs/小车follow_iou双板双电机实车测试记录_2026-08-15.md`：原始实车结果与证据边界。
- `docs/RK3576部署与实车测试_新对话交接摘要.md`：后续对话必读的当前状态。

小车电机协议唯一权威来源是用户提供的 `follow_iou_c.zip`。小车使用两块
双电机板：前板 JP17=`/dev/ttyS6`，后板 JP18=`/dev/ttyS1`；每块板接收固定
8 字节自定义 `0x10` M1/M2 帧。

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

## Hardware safety boundary

Do not issue a real motor or initialization write unless the vehicle is
securely raised, only one board/channel is selected, personnel are clear of the
drivetrain, and an immediate Coast or hardware power cut is available.

Do not apply another vehicle's register table, four single-register motor
writes, or inferred read-register meanings to the small car. The generic
Modbus codec does not define a device register map by itself.

完整实车步骤只从上述映射码表、实车记录和交接摘要获取，不再维护重复的
寄存器/轮位文档。

四轮物理轮位和每个电机的正反方向尚未实测冻结，因此当前不保留板级
映射节点、映射配置或真车命令管线。

键盘节点需要交互式终端，因此不在 Mock launch 内启动。需要仿真时，
先启动 `chassis_mock.launch.xml`，再在独立终端运行：

```bash
ros2 run sand_rake_control teleop_node --ros-args \
  --params-file src/sand_rake_control/config/small_vehicle.yaml
```

Current hardware status (2026-08-16): both boards completed the five-step
`follow_iou` initialization sequence, report FOLLOW from read register
`0x0008`, and were returned to Coast. A subsequent front/M1 30 rpm command was
read back from the raw state area; its physical wheel result was not recorded.
Wheel mapping and direction remain unfrozen.
