# motor_control

ROS 2 Humble 小车底盘工程。本目录只保留小车有效源码、配置、测试和交接
记录；`build/`、`install/`、`log/` 均为可重建产物，不应提交 Git。

## 工程内容

- `src/sand_rake_control`：遥控、安全裁决、小车运动学以及不连接真车的 Mock 仿真。
- `src/sand_rake_hw_tools`：受限的小车双板双电机实车诊断/点动工具。
- `src/sand_rake_interfaces`：底盘安全控制所依赖的 `SafetyEvent` 消息接口。
- 公司《第四版.doc》前部寄存器表：唯一寄存器与控制字依据（外部受控文件，不随仓库发布）。
- `docs/小车控制命令操作手册.md`：现场人员可逐步执行的构建、检查、初始化和单轮点动流程。
- `docs/小车follow_iou双板双电机实车测试记录_2026-08-15.md`：原始实车结果与证据边界。
- `docs/RK3576部署与实车测试_新对话交接摘要.md`：后续对话必读的当前状态。

小车电机寄存器及控制字以公司《第四版.doc》的前部码表为唯一依据；文档后部
示例已由用户明确判定为错误，不用于实现或验证。小车使用两块双电机板：前板
JP17=`/dev/ttyS6`，后板 JP18=`/dev/ttyS1`；每块板接收固定 8 字节自定义
`0x10` M1/M2 帧。

## Build

```bash
source /opt/ros/humble/setup.bash

MAKEFLAGS="-j1" CMAKE_BUILD_PARALLEL_LEVEL=1 \
  colcon build --symlink-install \
  --executor sequential --parallel-workers 1 \
  --packages-select sand_rake_interfaces

source install/setup.bash
MAKEFLAGS="-j1" CMAKE_BUILD_PARALLEL_LEVEL=1 \
  colcon build --symlink-install \
  --executor sequential --parallel-workers 1 \
  --packages-select sand_rake_control

source install/setup.bash
MAKEFLAGS="-j1" CMAKE_BUILD_PARALLEL_LEVEL=1 \
  colcon build --symlink-install \
  --executor sequential --parallel-workers 1 \
  --packages-select sand_rake_hw_tools

source install/setup.bash
CTEST_PARALLEL_LEVEL=1 colcon test --executor sequential --parallel-workers 1 --packages-select sand_rake_interfaces sand_rake_control sand_rake_hw_tools
colcon test-result --verbose
```

这是完全串行构建：`colcon` 一次只处理一个包，CMake、Make 或 Ninja 在包内也
只能运行一个编译任务。三个包必须按上述顺序逐个构建，不要并行启动多条命令。

## Scope

本仓库只负责小车电机驱动和底盘控制。感知、视觉、相机/雷达 bringup、通用录包
工具及其团队协作文档不属于本仓库范围，已从本地工作树剔除。`main` 远端分支
保持只读，以上清理没有修改远端内容。

## Hardware safety boundary

Do not issue a real motor or initialization write unless the vehicle is
securely raised, only one board/channel is selected, personnel are clear of the
drivetrain, and an immediate Coast or hardware power cut is available.

Do not apply another vehicle's register table, four single-register motor
writes, or inferred read-register meanings to the small car. The generic
Modbus codec does not define a device register map by itself.

完整实车步骤见 `docs/小车控制命令操作手册.md`。仓库不再维护自行编写的
重复寄存器码表。

四轮物理轮位和每个电机的正反方向尚未实测冻结，因此当前不保留板级
映射节点、映射配置或真车命令管线。

键盘节点需要交互式终端，因此不在 Mock launch 内启动。需要仿真时，
先启动 `chassis_mock.launch.xml`，再在独立终端运行：

```bash
ros2 run sand_rake_control teleop_node --ros-args \
  --params-file src/sand_rake_control/config/small_vehicle.yaml
```

Current hardware status (2026-08-17): the RK3576 source has been updated and
built serially. Both boards completed the five-step initialization, report
FOLLOW, no alarm, and firmware version `0x0004`. Coast releases all four wheels,
but low-speed commands have produced no mechanical motion. During the latest
front/M1 50 rpm test, a full status sample started about 50 ms after the write
(read latency about 5.6 ms) and showed both
command states, Hall counts, and feedback speeds at zero, with a stable 27.0 V
bus and no alarm. No tty owner or other ROS node was found. Before more motion
tests, confirm with the vendor that firmware `0x0004` matches the company table
and whether a separate hardware/protocol enable is required. Wheel mapping and
direction remain unfrozen.
