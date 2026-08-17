# sand_rake_hw_tools

实车标定阶段的小车独立工具包。寄存器、状态位及控制字只采用公司
《第四版.doc》的前部码表；该文档后部示例不用于实现或测试。串口事务复用
`sand_rake_control` 的 CRC、RS232 和 Modbus `0x03` 能力。

## 小车四轮点动测试工具

`small_car_four_wheel_jog_once` 用于逐板、逐电机检查：

- 前板 JP17 固定 `/dev/ttyS6`，后板 JP18 固定 `/dev/ttyS1`；
- 每块板包含 M1、M2，码表定义 `0x0000=M1`、`0x0001=M2`；
- 公司表没有定义 M1/M2 对应哪一个物理轮，现场确认前不得推定左右；
- 固定控制帧为 `从站、0x10、M1 控制字、M2 控制字、CRC`；
- 控制字为 `rpm << 4 | state`，状态 0/1/2/3/4 分别是自由停车、正转、
  反转、刹车停车、减速到零；
- 默认 dry-run，真实执行必须确认车辆架空、硬件停机手段和 tty 独占；
- 单电机点动仅允许 30/60/100 rpm，持续 300 ms，另一电机保持自由停车；
- 动作前后及收到中断时均尝试发送双电机自由停车。

`--status-only` 先检查模式，再读取 `0x0000～0x000B`，输出 M1/M2 命令状态、
两路霍尔累计值、两路反馈转速、模式、母线电压、报警码和程序版本，全程不写入。

`--initialize` 的五项为：清报警、清霍尔累计、设置 FOLLOW、M1 减速时间为 0、
M2 减速时间为 0。现有固件路径发送 `0x06` 后不读取回显；工具通过最后重新读取
`0x0008` 确认 FOLLOW。此行为来自现有固件实现，不引用公司文档后部错误示例。

`0x10` 写操作显示 `write=OK` 只代表 Linux tty 接收了完整发送数据，不代表电机
已经产生机械动作。

完整命令和安全顺序见 `docs/小车控制命令操作手册.md`。

## RK3576 构建与测试

```bash
cd /home/rockchip/sand_rake_ws
source /opt/ros/humble/setup.bash

CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --symlink-install \
  --executor sequential --parallel-workers 1 \
  --packages-select sand_rake_interfaces

source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --symlink-install \
  --executor sequential --parallel-workers 1 \
  --packages-select sand_rake_control

source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --symlink-install \
  --executor sequential --parallel-workers 1 \
  --packages-select sand_rake_hw_tools

source install/setup.bash
colcon test --executor sequential --parallel-workers 1 \
  --packages-select sand_rake_interfaces sand_rake_control sand_rake_hw_tools
colcon test-result --verbose
```

三条构建命令必须前后执行；上一条成功结束后才能开始下一条，禁止在多个终端中
同时构建。
