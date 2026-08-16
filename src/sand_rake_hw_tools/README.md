# sand_rake_hw_tools

实车标定阶段的小车独立工具包。它只复用 `sand_rake_control` 的通用 CRC、
RS232 和 Modbus `0x03` 事务层；寄存器和控制帧严格来自 `follow_iou_c.zip`。
不符合该协议白名单的旧诊断工具已经从工程中删除。

## 小车四轮点动测试工具

`small_car_four_wheel_jog_once` 是独立新增的逐电机测试工具，不修改
`sand_rake_control`。小车协议按用户提供的 `follow_iou_c.zip` 实际代码实现：

- 小车有前、后两块板，每块板控制 `M1/M2` 两个电机；
- `M1=右轮`、`M2=左轮`，因此四轮候选映射为
  `front/M1=RF`、`front/M2=LF`、`rear/M1=RR`、`rear/M2=LR`；
- 控制帧固定为 8 字节 `01 10 <M1 word> <M2 word> CRC`；
- 电机字编码为 `rpm << 4 | state`，`0/1/2/3` 分别为
  Coast/参考正向/参考反向/Brake；
- 参考代码不读取 `0x10` 回应，本工具保持相同收发行为；
- 前板仍固定 JP17=`/dev/ttyS6`，后板固定 JP18=`/dev/ttyS1`；
- `--initialize` 严格按参考代码顺序执行清报警、清霍尔、设置 FOLLOW、M1/M2
  减速时间置 0；初始化前后均发送 Coast，异常或中断时尝试最终 Coast。

真实动作前读取参考代码明确定义的 `0x0008` 当前模式并要求值为 `1`。
参考代码只定义向 `0x0009` 写 `1` 清报警，没有定义其读取语义。真实执行
还要求显式确认车辆架空、硬件断电手段和 tty 独占。

受控初始化示例：

```bash
ros2 run sand_rake_hw_tools small_car_four_wheel_jog_once \
  --board front --initialize --device /dev/ttyS6 --execute \
  --confirm-wheels-off-ground \
  --confirm-hardware-stop-ready \
  --confirm-exclusive-tty
```

该入口不要求标准 `0x06` 严格回显；五条写入完成并最终 Coast 后，等待延迟
字节到达、清空输入，再读取 `0x0008` 复核 FOLLOW。前后板必须分开执行。

小车纯只读状态检查：

```bash
ros2 run sand_rake_hw_tools small_car_four_wheel_jog_once \
  --board front --status-only --device /dev/ttyS6 --execute \
  --confirm-wheels-off-ground \
  --confirm-hardware-stop-ready \
  --confirm-exclusive-tty
```

该模式只读取 `0x0008` 当前模式以及从 `0x0000` 开始的两个电机状态字，
不会发送 `0x06` 或 `0x10`。

离线查看某一电机的固定测试计划：

```bash
ros2 run sand_rake_hw_tools small_car_four_wheel_jog_once \
  --board front --motor M1 --direction forward
```

当前固定安全参数为：单电机、300 ms、另一电机保持 Coast、动作前后双电机
Coast；速度只允许 `30/60/100 motor rpm` 三档。分别替换
`front/rear`、`M1/M2` 和
`forward/reverse` 可检查四轮的待执行帧；dry-run 不会打开 tty。

点动帧发送 50 ms 后，工具按参考代码中的 `REG_M1_STATE=0x0000` 读取连续
两个状态字并打印 `state_slot_0_raw/state_slot_1_raw`，随后在总时长 300 ms
到达时发送最终 Coast。实车已出现返回槽位反序证据，当前只能保留两个原始
字并观察是否出现所发控制字，不能据此确认机械执行或冻结 M1/M2 读取顺序。

测试时会额外构建一个不安装的伪串口专用二进制，验证以下事务顺序：

```text
读取 0x0008 当前模式 → 预 Coast → 受限 rpm 点动
→ 50 ms 后读取 0x0000 起始的两个状态字 → 总时长 300 ms → 最终 Coast
```

伪串口测试还覆盖设备/板卡组合拒绝、Coast-only、初始化五帧的完整顺序、
初始化前置读取超时不写入，以及点动/初始化收到 SIGINT 后仍发送最终 Coast。

## RK3576 构建与测试

整合工作区位于 `/home/rockchip/sand_rake_ws`：

```bash
cd /home/rockchip/sand_rake_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to sand_rake_hw_tools
source install/setup.bash
colcon test --packages-select sand_rake_hw_tools
colcon test-result --verbose
```

预期测试结尾：

```text
1 test, 0 errors, 0 failures, 0 skipped
```
