# 小车 follow_iou 双板双电机实车测试记录

> 说明（2026-08-17）：本文是历史现场记录，不再作为寄存器定义依据。寄存器、
> 状态位和控制字以公司《第四版.doc》前部码表为准，其后部示例不采用。本文中
> `state_slot_*` 和 M1/M2 左右轮候选说法只保留当时事实背景，不代表当前定义。

日期：2026-08-15  
车机：`rk3576-ubuntu`  
执行方式：从本机经 SSH 调用 RK3576 上的独立工具  
参考源码：用户提供的 `follow_iou_c.zip`

## 1. 本轮纠错结论

此前测试错误地把其他车辆寄存器表用于小车。用户明确纠正后，重新按
`follow_iou_c/src/motor_driver.c` 与 `src/main.c` 建立小车协议：

- 小车使用前、后两块双电机板；
- 前板：JP17=`/dev/ttyS6`；后板：JP18=`/dev/ttyS1`；
- 每块板只有 `M1/M2` 两个控制字；
- `main.c` 调用 `motor_set_rpm(m_r, m_l)`，因此候选映射为
  `M1=右轮`、`M2=左轮`；
- 写帧为 `01 10 <M1 word> <M2 word> CRC`；
- 电机字为 `rpm << 4 | state`，其中 `0/1/2/3` 为
  Coast/参考正向/参考反向/Brake；
- 参考实现写后只等待 2 ms，不读取 `0x10` 回应。

由此得到待现场视觉复核的四轮候选映射：

| 板/电机 | 候选物理轮位 |
|---|---|
| front/M1 | RF（右前） |
| front/M2 | LF（左前） |
| rear/M1 | RR（右后） |
| rear/M2 | LR（左后） |

## 2. 被撤销的错误解释

早期使用其他车辆地址所得读数和异常结论全部撤销，不作为小车模式、报警、
版本、转速或供电状态的证据。本轮只保留 `follow_iou_c` 明确定义的操作：
两块板从模式读取地址均得到 `0x0001`（FOLLOW）。本轮没有执行清报警、清
霍尔、写模式或写减速参数。

## 3. 工具与离线验证

独立工具：

```text
sand_rake_hw_tools/small_car_four_wheel_jog_once
```

固定参数：

```text
单电机 30 motor rpm
持续 300 ms
另一电机保持 Coast
动作前双电机 Coast
动作后双电机 Coast
SIGINT/SIGTERM 后尝试最终 Coast
```

本机和 RK3576 均完成构建；RK3576 伪串口测试结果：

```text
2 tests, 0 errors, 0 failures, 0 skipped
```

## 4. Coast-only 前置验证

| 驱动板 | tty | `0x0008` | mode latency | Coast 帧 | write latency | 结果 |
|---|---|---:|---:|---|---:|---|
| front | `/dev/ttyS6` | `0x0001` | 3735 µs | `01 10 00 00 00 00 C0 09` | 9 µs | OK |
| rear | `/dev/ttyS1` | `0x0001` | 3726 µs | `01 10 00 00 00 00 C0 09` | 9 µs | OK |

## 5. 单电机点动执行结果

每次事务均为“模式只读 → 预 Coast → 单电机 30 rpm/300 ms → 最终 Coast”。

| 板/电机 | 参考方向 | 点动帧 | mode latency/µs | pre/jog/final write latency/µs | 软件结果 |
|---|---|---|---:|---:|---|
| front/M1 | state 1 | `01 10 01 E1 00 00 91 C3` | 3801 | 9 / 9 / 82 | OK |
| front/M1 | state 2 | `01 10 01 E2 00 00 61 C3` | 3757 | 8 / 7 / 82 | OK |
| front/M2 | state 1 | `01 10 00 00 01 E1 01 D1` | 3775 | 20 / 6 / 78 | OK |
| front/M2 | state 2 | `01 10 00 00 01 E2 41 D0` | 3766 | 9 / 7 / 80 | OK |
| rear/M1 | state 1 | `01 10 01 E1 00 00 91 C3` | 3733 | 9 / 7 / 83 | OK |
| rear/M1 | state 2 | `01 10 01 E2 00 00 61 C3` | 3713 | 9 / 7 / 82 | OK |
| rear/M2 | state 1 | `01 10 00 00 01 E1 01 D1` | 3680 | 8 / 7 / 90 | OK |
| rear/M2 | state 2 | `01 10 00 00 01 E2 41 D0` | 3514 | 6 / 7 / 106 | OK |

所有 8 次调用退出码均为 0，所有最终 Coast 均成功写出。测试结束后又分别
向前、后板发送一次双电机 Coast；两次均 `result=OK`，随后确认两个 tty
均无进程占用。

## 6. 证据边界

上述 `write=OK` 证明 8 字节帧已完整写入 Linux tty。参考协议不读取
`0x10` 回应，因此不能把它表述为驱动板 strict echo 或执行确认。

软件无法远程观察轮胎。以下结论仍需现场人员根据实际轮胎动作确认后冻结：

- `front/M1/M2`、`rear/M1/M2` 是否分别对应候选轮位；
- state 1/state 2 分别对应物理车轮的哪个旋向；
- 每个物理轮实现车辆前进所需的 state。

在取得现场视觉结果前，不把候选映射写入正式 YAML，不进入整车落地或键盘
测试。

## 7. 提速复测

现场反馈 30 rpm 下四轮均未观察到动作。保持车辆架空后，将工具速度限制扩展
为仅允许 `30/60/100 rpm` 三档，并在本机、RK3576 重新完成伪串口测试。

首次执行 front/M1、state 1、60 rpm、300 ms：

```text
点动帧：01 10 03 C1 00 00 91 B1
mode_raw=0x0001
mode_latency_us=3739
pre_coast_latency_us=9
jog_latency_us=7
final_coast_latency_us=89
final_coast_result=OK
result=OK
```

该结果仍只证明完整帧写入 tty；是否产生轮胎动作等待现场观察。

现场确认 60 rpm 与 100 rpm 均无轮胎动作。随后在 100 rpm 点动发送 50 ms
后，按参考代码的 `REG_M1_STATE=0x0000` 连续读取两个状态字：

```text
mode_raw=0x0001
点动帧=01 10 06 41 00 00 90 95
motor_state_latency_us=3733
state_slot_0_raw=0x0000
state_slot_1_raw=0x0641
final_coast_result=OK
result=OK
```

`0x0641 = (100 << 4) | 1`。状态区出现了与所发控制字相同的值，这是板卡
状态更新的证据，但不能单独证明输出级已经执行。状态读取顺序与写帧中的
M1/M2 命名存在反序，后续需单独冻结。由于现场仍无机械动作，不再升速。
下一步优先确认驱动动力母线、急停/使能、接触器/保险和电机输出级是否实际
投入；绿灯与 `mode=1` 只能证明逻辑侧处于跟随模式。
