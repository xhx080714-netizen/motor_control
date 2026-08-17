# 耙沙捡球车硬件与系统准入 Gate

> 同步说明（2026-08-17）：本文保留团队 Day 3 准入记录。当前小车分支新增了
> Mock 底盘和受限硬件诊断工具，但正式四轮硬件命令管线仍未通过集成准入。

## 1. 文档说明

本文档记录项目各模块当前的准入状态，用于系统集成判断。

状态仅允许使用：

- PASS：已完成规定验证，可以进入下一阶段集成。
- BLOCKED：当前未满足准入条件，需要完成对应事项后重新检查。

对于 BLOCKED 项，必须记录原因、负责人和下一检查时间。

当前记录阶段：Day 3

---

## 2. Day 3 Gate

| Gate | 当前状态 | 当前结论 / 证据 | 负责人 | 下一检查时间 |
|---|---|---|---|---|
| ROS 2 接口 `SafetyEvent` | PASS | `SafetyEvent.msg` 已生成；C++ / Python 示例均可发布 `/safety/event` | A | N/A |
| Keyboard Teleop | PASS | `teleop_node` 可独立运行；W/S/A/D/Q/E/Space 可正常发布 `/teleop/cmd_vel` | B / A协助验证 | N/A |
| Safety Controller | PASS | `safety_controller` 可接收 `/teleop/cmd_vel` 和 `/safety/event`，并输出 `/safety/cmd_vel` | B / A协助验证 | N/A |
| Teleop → Safety 控制链 | PASS | 独立 Teleop 终端与 launch 中的 Safety Controller 可通过 ROS 2 Topic 正常通信 | A / B | N/A |
| SafetyEvent 锁存停车链 | PASS | 测试 SafetyEvent 可触发安全停车；事件停止后仍保持锁存；人工 reset 后恢复 | A / B | N/A |
| Teleop 500 ms 超时停车 | PASS | Teleop 停止发布后 Safety Controller 能在设定超时后输出零速度 | B / A协助验证 | N/A |
| `mock_system.launch.py` 基础 Bringup | PASS | launch 可启动 Safety Controller，并可通过参数选择启动 SafetyEvent 测试发布器；退出后无相关残留进程 | A | N/A |
| Mock 控制链 | PASS | `sand_rake_bringup` 已复用控制包的 Safety → Chassis Control → Mock Chassis 启动链；Teleop 因 stdin/TTY 按设计在独立终端运行 | A / B | N/A |
| 真实底盘硬件链路 | BLOCKED | B 正在进行底盘驱动及实际硬件通信调试，尚未达到系统集成准入条件 | B | TBD |
| LiDAR / Mock LiDAR 链路 | BLOCKED | 团队当前决定暂缓雷达相关工作，因此 Mock LiDAR → ROI → SafetyEvent 链路尚未实现/集成 | E / 项目负责人 | TBD |
| RK3576 原生 ROS 2 准入 | BLOCKED | 尚未取得 D 的 Day 3 板端基线及 ROS 2 冒烟测试正式结论 | D | TBD |

---

## 3. 当前允许进入下一阶段的功能

当前以下 ROS 2 软件链路可以继续用于后续集成：

Keyboard
→ teleop_node
→ /teleop/cmd_vel
→ safety_controller
→ /safety/cmd_vel

以及：

SafetyEvent test publisher
→ /safety/event
→ safety_controller
→ STOP_LATCHED
→ manual reset

当前不得将真实底盘、LiDAR 感知链路视为已通过准入。

---

## 4. Day 4 前待确认事项

1. B 提供真实底盘驱动当前状态、实际节点名、通信接口及下一次准入检查时间。
2. D 提供 RK3576 系统基线以及原生 ROS 2 talker/listener 测试结论。
3. 项目负责人确认 LiDAR 工作暂停后的后续恢复时间；恢复前相关 Gate 保持 BLOCKED。
4. A 在获得真实设备接口后更新 `device_topics.md` 和硬件 bringup 配置。
