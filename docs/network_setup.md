# PC–RK3576 ROS 2 DDS Network Setup

> 同步说明（2026-08-17）：本文保留团队 `main` 的 Day 5 网络方案和历史结果。
> 车机目前已经恢复联网，但 DDS 双向发现、相机 Topic 和断线重连尚未按本文重新
> 验证，因此文中的历史 `PASS/BLOCKED` 不能直接当作当前验收结果。

## 1. Purpose

本文档记录耙沙车项目中开发 PC 与 RK3576 车机之间的网络及 ROS 2 DDS 通信配置、验证方法和故障排查流程。

目标通信结构：

```text
Development PC
Ubuntu 22.04
ROS 2 Humble
        │
        │ Ethernet / Wi-Fi LAN
        │ DDS / UDP-IP
        ▼
RK3576 Vehicle Computer
Ubuntu 22.04
ROS 2 Humble
```

Day 5 网络验收目标：

1. PC 与 RK3576 IP 网络互通。
2. 两端使用一致的 `ROS_DOMAIN_ID`。
3. 禁止 ROS 2 限制为 localhost 通信。
4. RK3576 发布的 ROS 2 Topic 可以被 PC 发现并订阅。
5. PC 发布的 ROS 2 Topic 可以被 RK3576 发现并订阅。
6. 网络断开并重新连接后，DDS 能够恢复发现。
7. PC 最终能够发现车机真实相机等项目 Topic。

---

# 2. System Environment

## 2.1 Development PC

```text
OS: Ubuntu 22.04
Architecture: x86_64
ROS distribution: ROS 2 Humble
```

ROS 环境：

```bash
source /opt/ros/humble/setup.bash
source ~/sand_rake_ws/install/setup.bash
```

---

## 2.2 RK3576 Vehicle Computer

```text
Platform: RK3576
OS: Ubuntu 22.04
Architecture: arm64
ROS distribution: ROS 2 Humble
```

ROS 环境：

```bash
source /opt/ros/humble/setup.bash
source ~/sand_rake_ws/install/setup.bash
```

> 如果车机 workspace 实际路径与 PC 不同，以车机实际部署路径为准。

---

# 3. Network Configuration

## 3.1 Current Network Topology

PC 与 RK3576 位于同一局域网中。

当前测试期间记录的地址：

```text
Development PC:
192.168.1.124

RK3576:
192.168.1.130
```

> 上述地址为当前测试阶段记录值。正式验收前需要重新通过 `ip addr` 确认，不能仅依赖历史记录。

查看本机 IPv4 地址：

```bash
ip addr
```

也可以使用：

```bash
hostname -I
```

---

# 4. IP Connectivity Test

DDS 测试前首先验证基础 IP 网络。

## 4.1 PC → RK3576

PC：

```bash
ping 192.168.1.130
```

期望：

```text
64 bytes from 192.168.1.130 ...
```

---

## 4.2 RK3576 → PC

RK3576：

```bash
ping 192.168.1.124
```

期望：

```text
64 bytes from 192.168.1.124 ...
```

当前状态：

```text
PC → RK3576 ping: PASS
RK3576 → PC ping: PASS
```

说明基础 IP 网络层已经具备双向通信能力。

---

# 5. ROS 2 DDS Environment

项目统一使用：

```text
ROS_DOMAIN_ID=42
ROS_LOCALHOST_ONLY=0
```

## 5.1 ROS_DOMAIN_ID

ROS 2 节点只有处于兼容的 DDS Domain 中才能通过 DDS Discovery 互相发现。

本项目统一设置：

```bash
export ROS_DOMAIN_ID=42
```

检查：

```bash
echo $ROS_DOMAIN_ID
```

期望：

```text
42
```

---

## 5.2 ROS_LOCALHOST_ONLY

跨机器通信需要保证 ROS 2 不被限制在本机回环接口。

设置：

```bash
export ROS_LOCALHOST_ONLY=0
```

检查：

```bash
echo $ROS_LOCALHOST_ONLY
```

期望：

```text
0
```

---

# 6. Important Note About Shell Environment

命令：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

只对：

```text
当前 Shell
+
该 Shell 创建的子进程
```

有效。

例如：

```text
RK3576 Local Terminal A
export ROS_DOMAIN_ID=42
```

不会自动影响之后新开的：

```text
SSH Terminal B
```

因此每次开始 DDS 测试时，都必须先检查：

```bash
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
```

不能假设另一终端已经继承这些变量。

---

# 7. Temporary DDS Test Configuration

正式写入 shell 配置前，测试阶段建议手动设置。

PC：

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

RK3576：

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

两端分别检查：

```bash
echo $ROS_DISTRO
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
```

期望：

```text
humble
42
0
```

---

# 8. ROS 2 Daemon Reset

ROS 2 CLI daemon 可能由之前不同 DDS Domain 环境启动。

改变 `ROS_DOMAIN_ID` 后，在测试前执行：

```bash
ros2 daemon stop
ros2 daemon start
```

PC 和 RK3576 均建议执行一次。

---

# 9. RK3576 → PC DDS Test

## 9.1 RK3576 Publisher

RK3576：

```bash
ros2 topic pub -r 2 /dds_test std_msgs/msg/String "data: 'hello from rk3576'"
```

预期：

```text
publishing #1
publishing #2
publishing #3
...
```

保持 Publisher 运行。

---

## 9.2 PC Discovery Test

PC：

```bash
ros2 topic list
```

期望存在：

```text
/dds_test
```

进一步检查：

```bash
ros2 topic info /dds_test
```

然后订阅：

```bash
ros2 topic echo /dds_test
```

期望：

```text
data: hello from rk3576
---
```

当前状态：

```text
RK3576 → PC DDS discovery:
BLOCKED

RK3576 → PC message transmission:
BLOCKED

Reason:
Vehicle power supply failure. RK3576 currently unavailable for runtime test.
```

---

# 10. PC → RK3576 DDS Test

RK3576 → PC 验证通过后，再进行反方向测试。

PC：

```bash
ros2 topic pub -r 2 /dds_test_pc std_msgs/msg/String "data: 'hello from pc'"
```

RK3576：

```bash
ros2 topic list
```

应存在：

```text
/dds_test_pc
```

然后：

```bash
ros2 topic echo /dds_test_pc
```

期望：

```text
data: hello from pc
---
```

当前状态：

```text
PC → RK3576 DDS discovery:
BLOCKED

PC → RK3576 message transmission:
BLOCKED

Reason:
Vehicle power supply failure.
```

---

# 11. DDS Multicast Test

如果：

```text
ping PASS
```

但：

```text
ros2 topic list
```

无法发现另一台机器上的 Topic，则下一步检查 DDS Discovery 所依赖的网络通信。

## 11.1 PC Receive / RK3576 Send

PC：

```bash
ros2 multicast receive
```

RK3576：

```bash
ros2 multicast send
```

确认 PC 是否收到数据。

---

## 11.2 RK3576 Receive / PC Send

RK3576：

```bash
ros2 multicast receive
```

PC：

```bash
ros2 multicast send
```

确认 RK3576 是否收到数据。

---

## 11.3 Interpretation

如果：

```text
ping       PASS
multicast  FAIL
```

优先排查：

```text
Firewall
Wireless AP multicast isolation
Network interface
VPN
Virtual network interfaces
Router/AP configuration
```

如果：

```text
ping       PASS
multicast  PASS
DDS Topic  FAIL
```

再进一步检查：

```text
ROS_DOMAIN_ID
ROS_LOCALHOST_ONLY
RMW implementation
DDS implementation
Multiple network interface selection
VPN / virtual interfaces
DDS configuration
```

在完成上述基础检查之前，不应直接修改复杂 DDS XML 配置。

---

# 12. RMW Information

如果 multicast 正常但 DDS Discovery 仍异常，可检查 ROS 2 当前使用的 RMW。

查看环境变量：

```bash
echo $RMW_IMPLEMENTATION
```

如果为空，不代表 ROS 2 没有使用 RMW，而是没有通过该环境变量强制指定实现。

必要时可以进一步查看系统安装的 RMW package：

```bash
ros2 pkg list | grep rmw
```

只有在基础网络、Domain、localhost 和 multicast 均确认正常后，才进入 RMW/DDS 实现层排查。

---

# 13. Multiple Network Interfaces

DDS Discovery 失败时需要注意 PC 是否同时存在：

```text
Ethernet
Wi-Fi
VPN
Docker bridge
Virtual machine interface
TUN/TAP interface
```

检查：

```bash
ip addr
```

以及：

```bash
ip route
```

需要确认：

```text
PC → RK3576
```

实际流量使用的是与 RK3576 同一局域网的物理网卡。

---

# 14. Real Project Topic Test

最小 `/dds_test` 双向通信通过后，再验证项目真实 Topic。

根据实际交付情况检查车机 Topic：

```bash
ros2 topic list
```

重点关注已经冻结的双摄接口：

```text
/front_camera/image_raw
/front_camera/camera_info

/rear_camera/image_raw
/rear_camera/camera_info
```

进一步检查：

```bash
ros2 topic info /front_camera/image_raw
ros2 topic info /rear_camera/image_raw
```

PC 应能够发现车机上的实际 Camera Publisher。

当前状态：

```text
Real camera Topic discovery:
BLOCKED

Reason:
RK3576 unavailable because of vehicle power supply failure.
```

---

# 15. Disconnect / Reconnect Test

DDS 基础通信通过之后必须测试网络恢复能力。

测试步骤：

1. 保持 RK3576 ROS 2 Publisher 正常运行。
2. PC 正常发现并订阅该 Topic。
3. 人工断开 PC 与 RK3576 之间的网络连接。
4. 确认通信中断。
5. 恢复网络连接。
6. 再次验证 IP connectivity。
7. 检查 DDS 是否自动恢复 Discovery。
8. 再次执行 Topic echo。

恢复网络后首先：

```bash
ping <peer_ip>
```

随后：

```bash
ros2 topic list
```

以及：

```bash
ros2 topic echo /dds_test
```

验收目标：

```text
Network reconnect:
PASS

DDS rediscovery:
PASS

Message transmission after reconnect:
PASS
```

当前状态：

```text
BLOCKED

Reason:
Vehicle power supply failure.
```

---

# 16. Persistent Environment Configuration

只有 DDS 双向通信验证 PASS 后，再考虑将项目统一参数写入：

```bash
~/.bashrc
```

例如：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

然后：

```bash
source ~/.bashrc
```

必须重新打开一个全新 Terminal 或重新 SSH 登录，再检查：

```bash
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
```

期望：

```text
42
0
```

不能只验证修改 `.bashrc` 的原 Shell。

---

# 17. Final Day 5 DDS Gate

| Check                                | Status      | Evidence / Notes            |
| ------------------------------------ | ----------- | --------------------------- |
| PC → RK3576 ping                     | PASS        | Previously verified         |
| RK3576 → PC ping                     | PASS        | Previously verified         |
| PC `ROS_DOMAIN_ID=42`                | To re-check | Verify before runtime test  |
| RK3576 `ROS_DOMAIN_ID=42`            | To re-check | Verify in SSH shell         |
| PC `ROS_LOCALHOST_ONLY=0`            | To re-check | Verify before runtime test  |
| RK3576 `ROS_LOCALHOST_ONLY=0`        | To re-check | Verify in SSH shell         |
| RK3576 → PC `/dds_test` discovery    | BLOCKED     | Vehicle power failure       |
| RK3576 → PC message reception        | BLOCKED     | Vehicle power failure       |
| PC → RK3576 `/dds_test_pc` discovery | BLOCKED     | Vehicle power failure       |
| PC → RK3576 message reception        | BLOCKED     | Vehicle power failure       |
| DDS multicast test                   | BLOCKED     | Run only if discovery fails |
| Real camera Topic discovery          | BLOCKED     | Requires vehicle runtime    |
| Network disconnect/reconnect         | BLOCKED     | Requires vehicle runtime    |
| DDS rediscovery after reconnect      | BLOCKED     | Requires vehicle runtime    |

---

# 18. Troubleshooting Order

ROS 2 跨机器 Topic 无法发现时，按照以下顺序排查：

```text
1. Is the publisher actually running?
            ↓
2. Are both ROS environments sourced?
            ↓
3. Is ROS_DOMAIN_ID identical?
            ↓
4. Is ROS_LOCALHOST_ONLY=0?
            ↓
5. Can the machines ping each other?
            ↓
6. Does multicast work in both directions?
            ↓
7. Are firewall / AP / VPN interfering?
            ↓
8. Are multiple NICs affecting DDS interface selection?
            ↓
9. Check RMW / DDS implementation
            ↓
10. Only then consider custom DDS configuration
```

原则：

> 优先验证最简单、最底层、最容易确定的问题，不在没有证据的情况下直接修改 DDS XML 或切换中间件。

---

# 19. Current Status

截至当前 Day 5：

```text
Basic network topology          DEFINED
PC ↔ RK3576 historical ping     PASS

ROS_DOMAIN_ID policy            FROZEN: 42
ROS_LOCALHOST_ONLY policy       FROZEN: 0

DDS test procedure              DEFINED
Multicast troubleshooting       DEFINED
Reconnect test procedure        DEFINED

RK3576 → PC runtime DDS         BLOCKED
PC → RK3576 runtime DDS         BLOCKED
Real camera Topic discovery     BLOCKED
Reconnect runtime validation    BLOCKED
```

Blocking reason:

```text
Vehicle power supply failure.
RK3576 is temporarily unavailable for runtime communication testing.
```

这些 BLOCKED 项必须在车辆供电恢复后重新执行并使用实际测试结果更新，不得提前标记为 PASS。

