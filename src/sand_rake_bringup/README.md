# sand_rake_bringup

系统启动与集成包，同步自远程 `main` 分支并按当前 `motor_control` 工程整理。

## 保留内容

- `mock_system.launch.py`：复用 `sand_rake_control` 已验证的 Mock 启动文件，
  不再复制控制节点清单和参数；
- `hardware_bringup.launch.py`：启动安全控制器，并明确提示尚未接入正式底盘、
  相机和激光驱动；
- `static_tf.launch.py`：前后相机静态 TF，安装位置未标定前默认关闭；
- `camera.yaml`：主分支相机接口参数；
- `demo.rviz`：主分支 RViz 配置。

主分支中的空占位 `chassis.yaml`、`lidar.yaml` 没有运行参数和引用，未保留。
团队共享的 `sand_rake_interfaces` 和 `docs/icd_v0.1.md` 已同步；不存在的
`demo.launch.py` 引用已移除。

## 启动 Mock

```bash
ros2 launch sand_rake_bringup mock_system.launch.py
```

这会包含 `sand_rake_control/launch/chassis_mock.launch.xml`。需要团队安全事件
测试发布器时增加 `enable_test_event:=true`。键盘遥控仍需在独立交互终端启动。

## 硬件集成骨架

```bash
ros2 launch sand_rake_bringup hardware_bringup.launch.py
```

默认只启动安全控制器。当前没有正式四轮硬件命令管线；各硬件开关只会输出
阻塞原因，不会访问小车串口。
