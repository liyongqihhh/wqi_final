# PX4 SITL 飞控集成

## 1. 作用与边界

本项目仅保留 `px4` 后端，旧 SJTU 替代飞控已删除。PX4 模式
运行真实的 PX4 v1.16 SITL 软件栈，使用 PX4 状态估计、位置/姿态控制和电机混控，
再由 Gazebo Classic Iris 电机与传感器插件执行物理仿真。ROS 2 只通过 Offboard
接口发送位置设定值；精确降落时保持 XY 位置并发送向下速度，不持续修改
Gazebo 实体坐标。

这是软件在环仿真，不是实体无人机飞控或硬件在环测试。校园 map 与 PX4 本地 NED
之间仍使用 Gazebo ground-truth 里程计做平移对齐，尚未实现 GPS/视觉/激光 SLAM
融合定位。

联合 PX4 SITL 为连续甲板着陆启用原生 EKF 的仿真三维速度辅助，仅融合已有
Gazebo 世界速度与高度，不融合其水平位置或姿态。它是 ground-truth 辅助，
不是 VIO；水平定位仍使用 GPS，独立 UAV 默认不启用该辅助。
当前固定质量 Iris 的空中归一化悬停推力标定约为 `0.705`，联合模式固定基准
`MPC_THR_HOVER=0.71`，避免使用受甲板支撑影响的在线推力估计。实际电机推力仍由
PX4 计算，起降仍需原生解锁/落地检测；独立 UAV 保留原生默认估计。
参数、就绪话题和失败保护见 [PX4 能量调度说明](px4_energy_scheduling.md)。

联合模式另外启用 Gazebo 接触感知 IMU 兼容选项：垂直甲板接触时使用
物理速度差分而非不一致的约束加速度读数，并在 MAVLink 下采样前使用 20 ms
加速度低通抑制接触数值扰动。测量仍通过原生 IMU/MAVLink 链路进入
PX4 EKF，保留原噪声及陀螺仪，不覆盖估计结果或落地标志。补丁文件位于
`simulation_ws/src/uav_px4_bringup/patches/gazebo_contact_imu.patch`，新安装自动应用。
已有安装须关闭仿真后执行 README 第 6.5.4 节的
`refresh_gazebo_imu.sh` 命令；未重编译插件时联合启动会拒绝旧二进制。

## 2. 软件组成

```text
配送状态机 /uav/execute_delivery
            |
            v
项目统一接口 /uav/takeoff /uav/land /uav/fly_to_pose
            |
            v
px4_offboard_controller
  map ENU <-> local NED
  OffboardControlMode + TrajectorySetpoint + VehicleCommand
            |
            v
Micro XRCE DDS Agent <-> PX4 SITL v1.16
            |
            v
Gazebo Classic Iris 电机、IMU、GPS、磁力计和气压计插件
```

项目传感器适配脚本在官方 Iris SDF 上追加 `/uav/odom`、顶部 3D 雷达、下向测距、
四个斜下测距、下视相机和 ROS IMU。官方 PX4 电机、MAVLink 和飞控传感器插件保持
不变。

## 3. 安装

```bash
cd ~/design_final/wqi_final/simulation_ws
bash ./setup_px4_backend.sh --install-system-deps
bash ./build_workspace.sh
source ./setup_workspace.bash
```

外部产物位置：

```text
~/design_final_artifacts/px4/PX4-Autopilot
~/design_final_artifacts/px4_ros_ws
```

这些目录是外部构建副本和产物，不应提交。完整上游源码已通过 Git 子模块纳入
`third_party/PX4-Autopilot`、`third_party/px4_msgs` 和
`third_party/Micro-XRCE-DDS-Agent`。新电脑须递归 clone，或在仓库根目录执行
`git submodule update --init --recursive`，再运行安装脚本。
安装脚本检查固定提交并从仓库源码生成外部副本，不会替换为上游最新分支。
项目补丁仍位于 `simulation_ws/src/uav_px4_bringup/patches/`，见
[源码管理与复现说明](../third_party/README.md)。

## 4. 启动

PX4 后端：

```bash
ros2 launch uav_bringup uav_sim.launch.py \
  flight_backend:=px4 gui:=false rviz:=true
```

协同 PX4 后端：

```bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  flight_backend:=px4 gui:=false rviz:=true \
  enable_energy_constraints:=false enable_dynamic_obstacles:=false
```

阶段 3 至 6 的命令行启动入口仅使用 PX4 SITL + ROS 2 Offboard，不提供替代飞控。
自定义控制界面已从源码交付中移除，RViz/Gazebo 界面仍可通过启动参数开启。

## 5. 检查与基础验收

```bash
ros2 topic echo /uav/flight_backend --once
ros2 topic echo /uav/px4/connected --once
ros2 topic echo /fmu/out/vehicle_status_v1 --once
ros2 service call /uav/takeoff std_srvs/srv/Trigger '{}'
ros2 topic echo /uav/odom --once
ros2 service call /uav/land std_srvs/srv/Trigger '{}'
```

2026-10-02 本机无界面回归证据：

- PX4 DDS 连接为 `true`。
- 起飞后进入 Offboard，`arming_state=2`、`nav_state=14`、`failsafe=false`。
- 2 m `FlyToPose` 成功，最终三维位置误差为 `0.114 m`。
- 降落后 `landed=true`、`at_rest=true`、`arming_state=1`。

这组结果证明基础闭环可运行，但不替代校园多目标、动态避障、协同停靠和正式实验
矩阵。完成论文实验前，应使用当前 PX4-only 版本运行固定目标与随机种子，并在
结果中记录源码提交、飞控版本、策略和实验条件。旧后端结果仅作历史记录。

## 6. 常见故障

开启能量约束后的统一电量与任务优化见 [PX4 能量调度说明](px4_energy_scheduling.md)。
已安装旧 PX4 时再次运行 `bash ./setup_px4_backend.sh`，以应用外部电池时间字段
兼容补丁。检查 `/uav/px4/battery_bridge_status` 为 `SYNCED:paper_energy_model`。

- `/uav/px4/connected=false`：确认 `MicroXRCEAgent` 正在运行，并检查
  `/fmu/out/vehicle_local_position` 和 `/fmu/out/vehicle_status_v1`。
- 找不到 `px4_msgs`：重新执行 `source ./setup_workspace.bash`；该脚本会先加载外部
  `px4_ros_ws`，再加载主工作空间。
- 找不到 PX4 固件：检查 `PX4_AUTOPILOT_DIR`，默认应指向
  `~/design_final_artifacts/px4/PX4-Autopilot`。
- VirtualBox 卡顿：使用 `gui:=false rviz:=false`；不要同时打开 Gazebo GUI 和 RViz。
- 任务被拒绝：确认旧仿真已停止、PX4 连接就绪及定位有效，再发送任务。
