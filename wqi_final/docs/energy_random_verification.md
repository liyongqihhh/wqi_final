# 九电池能量一致性与连续随机障碍

## 1. 实现范围

当前区域校园使用一个 UAV 电池和四辆 UGV 各自的驱动、充电电池，共九个有限电池包。
任务前预算检查、途中余量保护、正确载荷归属、物理停靠门控和有限源充电已经接入。
`enable_energy_constraints:=true` 开启；关闭时保留无能量约束基线。

这是公式驱动的仿真能耗，不是 PX4 实测电流或真实电池标定。实际 PX4/Gazebo
里程计提供速度与加速度，模型按仿真时间积分；载荷作为能耗模型输入，不等于
给 PX4 机体实时改变惯量。长期动态绕障等待没有有限上界，名义预算不保证一定完成任务。

## 2. 数学约定

UAV 水平推进采用 [Zeng 旋翼功率模型](https://arxiv.org/abs/1804.02238)，
并加入 [Dai 动态推重比修正](https://arxiv.org/abs/2202.08486)。
垂直部分参考 [Gong 多旋翼垂直飞行模型](https://arxiv.org/abs/2209.04128)，
按本机总旋翼盘面积、质量、垂直加速度和阻力设置推力。并非直接照搬论文样机参数。

记 `m = m_airframe + m_sensor + m_payload`，`W = mg`，总旋翼面积为 `A`：

```text
Pi = (1+k) W^(3/2) / sqrt(2 rho A)
v0 = sqrt(W / (2 rho A))
Ph(V) = P0 (1+3 V^2/Utip^2)
      + Pi kappa sqrt(sqrt(kappa^2+V^4/(4 v0^4))-V^2/(2 v0^2))
      + 0.5 d0 rho s A V^3
T = max(0.05 W, W + m az + 0.5 rho Sz vz |vz|)
Pv = P0 + (1+k) T/2 [vz + sqrt(vz^2 + 2 T/(rho A))]
Phover = P0 + Pi
Pprop = max(0.2 Phover, Ph + Pv - Phover)
Pbattery = (Pprop + Pcomputer + Plidar + Pcamera + Pcommunication) / eta_discharge
E_next[Wh] = E[Wh] - Pbattery[W] dt[s] / 3600
```

`Ph + Pv - Phover` 是本项目采用的分量叠加近似，不宣称为完整三维空气动力学；
低速下降涡环状态等未建模。上下升降的瞬时功率由速度、加速度、载荷决定，不能
武断要求所有下降工况一定比巡航耗电更多。当前参数总表在 `uav_control/config/battery_model.yaml`。

UGV 质量含车体、当前车载货物，以及停靠时的 UAV 本体和仍在机上的货物，包裹不重复计入：

```text
Pdrive = Pidle + (Crr m g |v| + 0.5 rho CdA |v|^3 + max(0,m a_parallel |v|)) / eta_drive
       + kv |v| + kw |omega|
Edrive_next = Edrive - Pdrive dt / 3600
Psource = Pcharger_idle + Pinput / eta_transfer
EUAV_next = EUAV + (eta_charge Pinput - PUAV_docked_idle) dt / 3600
Esource_next = Esource - Psource dt / 3600
```

驱动、充电源和 UAV 剩余能量均有限；充电源不越过保留量，UAV 不超出容量/充电目标。
充电只在停靠锁定、几何位置正确且 UAV 未飞行时发生。损耗留在源电池账目中，不能
把 UAV 获得的能量直接等同于 UGV 充电电池减少量。

## 3. 已执行的数值核对

2026-11-16 独立公式测试覆盖 24 个 UAV 飞行/载荷工况、12 个四车驱动/充电工况，
另有混合充放电至电池耗尽和人为能量注入检查，共 38 项。
隔离 ROS 域中运行真实能量节点，发布合成里程计和时钟，收到全部九个 BatteryState：

| 工况，10 仿真秒 | 公式预期变化 | 节点结果 |
|---|---:|---:|
| UAV 无载悬停 | -0.683836383 Wh | -0.683836383 Wh |
| UAV 停靠充电 | +0.436111111 Wh | +0.436111111 Wh |
| 供电车充电源电池 | -0.561111111 Wh | -0.561111111 Wh |
| 每辆 UGV 静止待机驱动电池 | -0.116666667 Wh | -0.116666667 Wh |

上述隔离 ROS 核对的最大变化误差约 `1.04e-12 Wh`，50 条发布状态的最大收支残差
约 `9.55e-12 Wh`，是浮点舍入量级。它是合成遥测节点验证，不是一次实飞配送。
原始报告在仓库外 `design_final_artifacts/random_energy/20261116/live_energy_reference.json`。

在线 `/regional_delivery/energy_status` 新增：

```text
system_net_consumed_wh = initial_total - current_total
ledger_net_consumed_wh = sum(drive_consumed + charging_source_consumed)
                       + UAV_consumed - UAV_charged
energy_balance_error_wh = system_net_consumed_wh - ledger_net_consumed_wh
energy_balance_ok = |energy_balance_error_wh| <= 1e-6 Wh
```

收支不一致会记录错误并将能量状态标记为未就绪，阻止新任务绕过能量检查。
这项守恒核对不验证物理参数是否已标定，也不代表调度全局最优。

## 4. 障碍随机运动

当前四车入口默认 `obstacle_motion:=random`，旧模式用 `obstacle_motion:=route`。
最高档不减量，仍有 18 个地面与 15 个空中物理障碍。

- 运动模型：随机目标方向叠加有记忆的 Ornstein-Uhlenbeck 速度扰动。
- 噪声每 0.1 仿真秒更新，相关时间 2.5 秒，标准差为速度上限的 18%。
- 随机目标从允许范围内部 80% 的范围采样；1.5～4.5 秒或靠近目标时重新抽取。
- 速度向量限幅，最大加速度 1.2 m/s²；靠近范围边缘根据制动距离限制向外速度。
- 只调用物理速度接口，不调用 `SetWorldPose`、`/set_entity_state` 或逐帧随机坐标。
- 地面固定高度、XY 随机；空中 XYZ 随机。每区五个中心高度为 13.5、16.5、15、18、12 m，
  每个允许上下移动 ±2 m，水平位于航线两侧，不共线。
- 随机模式从插入后运动，不读取机器人位置，不主动避让机器人；固定对照模式保留原候遇触发。
- 由场景种子及障碍名称派生独立随机种子，密度/生成顺序不改变同名障碍的种子。
- 整个活动范围检查道路、建筑、配送停靠区净空及障碍之间距离；不只检查起点。

无外界碰撞时控制器保持范围、速度和加速度约束；若被机器人碰撞产生外部冲量，不能
把控制器的平滑性假设冒充碰撞之后仍严格平滑。固定种子也不保证不同物理步长、编译器
或通信时序下整场轨迹完全相同。新随机模式完整配送和多种子避障成功率仍须单独实测。

### 原生 Gazebo 检查

2026-11-16 在最高档、种子 42、250 Hz 物理、Gazebo＋RViz 同开条件下，没有发送配送任务，
收集完整生成后 35 仿真秒的数据（含启动约 412 墙钟秒）。结果：

- `requested=33`、`spawned=33`、`failures=[]`，全部实际运动检查通过。
- 18 个地面轨迹均非直线；15 个空中轨迹均有 XYZ 变化，采样高度变化范围为 1.45～2.69 m。
- 最大检测范围越界为 0 m；采样速度在各自上限内，最大采样加速度约 1.20 m/s²。
- 收到四个 `map` 坐标系、900×800、0.2 m 分辨率的全局代价地图。
- 99 条实际场景能量状态均收支平衡，最大残差约 `2.54e-12 Wh`，结束时能量就绪。

原始轨迹、配置、日志及 `native_motion_audit.json` 在仓库外
`design_final_artifacts/random_energy/20261116/campus_view/`。这些是运动/话题检查，
不是完整避障配送成功。日志仍出现过 PX4 锁步超时/预飞垂直速度提示和迟到 TF 消息，
在高负载机器上仍需逐单检查预飞、传感器和调度就绪。

RViz 首次接收地图出现过一次 `active samplers with a different type` 着色器日志；
四个地图话题仍持续更新。该类首帧错误在 [RViz 上游问题](https://github.com/ros2/rviz/issues/463)
有记录，不能仅据此判断地图完全无法显示，也不能把收到了消息等同于已人工确认画面。
本轮随后由用户人工确认“地图与障碍显示正常”；此确认仅覆盖当前显示效果，
不代表随机障碍整程配送或其他图形驱动组合已经通过验收。
若画面空白或冻结，应先使用默认 `rviz_render_driver:=softpipe`，并提供截图排查。

同时运行仿真时，应把代码测试放在隔离 ROS 域，避免默认域与实际节点互扰：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
ROS_DOMAIN_ID=135 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp bash ./test_workspace.sh --executor sequential
```

本轮确有一次同域并发回归的既有 `test_peer_vehicle_layer` 退出卡住，未生成结果。
原始错误日志已保留在外部证据目录；该五项测试在隔离 Cyclone DDS 域中复测通过。
这证明隔离运行可通过，不宣称已经定位所有 DDS 退出卡滞的内部原因。

最终 16 包全工作空间隔离回归：**996 条记录，0 错误、0 失败、26 跳过**。
其中包含 CTest 包装记录，不能将该数字理解为配送次数。

## 5. 启动与观察

先关闭旧仿真。终端一：

```bash
cd ~/design_final/wqi_final/simulation_ws
bash ./build_workspace.sh --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source ./setup_workspace.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch campus_regional_delivery regional_campus.launch.py \
  execution_profile:=fast_test gui:=true rviz:=true rviz_render_driver:=llvmpipe \
  enable_energy_constraints:=true enable_dynamic_obstacles:=true \
  obstacle_density:=extreme obstacle_seed:=42 obstacle_motion:=random
```

RViz 默认打开 `UGV-01 UGV Global Costmap`，主题为 `/ugv_01/global_costmap/costmap`。
其他三车在 Displays 中勾选，建议先取消上一辆，避免四张重叠。局部地图、黄色规划线、
真实 UAV 轨迹与淡色随机活动范围线框仍保留。UAV 使用三维局部规划，不伪装成 Nav2 二维全局代价地图。

终端二：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /dynamic_obstacles/manifest std_msgs/msg/String --once --qos-durability transient_local
ros2 topic echo /regional_delivery/energy_status std_msgs/msg/String --once --qos-durability transient_local
```

确认 `spawned=33`、`failures=[]`、`motion_mode=bounded_random`，能量 `ready=true`、
`energy_balance_ok=true`，并等待 PX4 电量桥 `SYNCED` 后才能发单。

终端三，实验楼三楼往返（此命令不是本次已经完成该配送的声明）：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 action send_goal /regional_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['laboratory'], target_floors: [3], payload_masses_kg: [0.30], return_home: true}" --feedback
```
