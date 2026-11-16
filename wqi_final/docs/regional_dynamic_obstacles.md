# 当前四车校园的五档空地动态避障

## 1. 五档密度

本功能用于 `campus_regional_delivery/regional_campus.launch.py`，不复用旧校园坐标。
五个有效密度档位之外另保留关闭对照；每个有效档均在 A/B/C 三区配置地面和空中交通。

| 档位 | 参数 | 地面 | 空中 | 总数 |
|---|---|---:|---:|---:|
| 关闭对照 | `none` | 0 | 0 | 0 |
| 1 低 | `low` | 6 | 3 | 9 |
| 2 中 | `medium` | 9 | 6 | 15 |
| 3 高 | `high` | 12 | 9 | 21 |
| 4 很高 | `very_high` | 15 | 12 | 27 |
| 5 极高 | `extreme` | 18 | 15 | 33 |

同一种子下，高档包含低档选中的路线。每区始终有核心交通点，不会随机选完后
某一区没有空中障碍。旧地图最高档仅 2 个空中障碍，新入口最高档为 15 个。

默认 `obstacle_motion:=random`，地面在道路上的二维活动范围内运动，速度上限
约 0.42～0.65 m/s；空中为半径 0.75 m 的实体球，速度上限约 0.75～0.83 m/s。
空中活动中心分散在实际总站至 A/B/C 的航段两侧，分别为 13.5、16.5、15、18、12 m，
各自还可连续上下运动 ±2 m，位置不共线。它们都有物理碰撞体，不是装饰标记。
采用随机目标加相关速度扰动，以物理速度驱动、限制加速度，不逐帧随机修改坐标。
详细公式、范围校验和验收边界见 [随机运动与能量核对](energy_random_verification.md)。

## 2. 随机交通与固定路线对照

随机模式从模型插入时开始运动，不读取机器人位置，不让路、不追赶，也不因
机器人停止而停止。速度连续、加速度有界，运动保持在经过道路、建筑、交接区
净空与障碍之间间距检查的范围内。相同种子可重复伪随机输入，但物理/通信时序
不同不能保证整场机器人轨迹逐点相同。更换种子须重启，避免运行中瞬移。

以下候遇触发行为仅属于 `obstacle_motion:=route` 固定路线对照模式：

障碍先在路线端点候遇。机器人以非零速度接近指定交会点时，触发该障碍开始
固定路线往返。触发距离按名义机器人速度和障碍行程选取，以提高可重复相遇概率。
触发之后不再读取机器人位置，不追赶、不让路，也不靠近机器人就暂停。
端点减速/反向是交通往返，不是避让机器人。
纵向道路交通在停车端点前 8 m 提前触发，避免“车辆先被停着的障碍挡住，
却必须驶过它才能触发”的死锁；部分路线反向初始化，交通方向不完全相同。

这是实验场景激活机制，不是机器人避障算法。场景触发和诊断可读取 Gazebo 真值；
UGV 避障只读取自己的激光/位姿及传感器跟踪结果，UAV 避障只读取自己的三维
点云/测距/位姿。障碍真值话题不输入导航器，不冒充基于传感器的检测结果。

候遇点和交通路线避开车顶停靠区、建筑入口及固定交接位置，给楼层起降保留净空。
起飞/着陆仍要求车辆静止，不新增移动车辆着陆。
不能保证每一轮与每一个障碍都相遇；安全改道避开冲突本身也是正常导航结果。

## 3. 完整启动与发单

先关闭旧仿真。终端一，同时打开四车、PX4 UAV、九电池、最高档交通及两个窗口：

```bash
cd ~/design_final/wqi_final/simulation_ws
bash ./build_workspace.sh --executor sequential
source ./setup_workspace.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch campus_regional_delivery regional_campus.launch.py \
  enable_energy_constraints:=true enable_dynamic_obstacles:=true \
  obstacle_density:=extreme obstacle_seed:=42 obstacle_motion:=random gui:=true rviz:=true
```

需要其他档位只修改 `obstacle_density`。不考虑电量时用
`enable_energy_constraints:=false`，不会改变障碍配置或规避传感器输入。
恢复原静态无障碍模式时关闭旧仿真，使用默认入口，不携带动态参数。
如果要做同一算法的零密度对照，使用 `enable_dynamic_obstacles:=true obstacle_density:=none`，
这样仍加载动态规划器，只是不生成交通。不要把默认静态规划器和动态规划器的差异归因于密度。
场景密度/种子须重启生效，不提供运行中随机重置障碍坐标的服务。

终端二，source 同一工作空间并设置相同 RMW 后，检查真实生成结果：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /dynamic_obstacles/manifest std_msgs/msg/String --once \
  --qos-durability transient_local
ros2 topic echo /regional_delivery/energy_readiness std_msgs/msg/String --once \
  --qos-durability transient_local
ros2 topic echo /uav/px4/battery_bridge_status std_msgs/msg/String --once \
  --qos-durability transient_local
```

最高档须满足 `requested=33`、`spawned=33`、`failures=[]`；另外能量就绪为 READY，
PX4 电量桥为 SYNCED。生成器逐个等待 Gazebo 确认，避免大量并发生成丢请求，
因此极高档比低档更晚就绪。未就绪时任务管理器拒单并在 `task_admission` 解释原因。
最高档联合启动专项虽已通过，但加载期间曾发生能量遥测过期、关闭时部分进程被
强制结束。首次完整配送建议先用 `low`，确认持续 READY 后发单，再逐档增加。
不要禁用遥测新鲜度检查来强行发单；最高档完整配送与稳定退出仍是待验收项。

终端三，source、设置 RMW 后发送教学楼三楼任务，经过地面核心交通和空中交叉航段：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 action send_goal /regional_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['teaching_building'], target_floors: [3], payload_masses_kg: [0.30], return_home: true}" \
  --feedback
```

用 `cafeteria` 二楼或 `dormitory_1` 三楼可分别检查 C、B 航路。
创新中心紧邻 A 区车站，该短订单不保证遇到地面横穿，不能据此认定整个场景没有动态避障。
多件仍用相同长度的楼栋、载荷、楼层数组；地面卸货不填写楼层数组，而不是填 0。

纯 UGV 对照可在关闭上一套后启动：

```bash
ros2 launch campus_regional_delivery regional_campus.launch.py \
  delivery_mode:=ugv_only enable_energy_constraints:=true \
  enable_dynamic_obstacles:=true obstacle_density:=extreme gui:=true rviz:=true
```

纯 UGV 不生成 UAV，因此不是空中避障测试。随机模式的空中障碍仍连续运动；
固定路线对照模式中的空中候遇障碍不会被 UAV 激活。
最高档增加实际模型和传感器负载，VirtualBox 下可能明显降低实时率。墙钟耗时不能
当作仿真配送耗时，两种时钟和相同任务窗口应在正式结果中分别记录。

## 4. Gazebo 与 RViz

- Gazebo 地面为黄色实体柱，空中为红色实体球，随机模式生成后持续运动。
- RViz 的 `Dynamic Ground and Air Traffic` 显示真实位置和淡色随机活动范围线框，
  不是把直线当成随机轨迹。固定路线模式才显示空中交通线。
- UAV 灰线是名义区域航段，黄色 `UAV Sensor Replanned Route` 是传感器重规划路径，
  `UAV Actual Flight Path` 是实际飞行轨迹。不要把灰线当成实时避障结果。
- 每辆 UGV 保留各自的 Nav2 黄色路径、激光、局部和全局代价地图；不共用动态跟踪器。
  默认显示 `UGV-01 UGV Global Costmap`，其他三车可在 Displays 勾选；建议每次只显示一层全局地图。
- Gazebo 雷达蓝色射线仍由 `visualize_sensor_rays` 控制，默认关闭示意但传感器运行。

15 m 是巡航基准，不是禁止改变的高度。UAV 三维绕行可短暂升降，并回到目标航段。
不通过固定转角、固定后退距离或固定等待时长控制机器人绕障。

## 5. 算法与安全

UGV 动态模式复用现有 D* Lite 增量全局规划和带 TTC/速度预测项的 DWB 局部规划，
通过四套独立预测器、话题和行为树接入。路径截断节点显式使用各车的 base_link
前缀，避免误查根 TF 后反复重发路径。无动态场景仍使用原 SmacPlanner2D＋RPP。
保留原最高速度 0.40 m/s、碰撞监控、物理包络和道路禁行掩膜。
核心横穿在既有六米宽道路上，不堵住四米接驳支路与道路圆头的狭窄交界。

新入口用 3 秒控制展开、8 秒碰撞预测；3 秒之后按轨迹末端停车位置检查余下
时窗，不冒充八秒恒速运动预测。0.30 m 安全余量大于约 0.124 m 的名义制动距离，
但不是实际制动标定。关闭提前投影终点朝向的 `GoalAlign` 软评分，低速朝向辅助
使用 4 m 路径前瞻，避免过度追随近处栅格折线；不是固定避让角度或动作。

附近存在动态跟踪且距目标大于 0.6 m 时，加入有界的目标推进量软评分，按候选
轨迹终点到目标的距离改善评价，而不是奖励绕圈或电机转动。只在无预测碰撞候选
内部偏好推进，不覆盖碰撞等级，也不强制最低行驶速度。

原 TTC 模型在已进入软包络后让所有候选 TTC=0，提前返回又忽略了后续侵入。
现继续计算未来最小间距，并在同一零 TTC 等级内比较归一化侵入积分，优先脱离
而非继续深入。普通正 TTC 安全等级不变。此项是项目改进，不宣称为论文原式或
形式化安全证明。

地面全局层只保留原来经过限制的短时动态占用，不恢复早先很长的未来碰撞条带；
局部代价地图仍基于真实扫描，速度预测由局部轨迹评分处理。

UAV 的新动态入口使用传感器三维局部体素图搜索：滤除地面和自身、近表面体素采样，
按消息采样时刻查询 TF 后变换到 map，以 SciPy KD-tree 检查障碍间距，
在 0.75 m、26 邻接的局部体素图上调用 SciPy Dijkstra 求路，再做视线简化和
整条路径/当前指令段校验。仍然安全的既有路径继续使用，避免反复换侧；没有安全路径时
悬停等待，不执行固定避让动作，也不在无路可走时强行穿越。

当前动态入口采用 12 m 预警和 3 m 表面路径缓冲，给运动/跟踪误差留出空间。
空中仍是局部反应式三维规划，不是全局三维占据地图、完整三维速度跟踪器或最优算法。
静止平台起降保护模式仍保留自己的传感器安全逻辑；交会点不安排在平台起降竖线上。
前后斜下测距外移至 x=±0.28 m，下视测距及相机移至 z=-0.06 m，避免射线穿过
PX4 原有 0.47×0.47×0.11 m 机体碰撞盒。Gazebo/URDF/安全监测高度同步修改，
没有修改机体质量、碰撞尺寸、电机或 PX4 动力学。

传感器超时在新动态模式按仿真时间判断，点云另有墙钟看门狗；低实时率不应被当成
传感器每隔一帧失联。丢失新鲜点云/准确 TF 时不复用过期位置继续飞行。

## 6. 状态与证据

```bash
ros2 topic echo /regional_delivery/dynamic_status std_msgs/msg/String --once \
  --qos-durability transient_local
ros2 topic echo /ugv_01/ugv/dynamic_replanning/status
ros2 topic echo /uav/dynamic_replanning/status std_msgs/msg/String \
  --qos-durability transient_local
```

`moving_actor_encounters` 是进入诊断范围的不同移动障碍数量，不是成功避障次数。
`planner_messages` 也不等于成功绕行次数。`actors_with_physical_contact` 来自各障碍
独立的 Gazebo 接触传感器；包络净间距是几何近似，不把近距离直接算成物理碰撞。
诊断使用真值，只负责显示/记录，不参与避障决策。

本轮具体构建、单车、独立 PX4、最高密度部署结果和失败修复记录见
[`evaluation_report.md`](evaluation_report.md)。不把独立专项通过扩写成四车＋九电池＋
最高密度整程、全部区域楼层或统计成功率已经验收。

## 7. 文件职责

- `campus_regional_delivery/dynamic_scenario.py`：当前地图的五档路线与触发配置。
- `campus_regional_delivery/dynamic_monitor.py`：真值显示、相遇和物理接触记录。
- `campus_regional_delivery/launch/regional_campus.launch.py`：可选启动及独立预测器/BT 接线。
- `campus_regional_delivery/fleet.py`：动态 Nav2 参数与 RViz 显示适配，静态默认保持。
- `campus_dynamic_obstacles/scenario.py`：分区、可重复、逐档包含的路线选择。
- `campus_dynamic_obstacles/scripts/dynamic_obstacle_spawner`：顺序生成、真实数量和独立接触节点。
- `campus_dynamic_obstacles/src/dynamic_obstacle_plugin.cpp`：一次触发后固定物理速度往返。
- `uav_control/local_cloud_guard.py`：多表面点云过滤、采样时刻变换、局部体素图搜索与路径检查。
- `uav_control/flight_action_server.py`、`safety_monitor.py`：可选安全路径检查和点云接口。

实验探针、原始记录和绘图均在仓库外 `design_final_artifacts/regional_dynamic/20261112/`，
没有将自动实验脚本加入毕业设计源码。

最终专项、最高档启动检查、全工作空间测试和未验收项见
[本阶段验收记录](regional_dynamic_acceptance.md)。只到点但发生接触仍判失败。
