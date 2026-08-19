# 校园空地协同配送实验方法

## 实验目标

本文件定义毕业论文可重复使用的定量实验流程。实验回答四个问题：

1. UGV、UAV 和空地协同三种模式的配送时间与路径长度有何差异。
2. UAV/UGV 载荷、三电池约束和停靠充电如何影响任务可执行性与总能耗。
3. 不同动态障碍密度下，系统的成功率、恢复次数和局部重规划次数如何变化。
4. 仿真速度变化时，任务结果是否仍具有可重复性。

实验只评估 ROS 2 Humble、Gazebo Classic 中的电脑端仿真。电量是功率模型积分值，
不是实际电池或 ESC 遥测值。

## 被测系统与算法

| 模式 | 路径规划与控制 | 主要能量数据 |
|---|---|---|
| `ugv_only` | 静态阶段使用 Nav2 `SmacPlanner2D + Regulated Pure Pursuit`；动态阶段使用 D* Lite 增量全局规划、DWB 加速度约束轨迹采样和预测碰撞评价 | UGV 速度/转向/载荷功率积分 |
| `uav_only` | 15 m 航路图 Dijkstra、3D 雷达前向冲突检测、切向绕行、阻塞边重规划和有界局部三维绕行 | UAV 推进、载荷和辅助设备功率积分 |
| `cooperative` | UGV 道路层规划与 UAV 局部配送层调度，固定会合点起降和停靠充电 | UAV 飞行、UGV 驱动、UGV 充电源三类能耗 |

UGV 动态阶段由 360 度激光聚类生成带地图坐标位置、速度、半径和置信度的结构化
障碍轨迹。D* Lite 只修复代价变化影响的网格；DWB 的自定义预测碰撞评价器计算每条
候选轨迹与运动障碍在局部预测时域内的预计碰撞时间和最小净空。该链路不使用固定
倒车、固定转角、固定等待或由障碍物主动让行。

UAV 的在线处理属于传感器驱动的局部反应式避障：顶部 3D 雷达在当前目标方向检测到
近距离冲突时，先锁定一个切向绕行点，安全越过后继续原始目标；持续阻塞时再禁用
当前航路边并重新运行 Dijkstra。没有可用图路径时，选择满足高度和净空约束的局部
三维绕行点。该实现不是未知环境中的完整三维 SLAM 或全局占据栅格规划。

## 自变量与控制变量

正式对比使用以下自变量：

- 配送模式：`ugv_only`、`uav_only`、`cooperative`。
- 动态障碍密度：`none`、`low`、`medium`、`high`。
- 随机种子：`42`、`43`、`44`。
- 单目标场景：`teaching_building`。
- 多目标场景：`multi_target_regression`。
- 初始 UAV 飞行电池 SOC：正常任务使用 `0.80`，储备阈值为 `0.20`。
- 初始 UGV 驱动电池 SOC：正常任务使用 `0.80`，储备阈值为 `0.20`。
- 初始 UGV 充电电池 SOC：正常任务使用 `0.80`，储备阈值为 `0.10`。

每个对比组保持校园 world、起点、目标、载荷、控制参数和最大任务超时一致。关闭
Gazebo GUI、RViz 和传感器射线，只保留物理、传感器和 ROS 节点，以降低 VirtualBox
图形负载造成的偏差。

主矩阵为 `3 种模式 x 4 种密度 x 3 个种子 = 36 次运行`。三个种子构成每个组合的
三次独立重复。多目标和低电量拒绝实验单独运行，不与单目标效率表混合。

主矩阵之外必须保留三类 UGV 定向冲突回归：正面迎面、侧向横穿、斜后方追赶。
每类至少运行 3 次，障碍速度应高于或接近 UGV 速度，并同时记录相对闭合速度、预计
碰撞时间、路径改变时刻、最小物理净空和最终 Action 状态。这样可以证明安全结果来自
速度感知的实时规划，而不是障碍路线恰好没有与机器人相交。

## 动态障碍定义

`campus_dynamic_obstacles` 通过 Gazebo ModelPlugin 对带碰撞体的实体施加线速度，
不使用连续坐标瞬移。障碍物只读取自身固定路线、速度和边界，不读取机器人位置、
不主动减速且不执行让行；机器人 ground truth 只由独立评测器读取，用于事后计算
净间距和碰撞。配置固定路线并由种子决定选择顺序：

| 密度 | 地面障碍 | 空中障碍 | 总数 |
|---|---:|---:|---:|
| `none` | 0 | 0 | 0 |
| `low` | 3 | 0 | 3 |
| `medium` | 5 | 1 | 6 |
| `high` | 8 | 2 | 10 |

## 指标定义

对每次运行记录以下原始量：

- 成功与失败原因、完成目标数。
- 仿真时间、墙钟时间和实时因子 `RTF = T_sim / T_wall`。
- UGV/UAV 实际路径长度，即连续里程计点欧氏距离之和，并过滤异常跳变。
- 配送点误差，即发布 `DELIVERED` 事件时机器人位置与目标位置的欧氏距离。
- Nav2 恢复次数、UAV 阻塞边重规划次数和安全悬停次数。
- UGV 黄色全局路径变化次数、横向绕行方向反转次数和停滞时间。
- D* Lite 每次规划耗时、扩展节点数、代价更新数和是否复用既有搜索状态。
- 预测式 DWA 的活动障碍轨迹数、被选轨迹预计碰撞时间和最小预测净空。
- UAV 最小障碍净空。该指标只在 `CRUISE`、`RETURNING` 和 `APPROACH` 阶段统计，
  不把正常起飞或降落时接近地面计为障碍。
- 每个动态障碍物的 Gazebo ground-truth 里程计、UGV 二维表面净间距和 UAV 三维
  表面净间距。协同 UGV、独立 UGV、UAV 包络半径分别为 `0.60 m`、`0.22 m`、
  `0.56 m`，障碍物半径来自同一份场景 YAML。
- UGV/UAV 碰撞事件数和 `collision_free`。净间距小于等于 0 视为包络相交；一次
  持续接触只计一个事件，分离超过 `0.05 m` 后再次接触才计新事件。
- UAV 消耗/充入电量，以及 UAV 初始/结束 SOC。
- UGV 驱动能耗、充电源能耗，以及两块 UGV 电池的初始/结束 SOC。
- 系统总能耗；扣除 UAV 已充入电量，避免与 UGV 充电源消耗重复计数。

汇总指标计算如下：

```text
success_rate = successful_runs / total_runs
clearance = center_distance - robot_radius - obstacle_radius
avoidance_success_rate = collision_free_runs / total_runs
delivery_rate = completed_targets * 60 / simulated_duration_s
energy_per_target = total_energy_wh / completed_targets
endpoint_error = ||actual_delivery_position - configured_target_position||
```

任务 Action 成功但发生动态障碍碰撞时，本次运行最终仍标记为失败。这样任务成功率
不会掩盖碰撞，避障成功率也不再由“是否到达终点”间接代替。

每组报告均值和样本标准差，同时保留单次原始记录。三次重复只能用于展示初步离散
程度，不将其解释为大样本统计结论。

## 执行命令

先构建并加载工作空间：

```bash
cd ~/design_final/wqi_final/simulation_ws
bash ./build_workspace.sh
source ./setup_workspace.bash
```

运行单组教学楼协同实验三次：

```bash
ros2 launch delivery_evaluation experiment.launch.py \
  mode:=cooperative scenario:=teaching_building \
  obstacle_density:=medium repetitions:=3 random_seed:=42 \
  initial_battery_percentage:=0.80 \
  initial_ugv_drive_battery_percentage:=0.80 \
  initial_ugv_charging_battery_percentage:=0.80 \
  results_dir:=$WQI_BUILD_ROOT/experiment_results gui:=false rviz:=false
```

运行完整 36 次单目标实验矩阵：

```bash
ros2 run delivery_evaluation experiment_matrix \
  --modes ugv_only,uav_only,cooperative \
  --densities none,low,medium,high \
  --seeds 42,43,44 \
  --scenario teaching_building --repetitions 1 \
  --initial-battery 0.80 \
  --initial-ugv-drive-battery 0.80 \
  --initial-ugv-charging-battery 0.80 \
  --results-dir "$WQI_BUILD_ROOT/experiment_results" \
  --continue-on-failure
```

先检查命令而不启动仿真：

```bash
ros2 run delivery_evaluation experiment_matrix \
  --modes ugv_only,uav_only,cooperative \
  --densities none,medium --seeds 42 --dry-run
```

## 输出文件

每个批次自动生成：

- `runs.json`：完整机器可读原始数据。
- `runs.csv`：论文表格和后续统计输入。
- `summary.md`：任务成功率、真实动态避障成功率、效率、误差、能耗和碰撞汇总。
- `success_rate.png`、`avoidance_success_rate.png`、`phase_duration.png`、
  `energy_comparison.png`、`ugv_path_length.png`、`uav_path_length.png`：
  自动生成图表。

矩阵执行器还会在矩阵根目录合并所有批次，避免人工复制数据时改变指标口径。

## 验收与论文使用

无动态障碍基准至少应满足：任务连续三次成功、UAV 配送误差不超过 `0.5 m`、UGV
停靠误差不超过 `0.5 m`、UAV 无碰撞且返航后状态正确。动态障碍实验不预先伪造
成功率阈值，应报告各密度实测任务成功率、真实避障成功率、最小净间距、碰撞事件、
失败原因和重规划次数。低电量组必须在车辆移动前拒绝不可安全返航的任务；重新停靠
后应观测到充电量增加。

最终论文只引用仓库中保留的原始结果和生成图，不用单次烟雾测试代替正式矩阵。
