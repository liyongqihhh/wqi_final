# 下一阶段任务书：协同配送实验评估与论文数据自动化

> 本文件可以直接作为下一次 Codex 开发任务输入。执行者必须先阅读当前源码和
> 已有日志，再在当前工作树基础上增量开发，不得覆盖已经调好的地图、导航参数、
> UAV 飞控和 UGV-UAV 对接逻辑。

## 一、项目环境

- 仓库：`/home/wqi/design_final/wqi_final`
- ROS2 工作空间：`/home/wqi/design_final/wqi_final/simulation_ws`
- 环境：Ubuntu 22.04、ROS2 Humble、Gazebo Classic、VirtualBox
- 源码修改范围：`simulation_ws/src/`
- 设计文档允许写入：`docs/`
- 实验结果允许写入：`simulation_ws/experiment_results/`
- 禁止手工修改：`build/`、`install/`、`log/`
- 不得破坏或重命名现有 `ugvcar_*`、`uav_*`、`cooperative_delivery*`
  软件包。
- 不得使用 `/set_entity_state` 连续修改车辆坐标来伪造运动或重置实验。
- 所有实验节点必须设置 `use_sim_time:=true`。

## 二、当前已确认基线

1. UGV 能在校园地图中运行 Nav2，当前稳定巡航速度为 `0.22 m/s`。
2. UGV 使用 Gazebo ground truth 对齐 `map`，Gazebo 与 RViz 的位置一致。
3. UAV 能独立完成起飞、悬停、固定航路巡航、楼层配送、返航和降落。
4. UAV 已有 3D 激光雷达、下视相机、下向测距、斜下补盲、IMU 和安全球。
5. 联合系统能让 UAV 固定在 UGV 平台上运输，到建筑门口后解锁起飞，完成
   楼层配送后返航并重新对接。
6. 协同任务已支持单目标、多目标和四栋寝室的独立 UAV 配送点。
7. UAV 已按飞行阶段模拟耗电，支持往返电量准入、安全余量和 UGV 停靠充电。
8. 当前仍缺少系统化重复实验、完整的整程回归记录、路径与阶段统计、UGV 能耗估算
   以及可以直接用于毕业论文的对比表格。

本阶段将以上状态视为冻结基线。除非运行日志和可复现测试明确证明控制器存在
错误，否则不得修改 UGV 速度、Nav2 控制器、代价地图、校园道路、UAV PID、
航路点、碰撞模型或对接插件参数。

## 三、本阶段最终目标

新增一个独立的 `delivery_evaluation` 软件包，建立可重复的校园配送实验体系：

1. 自动执行 UGV 单独配送、UAV 单独配送和 UGV-UAV 协同配送三种模式。
2. 自动进行指定次数的重复实验，并在每次实验前检查初始条件。
3. 自动采集任务时间、路径长度、终点误差、阶段耗时、恢复次数、避障事件、
   对接状态和成功/失败原因。
4. 复用现有 UAV 电池累计话题，并根据可配置模型估算 UGV 和系统总能耗。
5. 将每次实验写入 CSV 和 JSON，失败实验也必须保留完整记录。
6. 自动生成 Markdown 汇总报告和论文可用的对比图表。
7. 完成教学楼三种模式的重复对比实验，以及一次完整多目标协同任务。
8. 保证现有 UGV、UAV 和协同启动命令继续可用。

本阶段的重点是“实验可重复、数据可追溯、结论可用于论文”，不是再次重写车辆
控制算法。

## 四、实验问题与对比原则

需要回答以下三个毕业设计问题：

1. 三种模式完成校园配送时，任务时间和移动距离有什么差异？
2. UAV 直接配送与 UGV 携带 UAV 协同配送的估算能耗有什么差异？
3. 协同系统能否可靠完成“UGV 到门口、UAV 到楼层、UAV 回收、UGV 返航”
   的闭环？

三种模式的服务终点必须在报告中明确区分：

| 模式 | 起点 | 配送终点 | 返航要求 |
|---|---|---|---|
| `ugv_only` | 物流中心 | 建筑门口 UGV 停止点 | 返回物流中心 |
| `uav_only` | 物流中心 UAV 起降点 | 建筑指定楼层 UAV 配送点 | 返回并降落 |
| `cooperative` | UAV 已对接的物流中心 UGV | UGV 到门口后 UAV 到指定楼层 | UAV 重新对接且 UGV 返回物流中心 |

UGV 无法到达楼层，因此 `ugv_only` 是地面配送基线，不得把它描述成与楼层配送
完全相同的服务。报告需要同时给出原始指标和“每个成功配送任务”的归一化指标。

## 五、正式实验矩阵

先运行快速冒烟测试，再执行正式实验：

| 编号 | 模式 | 目标 | 重复次数 | 用途 |
|---|---|---|---:|---|
| S0 | `cooperative` | `teaching_building` | 1 | 验证采集和报告链路 |
| E1 | `ugv_only` | `teaching_building` | 3 | 地面配送基线 |
| E2 | `uav_only` | `teaching_building` | 3 | 空中配送基线 |
| E3 | `cooperative` | `teaching_building` | 3 | 协同配送可靠性与对比 |
| E4 | `cooperative` | `laboratory,library,dormitory_2` | 1 | 多目标整程回归 |

正式结果中 S0 不计入统计。E1-E3 必须使用同一地图、同一目标配置和相同仿真
物理步长。所有正式实验默认 `gui:=false rviz:=false`，避免 VirtualBox 图形负载
改变实时因子。需要观察时另做演示运行，不得混入正式统计。

## 六、必须采集的指标

每次实验至少记录：

| 字段 | 说明 |
|---|---|
| `run_id` | 唯一实验编号 |
| `mode`、`scenario`、`repetition` | 模式、场景和重复序号 |
| `targets` | 目标数组 |
| `success`、`failure_reason` | 最终结果和原始失败原因 |
| `sim_duration_s`、`wall_duration_s` | 仿真时间和真实时间 |
| `real_time_factor` | `sim_duration_s / wall_duration_s` |
| `phase_durations_s` | 各任务阶段耗时 |
| `ugv_path_length_m` | 根据 `/ground_truth/odom` 积分的 UGV 路径长度 |
| `uav_path_length_m` | 根据 `/uav/odom` 积分的 UAV 三维路径长度 |
| `ugv_endpoint_error_m` | UGV 与配置停止点的二维误差 |
| `uav_endpoint_error_m` | UAV 与配送点的三维误差 |
| `nav_recovery_count` | Nav2 恢复次数 |
| `uav_safety_hold_count`、`uav_safety_hold_s` | UAV 安全悬停次数和时间 |
| `minimum_uav_clearance_m` | 飞行期间最小障碍物距离 |
| `detach_success`、`redock_success` | UAV 解锁和重新对接结果 |
| `ugv_energy_wh`、`uav_energy_wh`、`total_energy_wh` | 可配置模型的估算能耗 |
| `completed_targets` | 实际完成目标数 |

路径积分必须按消息时间戳排序，忽略时间倒退、重复样本和明显的定位跳变。UGV
优先使用 `/ground_truth/odom`，UAV 使用 `/uav/odom`。能耗字段必须使用
`estimated` 或 `估算` 描述，不得宣称它们来自真实电池电流传感器。

## 七、能耗数据要求

UAV 能耗不得重新建立第二套模型。必须直接采集
`/uav/battery_consumed_wh`、`/uav/battery_charged_wh` 和
`/uav/battery_power_w`，同时记录任务开始和结束的 `/uav/battery_state`。
UAV 模型参数来自 `uav_control/config/battery_model.yaml`。

只为 UGV 新建 `ugv_energy_model.yaml`，所有系数可配置且包含单位。不得把论文
参数散落硬编码在 Python 文件中。

建议模型：

```text
UGV power = idle_w
          + linear_w_per_mps * abs(v)
          + angular_w_per_radps * abs(w)

energy_Wh = integral(power_W * dt_s) / 3600
```

UAV 对接待机和充电已经由电池节点处理。评估包只读取累计值，不得自行重复积分
UAV 功率。UGV 模型需要支持 `payload_mass_kg` 参数和载荷修正系数，但本阶段
不要求新增会影响飞控稳定性的实体包裹模型。报告必须列出系数、公式和局限性。

## 八、建议新增文件

```text
simulation_ws/src/delivery_evaluation/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/delivery_evaluation
├── launch/experiment.launch.py
├── config/experiment_scenarios.yaml
├── config/ugv_energy_model.yaml
├── delivery_evaluation/
│   ├── __init__.py
│   ├── models.py
│   ├── scenario_config.py
│   ├── path_metrics.py
│   ├── energy_estimator.py
│   ├── metrics_collector.py
│   ├── experiment_runner.py
│   └── report_generator.py
└── test/
    ├── test_scenario_config.py
    ├── test_path_metrics.py
    ├── test_energy_estimator.py
    └── test_report_generator.py

docs/
├── evaluation_method.md
└── evaluation_results.md
```

运行结果写入以下目录，不要把 rosbag、Gazebo 日志或大体积原始图像提交到 Git：

```text
simulation_ws/experiment_results/<timestamp>/
├── runs.csv
├── runs.json
├── summary.md
├── phase_duration.png
├── path_length.png
├── energy_comparison.png
└── success_rate.png
```

如果环境没有 `matplotlib`，先完成 CSV、JSON 和 Markdown，再明确报告缺少图表的
依赖原因；安装系统依赖前必须获得用户许可。

## 九、节点职责和 ROS2 接口

### `experiment_runner`

- 读取实验场景和重复次数。
- 根据模式调用现有接口：
  - `ugv_only`：`/navigate_to_pose`
  - `uav_only`：`/uav/execute_delivery`
  - `cooperative`：`/cooperative_delivery/execute_mission`
- UGV 单独模式需要自动执行“目标点 -> 物流中心”的完整往返。
- 每次任务前检查初始条件，每次结束后等待系统稳定。
- 支持 `continue_on_failure`，失败后写入数据，不得静默丢弃。
- 发布 `/delivery_evaluation/status`，消息类型可使用 `std_msgs/String`。

### `metrics_collector`

至少订阅：

- `/clock`
- `/ground_truth/odom`
- `/uav/odom`
- `/cooperative_delivery/mission_status`
- `/uav/mission_status`
- `/uav/docked`
- `/uav/safety/blocked`
- `/uav/safety/min_distance`
- `/uav/battery_state`
- `/uav/battery_power_w`
- `/uav/battery_consumed_wh`
- `/uav/battery_charged_wh`

如果现有 Nav2 Feedback 无法向采集节点提供恢复次数，可以由 runner 在 Action
feedback 中记录，不要为了一个指标改写 Nav2。

### `report_generator`

- 只读取已落盘的实验文件，不依赖正在运行的 ROS 图。
- 按模式给出成功率、平均值、标准差、最小值和最大值。
- 生成三个模式的任务时间、路径长度和估算能耗对比。
- 失败记录单独列出，不得从平均结果中无说明地删除。
- 图表标题、坐标轴、单位和图例完整，能够直接用于毕业论文。

## 十、初始条件和可重复性

每次运行前必须自动确认：

1. 相应 Action Server 已就绪。
2. UGV 位于物流中心允许误差范围内。
3. UAV 单独模式下 UAV 已落地；协同模式下 `/uav/docked == true`。
4. UGV 和 UAV 速度低于稳定阈值。
5. 上一次任务处于终止状态，不存在仍在执行的 Goal。
6. 输出目录可写，`run_id` 不重复。

若初始条件不满足，记录 `PRECONDITION_FAILED` 并停止该批次，不得通过持续改写
Gazebo 坐标强制复位。由于当前 UGV 速度为 `0.22 m/s` 且 VirtualBox 实时因子
可能低于 1，任务总超时必须依据仿真时间；同时保留独立的真实时间无进展看门狗。

## 十一、实施计划表

| 阶段 | 工作内容 | 完成标准 |
|---|---|---|
| M0 | 审查现有 Action、Topic、日志和启动文件，保存基线参数 | 给出接口清单，不修改控制器 |
| M1 | 创建 `delivery_evaluation` 包和配置模型 | 新包可单独编译，配置校验通过 |
| M2 | 实现路径、端点、阶段和安全事件采集 | 合成数据单元测试通过 |
| M3 | 实现可配置能耗积分 | 单元测试覆盖飞行、对接和异常时间戳 |
| M4 | 实现三模式实验 runner | 能自动执行一次 S0 并写出结果 |
| M5 | 增加初始条件、超时、失败和取消处理 | 失败实验同样生成完整记录 |
| M6 | 实现 CSV、JSON、Markdown 和图表生成 | 单次命令生成全部报告 |
| M7 | 全工作空间编译和回归测试 | 现有软件包仍可编译，测试无失败 |
| M8 | 执行 E1-E4 正式实验 | 获得完整、非手工编造的数据集 |
| M9 | 更新实验方法、结果文档和 README | 命令、系数、结果和限制可复现 |

每完成一个阶段就进行对应编译或测试，不要等到最后一次性排错。

## 十二、验收标准

1. `colcon build` 全工作空间成功。
2. `colcon test --packages-select delivery_evaluation` 无失败。
3. 原有 UGV、UAV、联合启动命令仍能运行。
4. 正式 E1、E2、E3 各生成 3 条有效记录。
5. 教学楼协同闭环连续 3 次均成功，并且每次最终 UAV 已对接、UGV 已回到物流
   中心。
6. E4 完成 `laboratory -> library -> dormitory_2 -> logistics_center`，完成目标数
   为 3。
7. UGV 门口停止误差不超过 `0.5 m`。
8. UAV 配送点水平误差不超过 `0.5 m`，高度误差不超过 `0.3 m`。
9. 每条成功记录的路径长度、任务时间和估算能耗均为正数且单位明确。
10. 失败或取消任务不会被记为成功，也不会丢失失败原因。
11. 报告包含成功率、均值、标准差和至少四张对比图。
12. 正式实验只启动一个 Gazebo server，并支持 `gui:=false rviz:=false`。
13. 不修改当前稳定的 `0.22 m/s` UGV 控制基线，除非提交可复现故障证据。

## 十三、预期使用命令

编译：

```bash
cd ~/design_final/wqi_final/simulation_ws
source /opt/ros/humble/setup.bash
colcon build --executor sequential --event-handlers console_cohesion+
source install/setup.bash
```

运行一次协同冒烟实验：

```bash
ros2 launch delivery_evaluation experiment.launch.py \
  mode:=cooperative scenario:=teaching_building repetitions:=1 \
  gui:=false rviz:=false
```

运行教学楼三模式正式实验：

```bash
ros2 launch delivery_evaluation experiment.launch.py \
  mode:=ugv_only scenario:=teaching_building repetitions:=3 \
  gui:=false rviz:=false

ros2 launch delivery_evaluation experiment.launch.py \
  mode:=uav_only scenario:=teaching_building repetitions:=3 \
  gui:=false rviz:=false

ros2 launch delivery_evaluation experiment.launch.py \
  mode:=cooperative scenario:=teaching_building repetitions:=3 \
  gui:=false rviz:=false
```

运行多目标协同回归：

```bash
ros2 launch delivery_evaluation experiment.launch.py \
  mode:=cooperative scenario:=multi_target_regression repetitions:=1 \
  gui:=false rviz:=false
```

重新生成汇总报告：

```bash
ros2 run delivery_evaluation generate_report \
  --input ~/design_final/wqi_final/simulation_ws/experiment_results/<timestamp>
```

具体参数名可以根据现有 launch 风格小幅调整，但最终 README 中必须给出经过实际
验证、可以直接执行的命令。

## 十四、明确不属于本阶段的内容

- 不迁移 PX4 或开发板。
- 不重新绘制校园地图或道路。
- 不再次提高 UGV 速度。
- 不实现在线三维全局重规划。
- 不实现移动中的 UGV 起降或移动平台追踪降落。
- 不新增会改变飞行稳定性的实体包裹模型。
- 不用连续传送坐标代替物理运动。

这些功能可以作为论文“后续工作”，不应阻塞本阶段实验数据产出。

## 十五、执行要求

执行者必须从 M0 到 M9 直接完成实现、编译、自动测试和可行范围内的真实仿真，
不能只给建议或停在代码框架。正式长时间实验必须持续观察任务状态和日志；如果
受 VirtualBox 运行时间限制未能完成全部重复次数，需要保留已生成的数据，说明
精确未完成项，并提供可继续运行且不会覆盖已有结果的命令。

最终回答必须说明：

- 新增和修改的文件；
- 数据采集和能耗估算方法；
- 编译与测试结果；
- 每个正式场景的实际成功次数；
- 生成结果目录；
- 发现的失败原因和修复；
- 未完成项及剩余风险；
- 完整复现命令。
