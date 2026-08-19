# wqi_final：校园空地协同物流配送仿真系统

本项目是基于 ROS 2 Humble、Gazebo Classic、RViz2 和 Nav2 的毕业设计仿真平台，
面向校园最后一公里物流配送。系统包含可独立运行的 UGV 地面无人车、UAV 四旋翼
无人机，以及带任务规划、物理停靠、电量约束和自动充电的 UGV-UAV 协同配送流程。

当前版本仅用于电脑端仿真，不需要部署到开发板。仓库已经删除 Hello World、基础
话题/服务示例、教学巡逻节点、rosbag 练习和硬件启动代码，仅保留与毕业设计直接相关
的源代码。`v1.0.0` 是清理后的第一版仿真基线。

## 1. 项目目录

```text
wqi_final/
├── README.md
├── docs/
│   ├── system_architecture.md       # 系统架构、协同状态机和数据流
│   ├── completion_checklist.md      # 任务书逐项完成度和论文收尾清单
│   ├── evaluation_method.md         # 实验变量、指标、矩阵和统计方法
│   ├── evaluation_report.md         # 已完成的构建、测试和运行证据
│   ├── paper_based_dynamic_navigation.md # D* Lite 与预测式 DWA 的理论和实现
│   ├── cooperative_delivery.md      # 停靠插件和协同任务说明
│   ├── uav_subsystem.md             # UAV 动力学、传感器和安全控制
│   └── uav_battery.md               # UAV 功率、电池和充电模型
└── simulation_ws/
    └── src/
        ├── ugvcar_description       # UGV 模型、Gazebo 场景和校园地图生成器
        ├── ugvcar_navigation2       # Nav2 地图、参数和启动文件
        ├── ugvcar_navigation2_interfaces # 动态障碍位置/速度结构化消息
        ├── ugvcar_application       # UGV 多目标配送管理器
        ├── uav_interfaces           # UAV Action 和服务接口
        ├── uav_description          # UAV 模型、传感器、TF 和 RViz 配置
        ├── uav_control              # 飞行控制、电量模型和三维安全监控
        ├── uav_navigation           # UAV 航点、15 m 航路图和路径规划
        ├── uav_application          # UAV 配送任务状态机
        ├── uav_bringup              # 独立 UAV 校园仿真启动
        ├── cooperative_delivery_interfaces # 空地协同 Action 接口
        ├── cooperative_delivery     # 协同管理器、停靠插件和联合启动
        ├── campus_dynamic_obstacles # 可复现的地面/空中物理动态障碍
        ├── delivery_evaluation      # 自动实验、指标采集、报告和图表
        ├── simulation_ui            # 六阶段 PyQt5 控制与实时监控界面
        └── vendor/sjtu_drone_description # Gazebo 四旋翼力/力矩动力学插件
```

## 2. 已实现功能

- 在 Gazebo 中建立包含 11 栋主要建筑、闭合道路、碰撞体和配送点的校园场景。
- 阶段 1 至 5 使用 Nav2 `SmacPlanner2D + Regulated Pure Pursuit` 完成稳定静态
  导航；阶段 6 切换为 `D* Lite + DWB`。D* Lite 复用上次搜索状态，只修复动态
  代价变化影响的网格；自定义 `PredictiveCollisionCritic` 根据 UGV 和障碍物的相对
  速度、候选轨迹及预计碰撞时间评价 DWB 轨迹，覆盖前方、横穿和斜后方追赶工况。
  系统不执行固定倒车、固定转角或固定等待。
- UGV 多目标任务将 `campus_layout.yaml` 道路中心线构造成加权道路图，先用
  Dijkstra 计算站点间道路距离，再求精确最短闭合访问顺序；每个实际行驶段仍由
  Nav2 规划，并在阶段 6 由 D* Lite 根据动态代价增量重规划。
- UAV 使用 Gazebo 力/力矩动力学完成物理起飞、悬停、巡航、下降和降落，不使用
  `/set_entity_state` 持续修改坐标模拟飞行。
- UAV 配备顶部 3D 雷达、下视相机、下向测距、四个斜下短距传感器和 IMU。
- UAV 顶部 3D 雷达持续判断三维轨迹冲突，每 `0.5 s` 生成平滑绕障路径并通过前视
  点跟随黄色 `/uav/replanned_path`；`1.8 m` 三维安全球用于紧急保护。持续阻塞时
  禁用当前航路边并重新运行 Dijkstra，而不是执行固定方向和固定距离的绕行动作。
- 协同系统使用 Gazebo 固定关节将 UAV 停靠在 UGV 上；UGV 到达建筑门口并稳定后，
  UAV 才解锁起飞，配送结束后重新落到 UGV 并锁定。
- 单个任务最多支持 10 件货物，每件货物分别设置目标、楼层和质量，并保持目标、
  楼层、载荷三组数据在路线优化后仍一一对应。
- UAV 电量模型区分起飞、加速、巡航、转弯、悬停、下降和停靠充电；起飞前检查
  完整任务能量和安全返航余量。
- UGV 使用相互独立的驱动电池和 UAV 充电电池。驱动功率随速度、转向、加速度、
  剩余货物质量和停靠 UAV 质量变化；充电电池只通过充电转换器向 UAV 供能。
- 协同任务开始前同时检查 UAV 架次能量、UGV 完整道路行程能量和有限充电电池预算，
  任一电池低于任务需求与安全储备时均拒绝任务。
- 支持 `none`、`low`、`medium`、`high` 四档可复现动态障碍场景。
- 支持 UGV、UAV、空地协同三种模式的自动实验，输出任务成功率、真实动态避障
  成功率、时间、路径、误差、恢复次数、重规划次数、能耗、SOC、最小净空和碰撞
  事件数。即使 Action 返回成功，检测到物理包络相交时该次实验仍判定为失败。
- 提供六阶段 PyQt5 控制界面，分别显示 UAV 飞行电池、UGV 驱动电池、UGV 充电
  电池及其功率，并显示停靠、安全状态、机器人位置、动态障碍数量和重规划次数。

## 3. 编译与环境加载

### 3.1 从 GitHub 完整复现

在已安装 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Classic 的新环境中，将仓库
克隆为 `~/design_final`，这样目录结构与本文后续命令保持一致：

```bash
git clone https://github.com/liyongqihhh/wqi_final.git ~/design_final
cd ~/design_final/wqi_final/simulation_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash ./build_workspace.sh
source ./setup_workspace.bash
bash ./test_workspace.sh --event-handlers console_cohesion+
```

编译和测试通过后，可先进行无图形界面的联合仿真检查：

```bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  gui:=false rviz:=false \
  enable_energy_constraints:=false enable_dynamic_obstacles:=false
```

仓库只保存 `simulation_ws/src/` 源码、配置、地图、脚本和文档。编译产物默认
写入 Git 仓库同级目录，克隆后的结构如下：

```text
~/design_final/                    # Git 源码仓库，可直接提交或打包
└── wqi_final/simulation_ws/
    ├── src/
    ├── build_workspace.sh
    ├── setup_workspace.bash
    └── test_workspace.sh
~/design_final_artifacts/          # 不属于 Git 仓库
├── build/                          # 编译中间文件
├── install/                        # 可执行程序和动态库
├── log/                            # colcon 编译及测试日志
└── experiment_results/             # 自动评测记录、CSV、图表和报告
```

不要在源码目录中直接执行普通的 `colcon build`。统一使用
`bash ./build_workspace.sh`，它会自动根据仓库目录名选择同级的
`<仓库名>_artifacts/`。需要指定其他磁盘时，可以在编译前设置：

```bash
export WQI_BUILD_ROOT=/path/to/wqi_final_artifacts
bash ./build_workspace.sh
```

`build/`、`install/`、`log/` 和实验结果都不上传 GitHub。前三者是可重新生成的
本机产物，也不能从其他电脑复制使用。评测输出目录还可通过
`WQI_EXPERIMENT_RESULTS_DIR` 单独覆盖。

每次打开新终端后，先执行：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
```

首次使用或修改源代码后编译整个工作空间：

```bash
bash ./build_workspace.sh
source ./setup_workspace.bash
```

如果代码没有重新编译，只需要加载现有安装环境：

```bash
source ./setup_workspace.bash
```

仅编译 UAV 子系统：

```bash
bash ./build_workspace.sh --packages-select \
  sjtu_drone_description uav_interfaces uav_description uav_control \
  uav_navigation uav_application uav_bringup
source ./setup_workspace.bash
```

编译空地协同系统及其依赖：

```bash
bash ./build_workspace.sh --packages-up-to cooperative_delivery uav_bringup
source ./setup_workspace.bash
```

## 4. 四档动态障碍启动方法

动态障碍共有四档。障碍是带碰撞体的 Gazebo 实体，通过物理速度沿固定路线往返，
不是连续修改坐标的动画。相同 `random_seed` 会选择相同路线，便于重复实验。

最直接的打开位置在图形界面：运行 `ros2 run simulation_ui simulation_dashboard`，
左侧选择 **阶段 6：动态障碍协同配送**，然后在任务配置区的 **动态障碍密度** 下拉框
选择 `无动态障碍`、`低密度`、`中密度` 或 `高密度`。阶段 1 至 5 会自动锁定为
`无动态障碍`，只有阶段 6 可以切换这四档。

| 参数 | 地面动态障碍 | 空中动态障碍 | 总数 | 用途 |
|---|---:|---:|---:|---|
| `none` | 0 | 0 | 0 | 无障碍基准组 |
| `low` | 3 | 0 | 3 | 低密度地面障碍 |
| `medium` | 5 | 1 | 6 | 中密度空地障碍 |
| `high` | 8 | 2 | 10 | 高密度压力测试 |

### 4.1 使用联合启动文件打开

阶段 6 必须由联合启动文件同时启用动态障碍、UGV 360 度动态障碍预测器和 UAV
3D 雷达重规划器。不要先按静态阶段启动后再单独追加障碍生成器，否则机器人侧的
动态重规划参数不会按阶段 6 启用。

终端 1 启动低密度阶段 6 仿真：

```bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  gui:=true rviz:=false visualize_sensor_rays:=false \
  enable_energy_constraints:=true enable_dynamic_obstacles:=true \
  obstacle_density:=low random_seed:=42 \
  initial_battery_percentage:=0.80 \
  initial_ugv_drive_battery_percentage:=0.80 \
  initial_ugv_charging_battery_percentage:=0.80
```

将 `obstacle_density:=low` 改为 `none`、`medium` 或 `high` 即可切换其他密度。
`none` 仍使用阶段 6 的机器人控制链，但不会生成动态障碍。
阶段 4、5、6 均按 UGV 搭载 UAV 后的 `0.60 m` 组合外廓规划；单独运行 UGV 时
仍使用 `0.22 m` 物理半径。动态障碍物严格沿场景配置的固定路线和速度运行，不查询
UGV/UAV 位置，也不会主动减速或让路。UGV 使用完整 360 度激光扫描聚类并在地图
坐标系中估计障碍速度，计算 `15 s` 预测窗内的最近会遇状态，并发布结构化轨迹
`/ugv/tracked_dynamic_obstacles` 及固定风险点 `/scan_dynamic_predictions`。预测点
只进入全局代价地图；D* Lite 以 `1 Hz` 增量修复受代价变化影响的路径，并在瞬时
规划失败时保留上一条仍有效的路径，避免反复取消控制器。DWB 以 `10 Hz` 采样
加速度受限的非完整约束轨迹，自定义预测碰撞评价器在 `4 s` 局部时域内比较候选轨迹
与运动障碍物的碰撞时间和净空，再选择安全且保持前进的速度。局部代价地图仍只处理
真实 `/scan`，承担静态几何约束和及时清障；前向碰撞监视器仅作为最后的紧急停止层。
前方、横穿和斜后方追近的障碍使用同一套相对运动模型，不执行固定倒车、固定角度
转弯或固定等待。全局代价地图继续使用
`1.0 m` 道路软边界来生成居中的安全路线；局部代价地图使用 `0.65 m` 软边界和
`0.8 m` 障碍膨胀半径，让小车在绕障时可以使用更多铺装路面。道路外仍是致命代价，
`0.60 m` 协同外廓没有缩小。

UAV 使用顶部 3D 雷达的最近有效障碍向量，每 `0.5 s` 重算平滑三维绕障路径，并以
前视点跟随 `/uav/replanned_path`。绕障路径保持向目标前进，不使用预设的
左、右、上、下固定绕点；障碍离开后恢复目标直线路径。

终端 2 发送协同任务：

```bash
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['teaching_building'], return_home: true}" --feedback
```

动态障碍实体显示在 Gazebo 中。联合 RViz 同时显示 UGV 全局和局部代价地图、
UGV 黄色实时路线 `/plan`、UAV 青色任务路线 `/uav/planned_path`、UAV 黄色动态
绕障路线 `/uav/replanned_path` 和橙色实际轨迹 `/uav/path`。切换密度前应按
`Ctrl+C` 结束全部进程并重新启动 Gazebo；不要同时运行两个障碍生成器，否则旧实体
不会自动删除。

机器人侧避障状态可通过以下话题检查：

```bash
ros2 topic echo /ugv/dynamic_replanning/status
ros2 topic echo /ugv/dynamic_replanning/tracked_obstacles
ros2 topic echo /ugv/tracked_dynamic_obstacles
ros2 topic echo /ugv/dstar_lite/status
ros2 topic echo /ugv/predictive_dwa/status
ros2 topic echo /scan_dynamic_predictions
ros2 topic echo /plan
ros2 topic echo /uav/safety/status
ros2 topic echo /uav/safety/nearest_obstacle
ros2 topic echo /uav/dynamic_replanning/status
ros2 topic echo /uav/dynamic_replanning/count
ros2 topic echo /uav/replanned_path
```

### 4.2 一条命令自动启动、运行任务并保存结果

下面四条命令会自动启动协同仿真、生成指定密度障碍、执行教学楼配送、记录指标，
任务完成后自动关闭该批次。`gui:=true rviz:=false` 用于在 Gazebo 中观察障碍；在
VirtualBox 中做正式计时实验时应改为 `gui:=false rviz:=false`。

```bash
# 无障碍
ros2 launch delivery_evaluation experiment.launch.py \
  mode:=cooperative scenario:=teaching_building \
  obstacle_density:=none repetitions:=1 random_seed:=42 \
  results_dir:=$WQI_BUILD_ROOT/experiment_results gui:=true rviz:=false
```

```bash
# 低密度
ros2 launch delivery_evaluation experiment.launch.py \
  mode:=cooperative scenario:=teaching_building \
  obstacle_density:=low repetitions:=1 random_seed:=42 \
  results_dir:=$WQI_BUILD_ROOT/experiment_results gui:=true rviz:=false
```

```bash
# 中密度
ros2 launch delivery_evaluation experiment.launch.py \
  mode:=cooperative scenario:=teaching_building \
  obstacle_density:=medium repetitions:=1 random_seed:=42 \
  results_dir:=$WQI_BUILD_ROOT/experiment_results gui:=true rviz:=false
```

```bash
# 高密度
ros2 launch delivery_evaluation experiment.launch.py \
  mode:=cooperative scenario:=teaching_building \
  obstacle_density:=high repetitions:=1 random_seed:=42 \
  results_dir:=$WQI_BUILD_ROOT/experiment_results gui:=true rviz:=false
```

查看当前障碍数量和场景信息：

```bash
ros2 topic echo /dynamic_obstacles/count
ros2 topic echo /dynamic_obstacles/scenario
```

评测器会按照相同 `obstacle_density` 和 `random_seed` 订阅本批次实际生成的每个
`/dynamic_obstacles/<name>/odom`。地面障碍按二维表面净间距与 UGV 判断，空中
障碍按三维表面净间距与 UAV 判断：

```text
clearance = center_distance - robot_envelope_radius - obstacle_radius
collision = clearance <= 0
avoidance_success_rate = collision_free_runs / total_runs
```

持续接触只记一次碰撞，分离后再次接触才记为新事件。协同 UGV、独立 UGV 和 UAV
的判定包络半径分别为 `0.60 m`、`0.22 m` 和 `0.56 m`。

## 5. 图形化控制界面

控制界面可以选择六个毕业设计阶段，设置最多 10 件货物的配送位置、楼层和质量，
设置 UAV 飞行电池、UGV 驱动电池、UGV 充电电池的初始电量，以及是否返航、是否
显示传感器射线、打开 RViz/Gazebo 的方式。阶段 6 还可以直接选择四档动态障碍密度。

编译并启动界面：

```bash
cd ~/design_final/wqi_final/simulation_ws
bash ./build_workspace.sh --packages-up-to simulation_ui
source ./setup_workspace.bash
ros2 run simulation_ui simulation_dashboard
```

操作顺序：

1. 在左侧选择阶段 1 至阶段 6。
2. 设置货物件数，并为每件货物填写目标、楼层和质量。
3. 阶段 5、6 分别设置三块初始电量；阶段 6 再选择动态障碍密度。
4. 选择 `RViz`、`Gazebo` 或 `两者`。
5. 点击 `启动仿真`，等待 Action 和导航服务就绪。
6. 点击 `运行配送任务`。
7. `停止任务` 只取消当前任务；`停止全部` 关闭界面启动的所有进程。

阶段 1 使用 RViz 手动标点，因此没有自动配送路线按钮。界面状态区会实时显示任务
阶段、三块电池的电量与功率、停靠、安全状态、UGV/UAV 位置、动态障碍数量和
重规划次数。

UAV 在航路中以 `15 m` 高度巡航，在建筑物正面配送点使用楼层中心高度：

```text
delivery_height = 1.6 + (floor - 1) * 3.2 m
```

初始电量和障碍密度都是启动参数。Gazebo 已经运行后再修改这些值不会改变当前仿真，
必须点击 `启动仿真` 重新启动。任一电池低于 `40%` 时界面会警告；UAV 飞行电池
安全储备为 `20%`，UGV 驱动电池为 `20%`，UGV 充电电池为 `10%`。低于联合任务
需求时，界面和日志会显示是 UAV 架次、UGV 驱动还是充电预算不足。

## 6. 六阶段毕业设计测试

建议按顺序完成六个阶段。VirtualBox 性能不足时不要同时打开 Gazebo 和 RViz：

- 观察导航地图：`gui:=false rviz:=true`
- 观察 Gazebo 模型和动态障碍：`gui:=true rviz:=false`
- 仅采集正式数据：`gui:=false rviz:=false`

| 阶段 | 被测系统 | 电量模型 | 动态障碍 | 任务输入 |
|---|---|---|---|---|
| 1 | 房间内 UGV 标点导航 | 关闭 | 关闭 | RViz 手动标点 |
| 2 | 校园内 UGV 自动导航 | 关闭 | 关闭 | `delivery_task.launch.py` |
| 3 | 校园内 UAV 自动导航 | 关闭 | 关闭 | `/uav/execute_delivery` Action |
| 4 | 校园空地协同导航 | 关闭 | 关闭 | 协同 Action |
| 5 | 三电池约束空地协同 | 开启 | 关闭 | 正常电量、低电量拒绝和停靠充电 |
| 6 | 动态障碍空地协同配送 | 开启 | 四档可选 | 协同 Action 与定量实验 |

### 6.1 阶段一：房间 UGV 避障与导航

终端 1 启动房间 Gazebo 并生成 UGV：

```bash
ros2 launch ugvcar_description gazebo_sim.launch.py
```

终端 2 启动房间地图、Nav2 和 RViz：

```bash
ros2 launch ugvcar_navigation2 navigation2.launch.py
```

如果 RViz 尚未初始化定位，先使用 `2D Pose Estimate`；然后使用 `Nav2 Goal` 或
`2D Goal Pose` 在障碍物另一侧的空闲区域标点。小车应绕开障碍，到达目标且不碰墙。

### 6.2 阶段二：校园 UGV 避障与导航

终端 1 启动校园 Gazebo：

```bash
ros2 launch ugvcar_description campus_delivery_sim.launch.py \
  gui:=true visualize_sensor_rays:=false
```

终端 2 启动校园 Nav2、地图、禁行掩膜和 RViz：

```bash
ros2 launch ugvcar_navigation2 campus_navigation.launch.py \
  rviz:=true localization_mode:=ground_truth
```

默认使用 Gazebo ground truth 定位，保证 Gazebo 和 RViz 位姿一致。需要单独测试
AMCL 时使用：

```bash
ros2 launch ugvcar_navigation2 campus_navigation.launch.py \
  localization_mode:=amcl
```

终端 3 在 Nav2 就绪后运行教学楼短路线：

```bash
ros2 launch ugvcar_application delivery_task.launch.py \
  delivery_targets:="['teaching_building']"
```

运行教学楼、实验楼和二号宿舍多目标路线：

```bash
ros2 launch ugvcar_application delivery_task.launch.py \
  delivery_targets:="['teaching_building','laboratory','dormitory_2']" \
  wait_duration:=10.0
```

可用目标：`cafeteria`、`teaching_building`、`innovation_center`、`library`、
`laboratory`、`gymnasium`、`dormitory_1`、`dormitory_2`、`dormitory_3`、
`dormitory_4`。四栋宿舍共享道路中心停靠点 `(30.0, 11.0)`。

UGV 直线路段最高巡航速度为 `0.40 m/s`，控制器会在转角、障碍物和终点附近主动
减速。较长的速度相关前视距离和受限角加速度用于抑制高速下的 S 形摆动。局部代价地图
膨胀半径为 `0.8 m`，全局代价地图膨胀半径为 `1.0 m`，禁行掩膜在道路边缘保留
可恢复的低代价缓冲带。验收标准是小车保持在道路内、避开占据区域、到达所有目标并
返回物流中心。

### 6.3 阶段三：校园 UAV 避障与导航

该启动文件复用校园 world，但不会生成或启动 UGV。

终端 1 启动 UAV、Gazebo 和 RViz：

```bash
ros2 launch uav_bringup uav_sim.launch.py \
  gui:=true rviz:=true visualize_sensor_rays:=false \
  enable_energy_constraints:=false
```

VirtualBox 推荐二选一：

```bash
ros2 launch uav_bringup uav_sim.launch.py gui:=false rviz:=true
ros2 launch uav_bringup uav_sim.launch.py gui:=true rviz:=false
```

终端 2 发送教学楼配送任务：

```bash
ros2 action send_goal /uav/execute_delivery \
  uav_interfaces/action/ExecuteDelivery \
  "{targets: ['teaching_building'], return_home: true}" --feedback
```

发送教学楼三层和图书馆五层多目标任务：

```bash
ros2 action send_goal /uav/execute_delivery \
  uav_interfaces/action/ExecuteDelivery \
  "{targets: ['teaching_building','library'], return_home: true, \
  target_floors: [3,5], payload_masses_kg: [0.30,0.25]}" --feedback
```

任务流程为：物理起飞、悬停 5 秒、进入 `15 m` 航路、沿闭合航路图飞行、接近建筑
正面配送点、下降到指定楼层、悬停配送、返航和物理降落。由于部分建筑高于 15 m，
规划器不会在建筑物之间直接使用可能穿楼的目标到目标直线。

主要接口：

- Action：`/uav/fly_to_pose`、`/uav/execute_delivery`
- 服务：`/uav/takeoff`、`/uav/land`、`/uav/check_delivery_energy`
- 状态：`/uav/odom`、`/uav/imu`、`/uav/flight_state`、`/uav/mission_status`
- 电量（仅阶段 5、6 启用）：`/uav/battery_state`、`/uav/battery_percentage`、
  `/uav/battery_power_w`、`/uav/battery_consumed_wh`、
  `/uav/battery_charged_wh`、`/uav/remaining_energy`
- 感知：`/uav/lidar/points`、`/uav/down_camera/image_raw`、
  `/uav/range/down`、`/uav/range/front_down`、`/uav/range/rear_down`、
  `/uav/range/left_down`、`/uav/range/right_down`
- 安全：`/uav/safety/blocked`、`/uav/safety/status`、
  `/uav/safety/min_distance`、`/uav/safety/ground_clearance`
- 可视化：`/uav/planned_path`、`/uav/path`、`/uav/delivery_points`、
  `/uav/safety_sphere`、`/uav/optimized_route`

检查传感器是否发布：

```bash
ros2 topic hz /uav/lidar/points
ros2 topic hz /uav/down_camera/image_raw
ros2 topic echo /uav/range/down
ros2 topic echo /uav/safety/status
ros2 topic hz /uav/imu
```

RViz 中青色 `/uav/planned_path` 是节点到节点计划路线，橙色 `/uav/path` 是受惯性
影响的真实飞行里程计轨迹，因此转弯处可以出现小弧线。验收标准是 UAV 不穿过建筑、
危险距离前能够悬停或绕行、完成目标后返回物流中心，并以 `LANDED` 和 `CLEAR` 结束。

### 6.4 阶段四：校园 UGV-UAV 协同配送

联合启动文件只创建一个 Gazebo，同时启动 UGV、UAV、Nav2、飞控、停靠插件和协同
任务管理器。该阶段不启动 UAV/UGV 电池节点，也不生成动态障碍，主要验证导航、
交接和重新停靠，不会因为电量状态缺失而拒绝任务。

终端 1 启动联合仿真：

```bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  gui:=true rviz:=true visualize_sensor_rays:=false \
  enable_energy_constraints:=false enable_dynamic_obstacles:=false
```

终端 2 运行教学楼完整协同任务：

```bash
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['teaching_building'], return_home: true}" --feedback
```

运行实验楼、图书馆和四号宿舍多目标任务：

```bash
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['laboratory','library','dormitory_4'], return_home: true, \
  target_floors: [4,5,11], payload_masses_kg: [0.35,0.20,0.25]}" --feedback
```

协同状态流程：

```text
PREPARING -> UGV_TRANSIT -> UGV_SETTLING -> UAV_DETACHING
-> UAV_DELIVERING -> UAV_DOCKING -> RETURNING_HOME -> COMPLETED
```

主要接口：

- 协同 Action：`/cooperative_delivery/execute_mission`
- 协同状态：`/cooperative_delivery/mission_status`
- 优化顺序：`/cooperative_delivery/optimized_route`
- 停靠服务：`/uav/attach_uav`、`/uav/detach_uav`
- 停靠状态：`/uav/docked`

验收标准是 UGV 先到达建筑门口的停靠点并稳定，UAV 随后解锁，完成楼层配送后返回
UGV 并重新锁定，最后 UGV 返回物流中心，任务状态为 `COMPLETED`。

### 6.5 阶段五：加入电量约束的空地协同配送

本阶段固定 `enable_dynamic_obstacles:=false`，只验证能量模型、任务准入、空地交接和
停靠充电，不把动态避障因素混入能耗基准。

电量模型以 Zeng 旋翼无人机水平功率模型为基础，使用 Gong 垂直起降模型补充上升和
下降，并使用 Dai 动态推重比修正加速和转弯功率。模型包含机体、传感器、货物质量
以及约 `25 W` 的计算机、雷达、相机和通信负载。

核心关系：

```text
m = m_airframe + m_sensor + m_payload
P_induced = (1 + k) * (m*g)^(3/2) / sqrt(2*rho*A)
P_battery = (P_horizontal + P_vertical - P_hover + P_auxiliary)
            / discharge_efficiency

E_raw = sum(P_battery(v_j, a_j, payload_j) * dt_j / 3600)
E_predicted = 1.25 * E_raw
E_required = E_predicted + battery_capacity * 0.20
仅当 E_available >= E_required 时接受任务
```

UGV 有两块互不混用的电池：`300 Wh` 驱动电池和 `250 Wh` UAV 充电电池。驱动
模型按实时里程计积分，质量会在每件货物交给 UAV 后减少，UAV 只有在停靠时才计入
UGV 总质量：

```text
m_ugv = m_base + m_remaining_cargo + docked * m_uav
P_drive = P_idle + (Crr*m_ugv*g*v + 0.5*rho*CdA*v^3
          + max(0, m_ugv*a*v)) / eta_drive
          + k_v*|v| + k_w*|omega|

E_drive_required = 1.20 * integral(P_drive dt) + 20% drive reserve
E_charge_source = E_uav_charge / eta_charge + charger_idle_energy
```

充电电池保留 `10%` 安全储备，到达储备后 `/ugv/charger_available` 变为 `false`，
UAV 即使仍停靠也不再充电。任务管理器会在 UGV 开始移动前联合检查完整道路行程、
逐件变化的货物质量、UAV 每一架次和有限充电电池预算。

#### 6.5.1 验证停靠自动充电

终端 1 以 30% 电量启动联合系统：

```bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  gui:=true rviz:=true visualize_sensor_rays:=false \
  enable_energy_constraints:=true enable_dynamic_obstacles:=false \
  initial_battery_percentage:=0.30 \
  initial_ugv_drive_battery_percentage:=0.80 \
  initial_ugv_charging_battery_percentage:=0.80
```

终端 2 检查停靠和充电：

```bash
ros2 topic echo /uav/docked --once
ros2 topic echo /uav/battery_status
ros2 topic echo /uav/battery_percentage
ros2 topic echo /uav/battery_power_w
ros2 topic echo /uav/battery_charged_wh
ros2 topic echo /ugv/drive_battery_state
ros2 topic echo /ugv/charging_battery_state
ros2 topic echo /ugv/drive_power_w
ros2 topic echo /ugv/total_carried_mass_kg
```

`/uav/docked` 应为 `true`，状态应为 `CHARGING`，电量百分比应随仿真时间增加。默认
净充电功率约为 `-157 W`，负功率表示能量流入 UAV 电池。

#### 6.5.2 起飞前能量预算和正常任务

查询教学楼三层任务是否满足安全返航条件：

```bash
ros2 service call /uav/check_delivery_energy \
  uav_interfaces/srv/CheckDeliveryEnergy \
  "{targets: ['teaching_building'], return_home: true, \
  home_name: 'teaching_building', landing_height: 0.42, \
  payload_masses_kg: [0.30], target_floors: [3]}"
```

电量充足时响应应包含 `feasible: true`，并给出推进能量、辅助设备能量、载荷惩罚、
安全储备、总需求和预计任务结束 SOC。随后发送协同任务：

```bash
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['teaching_building'], return_home: true}" --feedback
```

飞行期间应经过起飞、悬停、巡航和降落耗电状态；重新停靠后必须恢复 `CHARGING`。

#### 6.5.3 验证低电量拒绝

停止之前的仿真，将 UGV 驱动电池设为 `10%`，验证任务在车辆移动前被拒绝：

```bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  gui:=true rviz:=true visualize_sensor_rays:=false \
  enable_energy_constraints:=true enable_dynamic_obstacles:=false \
  initial_battery_percentage:=0.80 \
  initial_ugv_drive_battery_percentage:=0.10 \
  initial_ugv_charging_battery_percentage:=0.80
```

发送教学楼协同任务：

```bash
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['teaching_building'], return_home: true}" --feedback
```

结果应为 `success: false`，原因包含 `UGV drive energy REJECT`。UGV 不应开始移动，
UAV 应保持停靠。也可以分别把 UAV 电池设为 `0.01`、UGV 充电电池设为 `0.10`，
验证有限充电预算不足；如果 UGV 行驶期间能够在安全储备之上充入足够能量，低 UAV
初始电量任务仍可能合法通过，这不属于错误。

主要 UGV 电量接口：

- `/ugv/drive_battery_state`、`/ugv/drive_remaining_wh`、`/ugv/drive_consumed_wh`
- `/ugv/charging_battery_state`、`/ugv/charging_remaining_wh`、
  `/ugv/charging_consumed_wh`
- `/ugv/drive_power_w`、`/ugv/charging_source_power_w`、
  `/ugv/uav_charging_output_power_w`
- `/ugv/cargo_mass_kg`、`/ugv/total_carried_mass_kg`、`/ugv/charger_available`

### 6.6 阶段六：动态障碍、电量约束与空地协同配送

阶段六在阶段五全部能量约束基础上启动带碰撞体的动态障碍，作为最终完整系统。使用
UI 时只需选择左侧第六阶段，并在 `动态障碍密度` 下拉框选择四档之一。使用命令行时，
下面的联合启动文件会自行启动障碍生成器，不要再手动运行第二个生成器：

```bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  gui:=true rviz:=false visualize_sensor_rays:=false \
  enable_energy_constraints:=true enable_dynamic_obstacles:=true \
  obstacle_density:=medium random_seed:=42 \
  initial_battery_percentage:=0.80 \
  initial_ugv_drive_battery_percentage:=0.80 \
  initial_ugv_charging_battery_percentage:=0.80
```

将 `obstacle_density:=medium` 分别改成以下值即可打开四种密度：

```text
none    无动态障碍，0 个
low     低密度，3 个
medium  中密度，6 个
high    高密度，10 个
```

发送多目标、不同楼层和不同载荷任务：

```bash
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['teaching_building','library','dormitory_4'], \
  return_home: true, target_floors: [3,5,11], \
  payload_masses_kg: [0.30,0.20,0.25]}" --feedback
```

验收时依次运行 `none`、`low`、`medium`、`high`，每档至少 3 次并固定随机种子集合。
记录成功率、总时间、UGV/UAV 路径长度、三类能耗、最小净空、Nav2 恢复次数、UAV
重规划次数、安全悬停时间、UGV/UAV 碰撞事件数和真实动态避障成功率。阶段六是
论文最终系统结果，阶段一至五用于消融和单模块对照。

阶段六的障碍物是非合作对象：它们保持固定路线和速度，不负责避让机器人。UGV
由 360 度激光动态轨迹预测、D* Lite 增量全局规划和预测式 DWB 局部轨迹评价共同
完成道路内绕行；UAV 由顶部 3D 雷达、平滑三维路径重规划和前视点跟随完成绕障。
两者均不执行固定倒车距离、固定转向角度或固定绕点，因此实验中的避障结果来自
无人车和无人机自身的实时规划，而不是障碍物主动让路。算法理论、公式和与论文原型
的差异见 `docs/paper_based_dynamic_navigation.md`。

UGV 的预测占用使用激光扫描时刻对应的 Gazebo 地图位姿，不再把延迟扫描与最新位姿
混用；目标关联同时检查速度、位移、运动方向、拟合误差和轮廓尺寸。节点同时计算 UGV
地图坐标速度、障碍物速度、相对闭合速度、最近会遇时间和最近会遇距离。差速车不能
横向平移，因此先按等效横向加速度计算完成安全换道所需时间，再计算安全避让距离：

```text
D_lateral = R_ugv + R_obstacle + safety_margin
T_maneuver = 2 * sqrt(D_lateral / lateral_maneuver_acceleration)
D_safe = R_ugv + R_obstacle + safety_margin
         + closing_speed * (response_time + T_maneuver)
         + UGV_speed^2 / (2 * braking_deceleration)
```

默认参数为安全余量 `0.30 m`、系统响应时间 `5.00 s`、制动减速度 `0.60 m/s^2`，
等效横向机动加速度 `0.50 m/s^2`，并增加 `1.50 m` 规划触发缓冲。障碍物速度越高、
相对接近越快，避让触发距离越大。只有预计 `15 s` 内最近会遇距离小于安全轮廓，并且
当前距离进入安全距离加规划缓冲时，才在预计会遇点发布风险占用。每个风险由碰撞中心
和障碍物来向一侧的短保护点组成，使全局规划器选择相反方向绕行；不会把障碍物整条未来
路线提前铺进代价地图。近场保护范围为 `6.00 m`，保护点按 `0.75 m` 间隔延伸，
绕行侧偏移为 `1.60 m`。一次碰撞事件首次确认后，会锁定绕行侧、地图坐标会遇点及其
保护点，后续即使
同一物体被重新关联为不同跟踪 ID，也共用这一组固定锚点；只有风险连续清除后才允许
建立新锚点。这避免保护点随 UGV 向前移动，以及黄色路线左右切换。
预测风险只写入全局代价地图，用于提前改变路线；局部代价地图只接收真实 `/scan`，
避免控制器在虚拟风险点前停车后被斜后方障碍追撞。风险连续消失 `5` 帧后才清理，
避免绕行路径反复切换。机器人周围 `1.20 m` 不写入风险点，由原始 `/scan` 负责
近场碰撞检查。该方法同时识别前方迎面、侧向横穿和斜后方追赶障碍。UAV 使用最近
障碍表面点簇的中值向量，并连续确认目标切换，输出统一位于 `uav/base_link`
坐标系。

阶段六还设置了两级道路边缘恢复：D* Lite 在起点落入边界栅格时，可在 `0.60 m`
内选择最近可行栅格并输出返回道路的短路径；动态道路掩码在铺装边缘外保留 `0.30 m`
软代价恢复带。该恢复带只用于纠正动态会车后的单栅格越界，更外侧仍为致命禁行区，
阶段一至五地图不受影响。

独立 Gazebo 回归使用一条可复现的“斜后方追赶后反向迎面”路线：动态障碍速度
`0.70 m/s`，UGV 最高速度 `0.40 m/s`。任务 Action 成功，物理接触次数为 0，最小
中心距 `1.523 m`，扣除两者碰撞包络后的最小表面净距 `0.953 m`，最小规划安全
净距 `0.173 m`；移动段平均速度 `0.323 m/s`，黄色路径改变 12 次，横向方向反转
次数为 0。另从餐厅停靠点回归物流中心的约 `60.8 m` 路线耗时 `195.9 s`，Nav2
恢复次数为 0，终点误差约 `0.25 m`。

最终完整协同回归使用食堂二层、`0.25 kg` 载荷和自动返程任务。结果为
`SUCCEEDED`：UGV 总路径 `122.44 m`，墙钟时间 `998.0 s`，平均/最大速度
`0.329/0.409 m/s`；动态障碍接触 0，最小中心距/表面净距
`1.387/0.817 m`，最大倾角 `0.01 deg`。UGV 到食堂误差 `0.272 m` 后正确停止并
释放 UAV，UAV 完成配送和重新停靠，UGV 最终到达 `(-0.07, -43.23) m`。黄色路径
变化 25 次，D* Lite 28 次规划中有 16 次复用已有搜索状态。当前全工作空间测试为
`352 tests, 0 errors, 0 failures, 16 skipped`。这些是定向整程回归证据，完整四密度
统计仍应通过第 7 节实验矩阵生成，不能用单次结果替代论文统计。

运行阶段六时可在另一终端检查跟踪稳定性：

```bash
ros2 topic echo /ugv/dynamic_replanning/status
ros2 topic echo /ugv/dynamic_replanning/tracked_obstacles
ros2 topic echo /ugv/dstar_lite/status
ros2 topic echo /ugv/predictive_dwa/status
ros2 topic echo /uav/safety/nearest_obstacle
```

`/ugv/dynamic_replanning/status` 中的 `pose_gap` 应小于 `0.12 s`。动态障碍进入
雷达范围并形成稳定轨迹后，`stable_dynamic_tracks` 应大于 0；确认碰撞风险后，
`active_threats` 和 `risk_points` 应大于 0，并显示 `zone`、`obstacle_speed`、
`closing_speed`、`safe_distance`、`maneuver_time`、`avoidance_side`、`ttc`
和 `closest`。`zone` 可以是 `front`、`front_left`、`front_right`、`rear`、
`rear_left` 或 `rear_right`，`avoidance_side` 是规划器应选择的左右绕行方向。
风险解除后应恢复 `zone=clear`。UAV 最近障碍消息的 `frame_id` 应为
`uav/base_link`。

## 7. 自动化定量实验

评测器会启动真实仿真、发送真实 ROS 2 Action、采集数据并关闭该批次。每个结果目录
包含：

- `runs.json`：完整原始数据。
- `runs.csv`：论文表格输入。
- `summary.md`：任务成功率、真实动态避障成功率、效率、误差、能耗和碰撞汇总。
- `success_rate.png`、`avoidance_success_rate.png`、`phase_duration.png`、
  `energy_comparison.png`、`ugv_path_length.png`、`uav_path_length.png`：
  自动生成图表。

运行中密度教学楼协同任务三次：

```bash
ros2 launch delivery_evaluation experiment.launch.py \
  mode:=cooperative scenario:=teaching_building \
  obstacle_density:=medium repetitions:=3 random_seed:=42 \
  results_dir:=$WQI_BUILD_ROOT/experiment_results gui:=false rviz:=false
```

可用模式：

- `ugv_only`：仅 UGV。
- `uav_only`：仅 UAV。
- `cooperative`：UGV-UAV 协同。

运行论文完整对照矩阵：

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

该命令执行 `3 种模式 × 4 种障碍密度 × 3 个随机种子 = 36 次` 单目标实验。正式
运行前可先检查所有生成命令：

```bash
ros2 run delivery_evaluation experiment_matrix \
  --modes ugv_only,uav_only,cooperative \
  --densities none,medium --seeds 42 --dry-run
```

指标定义、控制变量和论文统计方法见
[`docs/evaluation_method.md`](docs/evaluation_method.md)。
任务书逐项完成状态和必须收尾顺序见
[`docs/completion_checklist.md`](docs/completion_checklist.md)。

## 8. 地图重新生成

修改校园布局后，在工作空间根目录执行：

```bash
python3 src/ugvcar_description/scripts/generate_campus_delivery.py
bash ./build_workspace.sh
source ./setup_workspace.bash
```

该脚本会重新生成 Gazebo world、占据地图和 Nav2 禁行掩膜。不要直接修改
源码仓库或外部产物目录中的 `build/`、`install/`、`log/` 生成文件。

## 9. 编译与自动测试

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash

bash ./test_workspace.sh --event-handlers console_cohesion+
```

2026-08-19 在外部构建目录执行的当前开发基线结果：

```text
16 packages finished
352 tests, 0 errors, 0 failures, 16 skipped
```

UGV 和 UAV Xacro 均通过 `check_urdf`；校园生成器能够重建 11 栋建筑、4 组闭合
道路、占据地图和禁行掩膜。详细运行证据见
[`docs/evaluation_report.md`](docs/evaluation_report.md)。

## 10. 当前限制与后续论文工作

软件主体、协同闭环、动态障碍和自动评测工具已经完成。论文提交前仍需：

1. 在冻结的 Git 提交上运行完整 36 次实验矩阵。
2. 保留所有 `runs.json`、`runs.csv`、汇总报告和图表。
3. 计算各组均值、样本标准差、失败原因和实时因子。
4. 对比 UGV、UAV 和协同模式的配送时间、路径长度、总能耗及单目标能耗。
5. 对比空载/有载、正常电量/低电量拒绝，以及不同障碍密度。
6. 使用论文参数或参考工况标定预测能耗与仿真实际积分能耗，并报告相对误差。

当前 UAV 使用 Gazebo ground truth 定位；已经实现航路边重规划和局部三维避障，但
没有实现未知环境中的在线三维 SLAM 或三维全局占据地图。论文中应如实说明这一边界，
不能用单次冒烟测试代替完整实验矩阵。UGV 是会在园区道路间移动的能源与货物基地，
但当前安全策略要求 UGV 到达固定会合点并停止后，UAV 才起飞或降落；尚未实现 UGV
行驶过程中的动态追踪降落。若任务书把“移动基地”严格解释为“车辆运动中着陆”，
该功能仍属于后续研究内容。
