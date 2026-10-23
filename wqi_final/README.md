# wqi_final：校园空地协同物流配送仿真系统

本项目是基于 ROS 2 Humble、Gazebo Classic、RViz2 和 Nav2 的毕业设计仿真平台，
面向校园最后一公里物流配送。系统包含可独立运行的 UGV 地面无人车、UAV 四旋翼
无人机，以及带任务规划、物理停靠、电量约束和自动充电的 UGV-UAV 协同配送流程。
UAV 飞行层仅使用 PX4 v1.16 SITL + ROS 2 Offboard。旧 SJTU 替代飞控已删除；
Gazebo Classic 的电机、传感器和机体物理仿真仍保留。

当前版本仅用于电脑端仿真，不需要部署到开发板。仓库已经删除 Hello World、基础
话题/服务示例、教学巡逻节点、rosbag 练习和硬件启动代码，仅保留与毕业设计直接相关
的源代码。`v1.0.0` 是清理后的第一版仿真基线。

旧校园的地图生成脚本和自定义控制界面已删除；已有场景及六阶段命令保留。
2026-10-19 新增三区布局包及其资源生成模块，以保证新 world、二维地图和俯视图使用
同一份坐标。已生成资产随源码交付，正常使用不需要重新生成。

## 当前地图：中央闭环、周边三区与四辆 UGV

按最新道路示意，**闭合主干道位于中央，功能区位于环路外围**，不是环路包围所有建筑。
A 区位于西北（教学楼、图书馆、实验楼、创新中心），B 区位于东北（四栋宿舍），
C 区位于南侧（食堂、体育馆）。物流中心在西南，公园不恢复。
保留原来全部建筑的尺寸、高度、门窗和碰撞体，仅调整位置。
按最新补充，总站另保留 UGV-00，三个区域各一辆，共四辆 UGV。UAV 从总站
车载平台起飞，在区域车上交接；返航后重新停靠总站车，不再从地面起降。

![与 Gazebo 世界坐标一致的校园俯视图](simulation_ws/src/campus_regional_delivery/assets/campus_layout_preview.png)

| 位置 | 地图坐标 x、y / m | 车辆命名空间 |
|---|---|---|
| 中央物流中心 / UGV-00 及车载 UAV | -56、-28 | `/ugv_00`、`/uav` |
| A 区配送点 / UGV-01 | -18、22 | `/ugv_01` |
| B 区配送点 / UGV-02 | 28、22 | `/ugv_02` |
| C 区配送点 / UGV-03 | 12、-33 | `/ugv_03` |

终端一，先关闭旧仿真，再编译并同时打开 Gazebo、RViz、四辆车及 PX4 UAV：

新四车入口默认使用官方 Cyclone DDS。首次准备环境执行
`sudo apt install ros-humble-rmw-cyclonedds-cpp`；后续每个终端都要使用相同的 RMW。

```bash
cd ~/design_final/wqi_final/simulation_ws
bash ./build_workspace.sh --executor sequential
source ./setup_workspace.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch campus_regional_delivery regional_campus.launch.py gui:=true rviz:=true
```

仅检查地图和四辆车时，在最后一条命令追加 `uav:=false`。
`gui:=false`、`rviz:=false` 可分别关闭对应窗口；`visualize_sensor_rays:=true`
只打开 Gazebo 雷达射线示意，默认关闭示意而不关闭传感器。
先生成四车和 UAV/PX4 模型，再分批激活四车导航，全部就绪后才打开 RViz，因此
RViz 会晚于 Gazebo 出现。UAV 生成和导航激活均有墙钟超时，不会无限等待。
新入口默认使用 Cyclone DDS，每车通过轻量退出适配容器加载官方 Nav2 组件，以减少四车负载。
VirtualBox 下新四车入口的 RViz 默认使用 `softpipe` 渲染，以避开本机一次崩溃所落入
的 LLVM 渲染路径。仅作用于 RViz，Gazebo、传感器和物理控制不变；可能更慢。
可追加 `rviz_render_driver:=llvmpipe` 恢复原渲染路径。该选项不表示首次地图 GLSL
警告已修复，也不替代实际画面检查。
`use_composition:=false` 可切回独立进程进行诊断；不会关闭生命周期心跳或传感器。
新静态地图使用 0.2 m 栅格，实际尺寸与建筑/配送坐标不变，局部碰撞地图仍为原
0.05 m。原单车/旧双车地图与参数、车辆速度及传感器不改；新旧实验应分开记录。

终端二，发送 A 区创新中心地面末端配送任务：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 action send_goal /regional_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['innovation_center'], payload_masses_kg: [0.30], return_home: true}" --feedback
```

此新流程为 UAV 从总站车起飞、降落到静止区域车上、模拟货物交接、区域车送达
并回配送点、UAV 返航并停回总站车。显式增加 `target_floors: [3]` 才执行楼层配送。
多订单可以使用 `targets: ['teaching_building','dormitory_1','cafeteria']`，并提供等长
载荷数组；调度按区域分组并按 1 kg UAV 载重分批，不宣称全局最优。

食堂二楼、0.30 kg、完成后返航的测试订单：

```bash
ros2 action send_goal /regional_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['cafeteria'], target_floors: [2], payload_masses_kg: [0.30], return_home: true}" --feedback
```

总站初始状态为 UAV 已装载在 UGV-00 上，模型插入时通过原严格物理捕获检查后
建立固定连接，避免等待订单期间在车顶滑移。这是初始装载，不是空中自动对接；
后续落车仍需 PX4 落地解除武装、静止平台、捕获距离/速度门限及实测稳定校验。
任务开始前也会复核初始锁定位置，不能只凭保留的 `docked` 标志起飞。
若停在 `UGV_PARKING`，查看实际等待项和捕获条件：

```bash
ros2 topic echo /uav/docked_carrier --qos-durability transient_local --once
ros2 topic echo /regional_delivery/capture_status --qos-durability transient_local
```

`capture_status` 在等待捕获时报告距离、四组过滤速度及具体阻断原因；超时结果
保留最后观测，不再误写为 `waiting for READY`。旧仿真进程不会自动加载修复，
修改后必须停止旧场景、重新编译并重启。

**只打开地图不会自动配送。**第一终端保持仿真运行，在第二终端发送上面的订单。
当前四车场景的接口是 `/regional_delivery/execute_mission`，不是旧双车的
`/cooperative_delivery/execute_mission`，也不是单 UAV 的 `/uav/execute_delivery`。
若一直 `Waiting for an action server`，先核对入口、两终端的 RMW 和 ROS_DOMAIN_ID，
不要仅根据车辆不动就调整导航或飞控。可以先检查：

```bash
ros2 action info /regional_delivery/execute_mission
# 应有 1 个 Action server；仅有客户端不代表服务端已经启动。
```

管理器启动会打印任务接口、运行模式和 DDS 域。订单被接受或拒绝都会写入日志及
保留话题，未知楼栋、超载、非法层数、已有活动任务不再静默拒绝：

```bash
ros2 topic echo /regional_delivery/task_admission --qos-durability transient_local --once
```

宿舍请使用 `dormitory_1` 至 `dormitory_4`，不要用含义不明确的 `dormitory`。
`Deactivating`、`Cleaning up`、`Destroying bond` 若出现在 Ctrl+C 之后，是生命周期
正常关闭步骤；请以任务 Action 的结果和关闭前第一条错误判断是否配送失败。

**判断 UAV 是否真正降落：**返航后不是降到地面，而是停到 UGV-00 的车顶平台。
当前 Iris 的中心目标高度约 0.42 m，平台顶面约 0.365 m，中心与平台之间的
0.055 m 是机体碰撞体下半厚度，不应把中心高度强行降到平台顶面。
`UAV_APPROACH` 是从巡航高度下降到 0.8 m；`UAV_LANDING` 才是最后的触地与解除武装。
配送 Action 的 `--feedback` 会持续更新下降剩余距离；也可在第三个终端检查：

```bash
ros2 topic echo /regional_delivery/landing_status --qos-durability transient_local
```

其中 `uav_z_m` 为实际中心高度，`target_z_m` 为接收车实际高度加停靠偏移，
`height_error_m` 为高度差。最终必须有 `phase: DOCKED_VERIFIED`、`verified: true`，
且 `carrier` 为目标车辆。该结论要求新鲜定位、PX4 已落地解除武装、实际已锁定，
并连续至少 0.5 仿真秒满足高度误差不超过 0.05 m、水平误差不超过 0.12 m。
定位陈旧或仍悬在 0.8 m 时，即使收到 LANDED 状态也不能宣布最终完成。

### 纯 UGV 对照模式

关闭上一套仿真后，使用同一张地图运行：

```bash
ros2 launch campus_regional_delivery regional_campus.launch.py \
  delivery_mode:=ugv_only gui:=true rviz:=true
```

该模式不生成 UAV、不启动 PX4，由总站 UGV-00 从总站出发，沿道路完成同一批
建筑门前配送并返库；三辆区域车留在各自配送点。发送任务仍使用上面的 Action。
对照任务不要填写 `target_floors`：纯地面车不能配送至上层，不能把地面与楼层任务
当成相同服务条件。新模式目前按区域及输入顺序服务，未宣称全局最优或完成能耗标定。

**验收边界：**2026-10-19 全工作空间基线为 16 包构建通过，666 项测试汇总为 0 错误、0 失败、23 跳过；
包含包装项，不能等同于真实配送次数。详见[分区验收记录](docs/regional_campus.md#5-验收边界)。
官方 Cyclone DDS 环境下，四车共 40 条双向路径规划、8 段实际短程运动和总站车
实际承载 UAV 已通过。该轮 UAV 起飞后出现速度融合中断并触发 PX4 原生降落，
不是完整协同通过；后续首次捕获超时也保留为失败证据。
后续组件化专项完成总站车短程往返、起飞、5 秒悬停、车顶降落/重锁、四车导航
恢复；最大水平漂移 0.0275 m、高度变化 0.1291 m。该轮 isolated 容器退出异常，
随后改为多线程组件容器；退出清理继续单独回归，不能称已全部稳定。
已修复任务管理器晚启动时漏收 UAV 保留状态而卡在 `PREPARING` 的 QoS 问题，
并增加先关闭生命周期、再卸载组件、最后关闭容器的退出保护。准备阶段可通过
`/regional_delivery/readiness` 查看具体缺少的条件。
早先三车版本的 20 条规划和 6 段实际短程导航是历史证据，不冒充四车整程验收。
新增一次 A 区创新中心完整闭环实测通过：总站车起飞、A 车降落锁定和模拟交货、
区域车送达并返回、UAV 返航降落锁定总站车。门前采样误差 0.0795 m；该轮退出
清理仍失败，配送成功与进程正常退出分别记录。最终独立四车退出专项已通过：
Gazebo/RViz 同时打开，总站车短程往返成功，四套导航各卸载全部 9 个组件，
7.95 秒正常退出，无段错误或强制终止；该条历史记录仅覆盖短程地面专项。
**2026-10-20 最新完整回归已通过：**使用正式 Action 完成创新中心 0.30 kg 单件
往返，UGV-01 接收、投递并返回，UAV 最后降落并锁定总站车，额外观察 5 仿真秒
未再升起。Gazebo 最终中心高度约 0.421 m，RViz TF 一致；本轮 RViz 与四套导航
正常清理，无非预期进程退出。新包重编译及 88 个用例、5 个 CTest 包装项通过。
这是一轮静态 A 区地面服务闭环，不是多区域/楼层/动态场景全部验收。
详细失败、专项和最终成功记录见 [实验报告](docs/evaluation_report.md#2026-10-20-车顶附近未降落报告的核查)。
**2026-10-21 食堂二楼配送功能回归通过：**启动后等待 300 墙钟秒再发单，0.30 kg
任务完成总站起飞、UGV-03 接收、食堂二楼悬停配送、返回区域车、区域车回站及
UAV 返航停靠总站，Action 为 SUCCEEDED。额外 5 仿真秒保持锁定，无再次升起。
本轮关闭时 RViz 出现 `GL_OUT_OF_MEMORY` 和堆损坏后退出 -6，因此整轮原始
`passed=false`，不能称整体稳定通过。定位节点退出竞态已调整退出顺序，真实 ROS
消息下 SIGINT/SIGTERM 专项均退出 0；最终新包 98 用例及 5 个 CTest 包装项通过。
当时尚未再次重跑包含该定位退出修改的完整 GUI 任务；RViz 退出问题仍待解决。
**2026-10-23 食堂返站修复：**后续运行在 C 车返站转角中止。空载车复现确认，
原规划线距不可通行边界约 0.576 m，小于 0.60 m 局部安全包络。只给新四车场景
增加全局规划净空；不改道路、速度、旧参数、局部碰撞保护或 PX4 控制参数。
107 个包内用例通过，包级 CTest 汇总 112 项零失败。Gazebo/RViz 同开，C 车两轮
实际往返共 4 段导航全部成功，碰撞预测警告 0，正常退出。完整协同回归结果以
[实验报告](docs/evaluation_report.md#2026-10-23-食堂返站转角的规划净空修复) 为准。
另修复了 Offboard 健康检查的多线程时间戳竞态，避免新接收时间被误判为负年龄；
保留原失联/应急降落门限。UAV 控制包和分区包合计 251 个用例通过。首次完整回归
在悬停后失败及 RViz 退出 -6 的记录均保留，未把专项通过等同于完整任务成功。
**最终食堂二楼完整配送功能已通过：**同开 Gazebo/RViz，0.30 kg 订单完成 C 车
接收、楼层服务、载机返站和 UAV 回总站落车，Action 成功且完成目标 1；整程碰撞
预测/融合丢失警告均为 0。Gazebo/TF 最终中心高度约 0.421 m，额外 5 仿真秒保持
锁定。但 RViz 关闭时仍退出 -6，故 `functional_passed=true, passed=false`，不宣称
图形退出稳定。原始数据、转角对照和降落曲线均保存在仓库外，详见同日实验报告。
B 区整程、C 区其他楼栋/楼层、全部楼栋及重复动态实验仍需逐项验收。
新入口尚未接入四车九电池联合约束，不移用旧双车能量结果。
完整坐标、命令、文件清单和验证记录见 [三区校园使用说明](docs/regional_campus.md)。
不要同时启动下文旧地图入口；它们使用不同校园坐标。

## 兼容入口：双车接力（旧校园）

导师确认的流程为：**物流总站 → A 车载 UAV 到 P1 → UAV 飞到 P2 并停靠 B 车 →
B 车载 UAV 到楼下 → UAV 配送至指定楼层并返回 B 车 → B 车返回 P2 → UAV 返航
并停靠 P1 的 A 车 → A 车载 UAV 回物流总站**。起降均在停车后的车载平台上进行。
原有六阶段和单车协同入口保留，不要与新接力入口同时运行。

终端一，编译并打开 Gazebo 和 RViz：

```bash
cd ~/design_final/wqi_final/simulation_ws
bash ./build_workspace.sh
source ./setup_workspace.bash
ros2 launch cooperative_delivery relay_delivery.launch.py gui:=true rviz:=true
```

终端二，等待 `/relay_delivery/execute_mission` 就绪后发送教学楼三层配送：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
ros2 action send_goal /relay_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['teaching_building'], target_floors: [3], payload_masses_kg: [0.30], return_home: true}" --feedback
```

两车独立导航、P1/P2 配置、短距离验收命令及实际验证边界见
[双车接力配送说明](docs/two_ugv_relay.md)。不能把旧单车实验数据当成新流程的验收结果。
双车入口会等待轮控制器和两套 Nav2 就绪；提交任务后，可用
`ros2 topic echo /relay_delivery/readiness` 查看缺少的接口。两车实时占用层仅在
此模式启用，不覆盖原地图或原 `nav2_params.yaml`。
2026-10-15 已完成短程一楼及后续三楼单包裹闭环验证；最新三楼任务载荷为 0.30 kg，
实际到达高度为 8 m，完整返还 A 车并回总站，独立验收 11 项通过。
这不是相同版本相同工况的重复统计；正式校园长航路、多目标与动态障碍的
双车重复实验仍待完成，具体证据和限制见上述说明第 7 节。

2026-10-16 已针对任务完成后的退出异常加入有序清理入口，继续复用官方 Nav2
过滤器信息节点。双车模式默认控制频率为 10 Hz、行为树周期为 100 ms、RViz
显示为 10 Hz；不修改单车默认频率、速度、地图、路径规划插件或停靠门限。
启动命令不变，可加 `controller_frequency:=20.0` 恢复双车旧控制频率。
两车短程导航及退出专项已通过，退出约 6.2 s，未重现段错误或强制终止；
完整测试汇总 592 项、0 失败、29 跳过（含测试包装项）。这不是新版本完整
楼层配送回归；仍有控制超时与首次 RViz 着色器警告，不能宣称全部性能问题已消除。

双车任务现在在移动车辆前预检整程五块电池：同时计算两车行驶与等待耗电、
货物交接后的质量变化、跨平台往返及楼层配送、有限充电源和两级充电损耗。
跨平台预测包含加减速和 PX4 起降等待，不再使用单一匀速近似。
可用 `ros2 topic echo /relay_delivery/energy_plan` 查看计划；这是配置条件下的
能量预测，不是已标定实测值，也不是全局最优调度。5% 充电电池专项实测已确认
在两车移动前拒绝任务，原始失败结果与独立负面验收分别保留。

## 1. 项目目录

```text
wqi_final/
├── README.md
├── third_party/                    # Git 子模块：固定版本飞控与通信源码
│   ├── PX4-Autopilot               # v1.16.0，含嵌套依赖
│   ├── px4_msgs                    # v1.16.0
│   └── Micro-XRCE-DDS-Agent         # v2.4.2
├── docs/
│   ├── system_architecture.md       # 系统架构、协同状态机和数据流
│   ├── completion_checklist.md      # 任务书逐项完成度和论文收尾清单
│   ├── evaluation_method.md         # 实验变量、指标、矩阵和统计方法
│   ├── evaluation_report.md         # 已完成的构建、测试和运行证据
│   ├── paper_based_dynamic_navigation.md # D* Lite 与预测式 DWA 的理论和实现
│   ├── cooperative_delivery.md      # 停靠插件和协同任务说明
│   ├── uav_subsystem.md             # UAV 动力学、传感器和安全控制
│   ├── uav_px4_backend.md            # PX4 SITL 安装、架构、接口和验收方法
│   └── uav_battery.md               # UAV 功率、电池和充电模型
└── simulation_ws/
    └── src/
        ├── ugvcar_description       # UGV 模型、现有 Gazebo 场景和配送配置
        ├── ugvcar_navigation2       # Nav2 地图、参数和启动文件
        ├── ugvcar_navigation2_interfaces # 动态障碍位置/速度结构化消息
        ├── ugvcar_application       # UGV 多目标配送管理器
        ├── uav_interfaces           # UAV Action 和服务接口
        ├── uav_description          # UAV 模型、传感器、TF 和 RViz 配置
        ├── uav_control              # 飞行控制、电量模型和三维安全监控
        ├── uav_navigation           # UAV 航点、15 m 航路图和路径规划
        ├── uav_application          # UAV 配送任务状态机
        ├── uav_bringup              # 独立 UAV 校园仿真启动
        ├── uav_px4_bringup          # PX4 SITL 与 Micro XRCE Agent 启动
        ├── cooperative_delivery_interfaces # 空地协同 Action 接口
        ├── cooperative_delivery     # 协同管理器、停靠插件和联合启动
        ├── campus_dynamic_obstacles # 可复现的地面/空中物理动态障碍
        ├── delivery_evaluation      # 自动实验、指标采集、报告和图表
        └── campus_regional_delivery # 新中央环路、总站+三区四车、协同/纯UGV入口
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
- UAV 使用 PX4 SITL
  的状态估计、位置/姿态控制和电机输出，通过 Gazebo Classic Iris 电机插件驱动模型。
  不使用 `/set_entity_state` 持续修改坐标模拟飞行。
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
- 通过 ROS 2 Action 和状态话题设置目标、楼层与载荷，并在 RViz 中查看轨迹、模型
  和电池状态；自定义 PyQt5 控制界面不属于当前源码交付。

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
  uav_interfaces uav_description uav_control \
  uav_navigation uav_application uav_bringup uav_px4_bringup
source ./setup_workspace.bash
```

编译空地协同系统及其依赖：

```bash
bash ./build_workspace.sh --packages-up-to cooperative_delivery uav_bringup
source ./setup_workspace.bash
```

### 3.2 获取固定源码并安装 PX4 SITL

PX4 是阶段 3 至 6 必需的唯一飞控。飞控源码和通信源码已经通过 Git 子模块纳入
`third_party/`，不要把子模块误认为可省略的外部文件。新电脑应递归克隆：

```bash
git clone --recurse-submodules https://github.com/liyongqihhh/wqi_final.git
cd wqi_final/wqi_final/simulation_ws
```

普通 clone 后先在仓库根目录运行 `git submodule update --init --recursive`。
GitHub ZIP 不包含子模块内容。原有本机目录不需要再次 clone。
安装脚本从固定源码准备外部构建副本，检查版本，应用项目补丁；固件、库、虚拟环境、
构建副本和日志仍保留在仓库同级的 artifacts 目录，不写入 `third_party/`：

```bash
cd ~/design_final/wqi_final/simulation_ws
bash ./setup_px4_backend.sh --install-system-deps
bash ./build_workspace.sh
source ./setup_workspace.bash
```

如果系统依赖已经安装，可省略 `--install-system-deps`。第一次下载和编译耗时较长，
并需要约数 GB 可用磁盘空间。安装脚本固定使用 PX4 `v1.16.0`、`px4_msgs v1.16.0`
和 Micro XRCE DDS Agent `v2.4.2`，避免消息版本不一致。

独立启动 PX4 UAV：

```bash
ros2 launch uav_bringup uav_sim.launch.py \
  flight_backend:=px4 gui:=false rviz:=true
```

联合启动 PX4 UAV 与 UGV：

```bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  flight_backend:=px4 gui:=false rviz:=true \
  enable_energy_constraints:=false enable_dynamic_obstacles:=false
```

检查实际启用的后端和 PX4 DDS 连接：

```bash
ros2 topic echo /uav/flight_backend --once
ros2 topic echo /uav/px4/connected --once
ros2 topic echo /fmu/out/vehicle_status_v1 --once
```

应分别看到 `PX4_SITL_OFFBOARD`、`true` 和 PX4 飞行状态。完整设计与故障检查见
[`docs/uav_px4_backend.md`](docs/uav_px4_backend.md)。

### 3.3 源码与构建产物清理

`simulation_ws/` 中不保留旧的 `build/`、`install/`、`log/`。继续使用
`bash ./build_workspace.sh` 编译，用 `source ./setup_workspace.bash` 加载外部安装环境；
不要再执行 `source install/setup.bash`。构建和环境脚本禁用自动 Python 字节码写入，
避免源码目录重新出现 `__pycache__`。

2026-10-12 已删除重复旧构建、停用 SJTU 产物、Python 缓存和 `.o` 编译中间文件。
当前外部运行库、PX4 固件、源码、Git 元数据及实验数据保留，因此无需为了启动
仿真重新编译；下次修改源码后构建会重新生成缺少的 `.o`，耗时会有所增加。

清理清单位于外部 `cleanup_20261012/result.json`，旧构建日志无损归档为 tar.gz。
86 份早于 2026-10-11 的 PX4 飞行日志无损压缩至
`cleanup_20261012/ulog_archive/`，逐份 SHA256 校验并在 `ulog_manifest.jsonl` 记录
原路径和归档路径；最近日志和已归档的回归原始记录没有删除。读取旧 ULog 前，可用
`gzip -dk <文件>.ulg.gz` 解压到归档旁边，不应把归档当成无用缓存删除。

## 4. 四档动态障碍启动方法

动态障碍共有四档。障碍是带碰撞体的 Gazebo 实体，通过物理速度沿固定路线往返，
不是连续修改坐标的动画。相同 `random_seed` 会选择相同路线，便于重复实验。

通过联合启动文件的 `obstacle_density` 参数选择 `none`、`low`、`medium` 或 `high`。
阶段 1 至 5 不启用动态障碍，阶段 6 的完整启动方法见第 4.1 节。

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
`/ugv/tracked_dynamic_obstacles`。全局占据 `/scan_dynamic_predictions` 和
`/points_dynamic_predictions` 随每帧测量更新，仅包含当前位置及最多 `1 s / 0.45 m`
的短时运动范围，不再用固定长虚拟墙封闭整段道路或路口。预测占据只进入全局代价
地图；D* Lite 以 `1 Hz` 检查路径，无效时重新规划，每 `5 s` 检查能否释放绕行。
有效旧路径仅在新路径明显缩短时替换，规划暂时失败不会反复取消控制器。
动态模式直接保留通过整车净空检查的 D* Lite 路径，不再用全局平滑切掉道路
安全余量；连续运动轨迹由局部控制器生成。普通校园和室内导航的平滑不变。
DWB 以 `10 Hz` 使用 Nav2 的 `StandardTrajectoryGenerator` 积分加速度约束下的
非完整约束轨迹，自定义预测碰撞评价器在 `8 s` 局部时域内比较候选轨迹
与运动障碍物的碰撞时间和净空，再选择安全且保持前进的速度。局部代价地图仍只处理
真实 `/scan`，承担静态几何约束和及时清障；前向碰撞监视器仅作为最后的紧急停止层。
前方、横穿和斜后方追近的障碍使用同一套相对运动模型，不执行固定倒车或固定角度
转弯；恢复仅使用 Nav2 的代价地图刷新和条件等待。预测器跟随控制器实际执行的
`/received_global_plan`，扫描等待对应时刻的实测位姿，避免候选路线掉头或回调延迟
让避让方向翻转。D* Lite 和 DWB 均检查整车净空，而不只检查车辆中心；全局道路
规划额外保留 `0.15 m` 跟踪余量。低速航向前视点沿剩余路径向前选取，
不会指向车辆后方的历史点。
普通校园导航保留原有全局 `1.0 m`、局部 `0.65 m` 软边界。
动态模式使用完整铺装路面及现有 `0.30 m` 软缓冲带，不对地图未知区重复膨胀；
禁行道路外区域仍由 keepout 限制。障碍膨胀半径保持 `0.8 m`，
`0.60 m` 协同外廓和 `0.30 m` 碰撞安全余量没有缩小。

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

## 5. 命令行任务与监控

自定义控制界面源码已经移除。使用第 6 节对应阶段的 `ros2 launch` 命令启动仿真，
再用 `ros2 action send_goal` 设置配送目标、楼层和逐件载荷。RViz 的模型、轨迹、
电池可视化及 Gazebo 界面保持不变。

例如，联合仿真就绪后向教学楼三层投递 0.30 kg 货物：

```bash
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['teaching_building'], return_home: true, \
  target_floors: [3], payload_masses_kg: [0.30]}" --feedback
```

阶段 1 仍在 RViz 中手动标点。三个电池的状态可在各自终端订阅：

```bash
ros2 topic echo /uav/battery_state
ros2 topic echo /ugv/drive_battery_state
ros2 topic echo /ugv/charging_battery_state
```

UAV 在航路中以 `15 m` 高度巡航，在建筑物正面配送点使用楼层中心高度：

```text
delivery_height = 1.6 + (floor - 1) * 3.2 m
```

初始电量和障碍密度都是启动参数。Gazebo 已经运行后再修改这些值不会改变当前仿真，
需要在原启动终端按 `Ctrl+C` 后重新启动。UAV 飞行电池安全储备为 `20%`，
UGV 驱动电池为 `20%`，UGV 充电电池为 `10%`。低于联合任务需求时，Action
结果和日志会说明是 UAV 架次、UGV 驱动还是充电预算不足。

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

推荐使用一个终端同时启动校园 Gazebo、小车、Nav2 和 RViz，避免只启动导航却没有
小车模型与 `odom`。每个新终端先执行 `source ./setup_workspace.bash`：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
ros2 launch ugvcar_navigation2 campus_ugv.launch.py gui:=false rviz:=true
```

同时打开 Gazebo 界面时改为 `gui:=true rviz:=true`；VirtualBox 推荐一次只打开一个
三维界面。此入口默认给 RViz 启用 Mesa 软件渲染，物理仿真仍正常运行；原生显卡
可追加 `rviz_software_rendering:=false`。切换之前先在旧仿真/导航终端按 `Ctrl+C`，
不要同时运行新入口和以下分终端启动方式。等待模型与导航就绪后再标点或发送任务。
入口会先检查旧节点；出现 `Previous simulation/navigation nodes are still running`
时，说明旧导航/仿真终端尚未退出。不要只关闭 RViz 窗口，应在启动它的终端按
`Ctrl+C`，等待退出后重试。重复运行两套同名 Nav2 会导致生命周期激活失败，
并出现 `Goal was rejected by server`；用 RViz 重新设置初始位置无法解决此冲突。

以下是可选的分终端启动方式，两个终端必须同时保持运行。

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

上面默认使用 PX4 SITL。可显式指定唯一支持的后端：

```bash
ros2 launch uav_bringup uav_sim.launch.py \
  flight_backend:=px4 gui:=false rviz:=true \
  visualize_sensor_rays:=false enable_energy_constraints:=false
```

VirtualBox 推荐二选一：

```bash
ros2 launch uav_bringup uav_sim.launch.py gui:=false rviz:=true
ros2 launch uav_bringup uav_sim.launch.py gui:=true rviz:=false
```

**切换到独立 UAV 前，先停止旧仿真。** 不能同时启动两套使用同一 Gazebo
端口的仿真。在旧仿真终端按 `Ctrl+C`，等待退出后再执行上述命令。
若提示 `Gazebo master ... is already running`，说明旧仿真仍占用端口，
本次启动已被拒绝，并不是 UAV 模型丢失。Gazebo 异常退出时会停止整套独立
UAV 启动，避免地图和任务节点仍运行而无人机没有生成。

PX4 模式下，先加载环境，等连接状态为 `true` 再发送任务：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
ros2 topic echo /uav/px4/connected --once
```

`setup_workspace.bash` 会加载已安装的 `px4_msgs` 环境；没有安装 PX4 时先执行 3.2 节，
不再提供替代飞控回退。

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

垂直升降段会先刹停水平运动并对准目标 XY，再保持同一条垂直轴线；若真实
障碍物挡住该轴线，安全监测仍会阻止直飞或触发避障，不会为了直线关闭雷达。
垂直航段使用 `vertical_path_clearance: 2.2`（`1.8 m` 安全球加 `0.4 m`
跟踪余量），水平航段仍使用 `dynamic_path_clearance: 2.5`。物流中心起飞点距
墙约 `2.5 m`，不再因为水平缓冲恰好碰墙而反复横向绕行。参数位于
`simulation_ws/src/uav_control/config/flight_control.yaml`；修改后重新编译并重启仿真。
四栋宿舍配送后先在当前楼层高度直飞宿舍中心 `(30, 11)`，再在中心调整至
`15 m` 巡航高度，避免先沿楼墙下降后才离楼。返航和任务电量预算使用同一
离楼航段，不重复计算这段距离。协同模式仍直接返回位于宿舍中心的 UGV。

后方障碍的预警绕行需要检测到距离在缩短；正在远离的静态楼墙不会仅因为
位于后方就反复触发绕行。进入安全近距离的物体仍受保护。每个新的飞行目标
都会刷新 `/uav/replanned_path`，不会在 RViz 中残留上一段的绕行线。

主要接口：

- Action：`/uav/fly_to_pose`、`/uav/execute_delivery`
- 服务：`/uav/takeoff`、`/uav/land`、`/uav/check_delivery_energy`
- 状态：`/uav/odom`、`/uav/imu`、`/uav/flight_state`、`/uav/mission_status`
- 飞控后端：`/uav/flight_backend`；PX4 专用连接状态：`/uav/px4/connected`
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

需要使用 PX4 时，在同一命令中追加 `flight_backend:=px4`。联合启动文件仍只启动
一个 Gazebo，并保持现有协同 Action、航点、电量和停靠接口不变。

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
仅当 E_available >= E_required 时允许 UAV 执行飞行架次
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
E_charge_source = (E_uav_stored + E_uav_docked_idle)
                 / (eta_uav_charge * eta_ugv_transfer) + charger_idle_energy
E_system = E_ugv_drive_used + E_ugv_charge_pack_used
           + E_uav_used - E_uav_stored
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

飞行期间应经过起飞、悬停、巡航和降落耗电状态。重新停靠后，若还有下一件配送，
按下一架次预算恢复充电；最后一件配送完成后关闭补电，以免将任务结束后的充电
混入配送能耗。停靠但未充电时仍有待机耗电。

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

#### 6.5.4 PX4 统一电量与能量调度

PX4 负责飞控；论文功率模型根据实际里程计速度、加速度和货物质量积分电量。
开启能量约束时，关闭 PX4 默认按解锁时间掉电的 `BatterySimulator`，通过 MAVLink
将 `/uav/battery_state` 注入 PX4，保留其 SOC 低电量保护。PX4 电量反馈量化到
1 个百分点，界面和调度器仍使用原始连续 SOC。它不是实测电池数据。

旧版已安装 PX4 需要再次执行 `bash ./setup_px4_backend.sh`，安装脚本会自动应用
外部电池“未知剩余时间”兼容补丁并重新编译；未提供的时间不能被当成 0 秒。

终端 1：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  flight_backend:=px4 gui:=false rviz:=true \
  enable_energy_constraints:=true enable_dynamic_obstacles:=false \
  route_policy:=energy_time charge_policy:=as_needed \
  initial_battery_percentage:=0.30
```

终端 2：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
ros2 topic echo /uav/px4/connected --once
ros2 topic echo /uav/px4/battery_bridge_status --once
ros2 topic echo /uav/px4/velocity_aiding_status --once
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['laboratory','library','dormitory_2'], return_home: true, target_floors: [4,3,4], payload_masses_kg: [0.35,0.25,0.20]}" --feedback
```

等待连接为 `true`、电量桥为 `SYNCED:paper_energy_model`、联合模式速度辅助为
`ACTIVE:GAZEBO_VELOCITY_HEIGHT_EKF` 后发送任务。
联合模式的返航点来自 `/uav/carrier_home` 的实际停稳车辆中心，不再把道路标定点
直接当作甲板中心。PX4 仍需真实接触、落地并上锁才能重新停靠。
联合 PX4 启动自动启用移动基座 SITL 配置，避免运输时把 UGV 运动当成 GPS
水平漂移或陀螺零偏；独立 UAV 保持原检查。当前定位无效时会拒绝起飞并保持停靠。
这些参数只适用于仿真，不能用于实体飞控。具体参数及故障保护见
[`docs/px4_energy_scheduling.md`](docs/px4_energy_scheduling.md)。
联合 PX4 还融合已有 Gazebo 里程计的三维速度与高度，防止连续返航时垂直估计
偏差干扰甲板着陆判定；不融合其水平位置或姿态，不强制空中上锁。它仍是 ground-truth
辅助仿真，不是 VIO/SLAM。独立 UAV 默认不启用此项。
电量足够时直接配送；沿途充电仍不足时，在停靠点进入 `CHARGING`，达到下一次
配送与安全余量后再放飞。充电电池或驱动电池预算不足时拒绝任务。

2026-10-05 的返航降落修复已跑通“物流中心运输到宿舍、宿舍 2 的 11 层投递、返回并
重新停靠”一次。复测时可将终端 1 的 `initial_battery_percentage` 设为 `0.80`，
等待连接及电量桥就绪后，在终端 2 发送：

```bash
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['dormitory_2'], return_home: false, target_floors: [11], payload_masses_kg: [0.65]}" --feedback
```

这里 `return_home: false` 仅表示 UGV 留在宿舍；UAV 仍返回小车并停靠。
该单次回归未开启动态障碍，也不是原 10 件物资批次全部通过。

2026-10-06 连续四架次着陆专项复测：先退出旧仿真并重新编译，终端 1 改为：

```bash
cd ~/design_final/wqi_final/simulation_ws
bash ./build_workspace.sh --executor sequential
source ./setup_workspace.bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  flight_backend:=px4 gui:=false rviz:=false \
  enable_energy_constraints:=true enable_dynamic_obstacles:=true \
  obstacle_density:=high random_seed:=42 route_policy:=input_order \
  charge_policy:=as_needed initial_battery_percentage:=1.0
```

终端 2 加载 `setup_workspace.bash`，等待上述三个就绪话题确认后执行：

```bash
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['dormitory_2','dormitory_1','dormitory_2','dormitory_3'], return_home: false, target_floors: [11,3,4,3], payload_masses_kg: [0.65,0.20,0.20,0.20]}" --feedback
```

这会复测上次失败的第四架次，不是完整 10 件任务或多种子实验。需要可视化时将
`gui` 或 `rviz` 改为 `true`，VirtualBox 建议只开一个三维界面。
联合 PX4 在稳定接触甲板后交接原生 `NAV_LAND`，仍等待真实落地和非强制
解除武装后才停靠；不会把接触检测直接写成 `LANDED`。当前固定质量 Iris 仿真
不使用在线悬停推力估计，使用本轮空中数据标定的归一化悬停推力 `0.71`。
它不是实际载荷变化后的飞控标定。`/uav/px4/landing_fault` 正常为空；若出现
故障原因，安全恢复落地也不会被计为配送成功。

以上四架次在 2026-10-06 已实际通过：`success: true`、`completed_targets: 4`，
四次均由 PX4 确认落地、非强制解除武装后再锁定，未触发应急故障。停靠锁定前
水平偏差分别约 `0.021 / 0.040 / 0.083 / 0.021 m`。数据和曲线保存在
`~/design_final_artifacts/regression_px4_multisortie_20261006_height/`，详见
[实验报告](docs/evaluation_report.md)。该结果为无界面、固定种子的单批回归，
不代表原 10 件整批、所有密度或多种子均通过。接触时估计速度仍有扰动，不能
据此声称完成实体甲板着陆标定。

2026-10-06 下午的再次运行在第二架次宿舍 1 返航时失败：机体已在约
`0.421 m` 的甲板高度停稳，但原生估计垂直速度持续抖动，30 个仿真秒内未确认落地。
因此上面的单批通过不等于已经解决重复性问题。此次新增接触感知 IMU 兼容补丁：
只在联合 PX4 的生成模型中启用，用原生 IMU 链路的物理速度差分计算加速度，
先平均连续两物理步的速度变化，再增加 20 ms 加速度低通，以抑制接触高频扰动在 MAVLink 下采样时
发生混叠。保留陀螺仪、噪声、EKF、旋翼动力学和原生落地判定，不伪造落地或强制解除武装。

已经安装 PX4 的电脑必须先关闭旧仿真，执行一次下面的离线插件更新；仅
`colcon build` 不会重编译外置 PX4 插件。新安装执行 `setup_px4_backend.sh` 时自动应用。

```bash
cd ~/design_final/wqi_final/simulation_ws
bash src/uav_px4_bringup/scripts/refresh_gazebo_imu.sh \
  "${PX4_AUTOPILOT_DIR:-/home/wqi/design_final_artifacts/px4/PX4-Autopilot}"
bash ./build_workspace.sh --executor sequential
source ./setup_workspace.bash
```

随后重新启动上面的联合仿真。插件未更新时启动检查会明确拒绝，不能继续
使用临时目录中的旧模型。着陆超时现在会显示实际状态、高度、期望高度和速度。

最终抗混叠版本已实际完成上述四件任务的着陆专项复测：`success: true`、
`completed_targets: 4`，四次均正常落地、解除武装并重新停靠。接近甲板至停靠
分别约 `3.8 / 3.8 / 3.8 / 4.0` 个仿真秒，原生接触段估计垂直速度最大约
`0.023 m/s`，低于未修改的 `0.25 m/s` 落地门限。最终 SOC 为 `76.27%`。
数据与曲线位于
`~/design_final_artifacts/regression_px4_multisortie_20261006_contact_antialias_verified/`。
该轮关闭动态障碍以隔离着陆故障，保留真实 UGV 道路导航、PX4 物理飞行和电量约束；
不能当作高密度十件整批或多种子通过。高密度运输段中断和回执超时记录保留在报告中。
重放时使用上面的四目标命令，将仿真启动参数改为
`enable_dynamic_obstacles:=false obstacle_density:=none`，其余楼层、载荷及电量不变。

终端 3：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
ros2 topic echo /cooperative_delivery/energy_plan
# 或查看充电目标、电池输入功率：
# ros2 topic echo /uav/charging_target_soc
# ros2 topic echo /uav/charging_input_power_w
```

阶段 5、6 通过 `route_policy` 和 `charge_policy` 启动参数选择排序与充电策略；
修改后需重新启动仿真。
`energy_time` 按三电池预算联合评价时间、载荷相关能耗与充电等待；最多 6 件货物
枚举顺序，7 至 10 件使用有限轮次 2-opt，不宣称全局最优。`distance` 为最短道路
顺序，`input_order` 为输入顺序。`full` 每次起飞前充满，`as_needed` 默认多留
`1 Wh` 缓冲；最后一次配送结束后的返程不再为下一架次充电。

独立重启的三策略对照实验（先用 `--dry-run` 检查，再移除此选项实际运行）：

```bash
ros2 run delivery_evaluation experiment_matrix \
  --modes cooperative --densities none --seeds 42,43,44 \
  --scenario multi_target_regression --flight-backend px4 \
  --energy-ablation --repetitions 1 --dry-run
```

输出记录与图表仍放在外部 artifacts 目录，按后端、排序策略和充电策略分组。
详细数学关系、测量口径和限制见 [PX4 能量调度说明](docs/px4_energy_scheduling.md)。

先检查低电量补电与起降交接，可执行短架次回归：

```bash
ros2 launch delivery_evaluation experiment.launch.py \
  mode:=cooperative scenario:=logistics_center_smoke repetitions:=1 \
  flight_backend:=px4 route_policy:=energy_time charge_policy:=as_needed \
  initial_battery_percentage:=0.19 obstacle_density:=none \
  gui:=false rviz:=false \
  results_dir:=$WQI_BUILD_ROOT/experiment_results
```

此场景在物流中心上方完成补电、起飞、悬停、投递模拟、降落和重新停靠，UGV 不
驶向其他楼栋，不是校园整程避障测试。成功后自动退出并生成 JSON、CSV 和图表。
2026-10-05 使用 `initial_battery_percentage:=0.22` 的该短架次已通过：补电等待
59.4 仿真秒，任务结束电量 22.55%，真实甲板着陆后重新停靠；实测记录见
[实验报告](docs/evaluation_report.md)。此单次结果不代表多目标或动态障碍全部通过。
PX4 起飞及落地确认额外预算默认分别为 20 和 10 仿真秒，可在电池 YAML 中调整；
这是预测用工程余量，不会人为额外扣除实际电量，也不是实物标定值。

### 6.6 阶段六：动态障碍、电量约束与空地协同配送

阶段六在阶段五全部能量约束基础上启动带碰撞体的动态障碍，作为最终完整系统。
使用 `obstacle_density` 启动参数选择四档之一。
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

## 8. 地图资源与维护

地图生成脚本已经移除。仿真直接加载随源码保存的资源，无需运行生成器：

- `src/ugvcar_description/world/`：房间与校园 Gazebo world。
- `src/ugvcar_navigation2/maps/`：占据地图和 Nav2 禁行掩膜。
- `src/ugvcar_description/config/campus_layout.yaml`：运行时道路网络和布局配置。
- UGV/UAV 各自的配送点配置：任务输入及航路规划仍需使用。

不能单独修改 world 而不对齐导航地图、禁行掩膜与配送点，否则 Gazebo 与 RViz 会
不一致。资源变更后使用 `bash ./build_workspace.sh` 安装；不要直接修改 `build/`、
`install/`、`log/` 中的生成文件。

## 9. 编译与自动测试

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash

bash ./test_workspace.sh --event-handlers console_cohesion+
```

2026-10-02 在外部构建目录执行的历史基线结果（包含现已移除的软件包）：

```text
17 packages finished
359 tests, 0 errors, 0 failures, 16 skipped
```

历史验证中 UGV 和 UAV Xacro 均通过 `check_urdf`；现有校园资源包含 11 栋建筑、
闭合道路、占据地图和禁行掩膜。详细运行证据见
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

PX4 后端已经完成 DDS 连接、物理起飞、Offboard 短距离 `FlyToPose` 和自动降落上锁
回归，但尚未完成当前 PX4-only 版本上的全部校园多目标、四档动态障碍和 36 次
正式实验矩阵。因此论文可将它描述为“已实现且通过基础闭环验证的 PX4 飞控集成”，
不能写成已经完成硬件飞行或全部统计实验。
