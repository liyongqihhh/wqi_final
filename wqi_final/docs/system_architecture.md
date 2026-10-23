# 校园 UGV-UAV 协同配送系统架构

## 系统边界

## 当前四车分区场景

2026-10-19 新入口为 `campus_regional_delivery/regional_campus.launch.py`。
一套 Gazebo 中配置总站 UGV-00 与 A/B/C 区的 UGV-01/02/03，共享静态地图，
各自维护 TF、Nav2、目标和局部代价地图。中央环路、世界、导航图与任务坐标
来自同一配置。旧单双车入口保留，不能与新入口同时启动。

```mermaid
flowchart TD
    CFG[campus_regions.yaml<br/>单一布局坐标] --> ASSET[新 world / Nav2 地图 / 航点 / 俯视图]
    ORDER[订单：楼栋、楼层、载荷] --> MODE{配送模式}
    MODE -->|协同| BATCH[区域分组 / UAV 载重分批]
    BATCH --> HUB[总站车物理停靠检查]
    HUB --> AIR[UAV 起飞并空运至区域]
    AIR --> PARK[区域车静止 / UAV 降落解除武装]
    PARK --> CAPTURE[原始物理门限检查 / 固定关节锁定]
    CAPTURE --> HANDOVER[模拟货物交接事件]
    HANDOVER --> LAST[所属区域车末端配送 / 可选楼层飞行]
    LAST --> DEPOT[区域车回本区配送点]
    DEPOT --> BACK[UAV 返航并重新停靠总站车]
    BACK --> DONE[任务完成]
    MODE -->|纯 UGV| GROUND[总站车沿道路访问相同门前目标]
    GROUND --> RETURN[总站车返回总站]
    RETURN --> DONE
```

地面规划复用原静态 SmacPlanner2D + Regulated Pure Pursuit；UAV 仍为 PX4 SITL
+ ROS 2 Offboard。接收车必须停车，飞行期间暂停静止车辆的地面 Nav2，恢复后才
允许地面导航。真实锁定和承载、40 条规划、8 段短程运动已实测；另有一次 A 区
创新中心空运/交接/地面配送/返航闭环成功，但该轮导航容器退出仍失败。B/C 整程
与重复验证尚未完成，不能把下方旧单双车能耗与重复实验直接算作四车完成项。
后续只修改退出适配的四车地面专项完成全部组件卸载，7.95 s 正常退出；不冒充
最终代码下重新通过了完整空运任务。
四驱动电池、四充电源和 UAV 电池的九电池约束尚未接入新入口。
坐标、命令和失败记录见 [三区四车说明](regional_campus.md)。

## 兼容入口边界

本项目的最终交付范围是 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 和
RViz 上的电脑端物理仿真。系统不依赖开发板、真实电池或真实电流传感器；UAV
仅使用 PX4 SITL 软件在环飞控和 ROS 2 Offboard 接口。

新增的双车接力入口为 `relay_delivery.launch.py`，由独立管理器编排 A 车、跨区
飞行、B 车接机、楼层配送和反向交接，详见[双车接力架构与验收](two_ugv_relay.md)。
下面的图保留单车基线结构；双车模式有两套独立 Nav2、四块 UGV 电池和一块 UAV
电池，不能将旧单车实验记录当作双车流程的实验结果。

```mermaid
flowchart LR
    CLI[ROS 2 命令行<br/>任务与仿真控制] --> CM[cooperative_delivery<br/>协同任务管理器]
    CLI --> UA[uav_application<br/>独立 UAV 任务管理器]
    CLI --> GA[ugvcar_application<br/>独立 UGV 任务管理器]

    CM --> NAV[Nav2<br/>静态 Smac+RPP<br/>动态 D* Lite+DWB]
    CM --> UA
    CM --> DOCK[Gazebo 固定关节<br/>对接与解锁]

    NAV --> UGV[UGV ros2_control<br/>差速驱动与传感器]
    UA --> FC[uav_control<br/>飞行 Action 与安全控制]
    FC --> PX4[PX4 SITL + DDS<br/>ROS 2 Offboard]
    PX4 --> UAV[Gazebo Classic Iris<br/>电机与机体物理仿真]

    BM[uav_control/battery_manager<br/>耗电 准入 充电] --> UA
    BM --> CM
    UGVBM[cooperative_delivery/ugv_energy_manager<br/>驱动电池 UAV充电电池] --> CM
    UGVBM --> UGV
    UGVBM --> BM
    CM --> UGVBM
    DOCK --> BM
    DOCK --> UGVBM

    WORLD[campus_delivery.world<br/>建筑 道路 碰撞环境] --> UGV
    WORLD --> UAV
    DYN[campus_dynamic_obstacles<br/>非合作固定路线物理障碍] --> US[UGV 360度激光]
    DYN --> AS[UAV 顶部 3D 雷达]
    US --> TRACK[动态目标位置速度跟踪]
    TRACK --> DSTAR[D* Lite<br/>增量全局路径修复]
    TRACK --> PDWA[预测式 DWB<br/>候选轨迹碰撞时间]
    DSTAR --> NAV
    PDWA --> NAV
    NAV --> UGV
    AS --> FC
    UGV --> VIZ[RViz 与状态话题]
    UAV --> VIZ
    CM --> VIZ
    UGV --> METRICS[delivery_evaluation<br/>路径 时间 误差 能耗]
    UAV --> METRICS
    CM --> METRICS
    METRICS --> REPORT[JSON CSV Markdown 图表]
```

## 软件包职责

| 软件包 | 职责 |
|---|---|
| `ugvcar_description` | UGV 模型、传感器、现有房间/校园 world 和配送配置 |
| `ugvcar_navigation2` | Nav2 地图、代价地图、行为树和启动文件 |
| `ugvcar_navigation2_interfaces` | UGV 动态障碍位置、速度、半径和置信度结构化消息 |
| `ugvcar_application` | UGV 多目标顺序优化与配送执行 |
| `uav_interfaces` | UAV Action 和能量检查 Service |
| `uav_description` | UAV Xacro、碰撞体、3D 雷达和下视传感器 |
| `uav_control` | 起降、定点飞行、安全球、电池与充电模型 |
| `uav_navigation` | 配送楼层点、空中航路图和路径可视化 |
| `uav_application` | UAV 配送状态机和能量准入 |
| `uav_bringup` | 独立 UAV 仿真组合启动 |
| `uav_px4_bringup` | PX4 SITL 和 Micro XRCE DDS Agent 启动 |
| `cooperative_delivery_interfaces` | 空地协同 Action |
| `cooperative_delivery` | UGV-UAV 调度、对接、联合启动、双 UGV 电池和能量序列规划 |
| `campus_dynamic_obstacles` | 可复现的非合作地面/空中物理动态障碍，只按固定路线运动 |
| `delivery_evaluation` | 实验矩阵、指标采集、聚合报告和图表 |
| `third_party/` | 固定版本 PX4、消息和 DDS Agent Git 子模块源码 |

## 协同配送主流程

UGV 在固定会合点停稳后 UAV 才能解锁。UAV 返回并重新对接后，UGV 才能继续
下一个地面目标或返回物流中心。

```mermaid
flowchart TD
    A[接收目标 楼层 逐件载荷 三块初始电量] --> B[校验参数 优化 UGV 顺序 更新剩余货物]
    B --> C{是否为阶段5或6?}
    C -- 否 --> D[确认 UAV 已停靠]
    C -- 是 --> C1{UAV飞行 UGV驱动 UGV充电<br/>三块电池均可行?}
    C1 -- 否 --> R[车辆移动前拒绝任务<br/>保持安全停靠]
    C1 -- 是 --> D
    D --> E[UGV Nav2 前往建筑门口停靠点]
    E --> F{UGV 到点并稳定?}
    F -- 否 --> X[清理代价地图并有限重试]
    X --> E
    F -- 是 --> G0{是否需要停靠补电?}
    G0 -- 是 --> G1[保持停靠 CHARGING<br/>源电池储备不足则失败]
    G1 --> G0
    G0 -- 否 --> G[起飞前重新检查 UAV 架次电量]
    G --> H[解锁 UAV 并起飞到目标楼层高度]
    H --> I[飞往楼栋正门配送点]
    I --> J[悬停并发布 DELIVERED 事件]
    J --> K[返回 UGV 上方并下降]
    K --> L{物理降落和对接成功?}
    L -- 否 --> Q[任务失败并记录原因]
    L -- 是 --> M[确认锁定并刷新代价地图]
    M --> N{还有目标?}
    N -- 是 --> N1[设置下一架次充电目标]
    N1 --> E
    N -- 否 --> O0[关闭后续补电]
    O0 --> O[UGV 返回物流中心]
    O --> P[任务 COMPLETED]
```

## UGV 分层动态导航

阶段 1 至 5 保持 `SmacPlanner2D + Regulated Pure Pursuit`，阶段 6 才启用论文
算法链。激光跟踪器先在地图坐标系估计障碍物位置、速度、半径和运动区域。长时域风险
点进入全局代价地图后，D* Lite 通过 `g/rhs` 局部一致性只修复发生变化的网格，输出
新的黄色 `/plan`。若某一周期的动态代价暂时导致规划失败，行为树保留上一条仍有效
路径，而不是取消控制器并反复清图。

DWB 在每个控制周期生成加速度受限的差速轨迹。预测碰撞评价器对机器人和障碍物建立
相对运动 `r(t) = r0 + v_rel*t`，求解 `||r(t)|| = R_safe` 的最早根作为预计碰撞
时间。无碰撞候选再按预测净空、路径对齐和目标进度排序。该评价使用完整二维相对速度，
因此前方、侧向和斜后方障碍进入同一套实时路径评价，不执行固定避让动作。详细公式、
论文映射和回归结果见 `paper_based_dynamic_navigation.md`。

## UAV 飞行和安全控制

UAV 使用 PX4 SITL 状态估计、
位置/姿态控制和电机输出，再由 Gazebo Classic Iris 插件执行动力学。
不通过持续修改实体坐标模拟飞行，并复用 `/uav/takeoff`、`/uav/land`、
`/uav/fly_to_pose` 和配送状态机。已知建筑由固定航路图绕开，顶部 3D 雷达、下向
测距和四个斜下传感器共同形成三维安全感知。
当 3D 雷达在目标方向前方 `5 m` 内发现冲突时，控制器锁定一次切向绕行点，越过
障碍后继续原目标；`1.8 m` 安全球负责紧急保护。持续阻塞时再标记当前航路边并
重新运行 Dijkstra；图上没有可用替代路线时，控制器在高度边界和安全半径内选择
局部三维绕行点。最多重规划次数用于防止任务在不可达障碍前无限循环。

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> TAKEOFF: Action accepted
    TAKEOFF --> HOVER: Target altitude reached
    HOVER --> CRUISE: Hover timer complete
    CRUISE --> APPROACH: Final corridor node reached
    APPROACH --> DELIVERING: Position settled
    DELIVERING --> RETURNING: Delivery event published
    RETURNING --> LANDING: Home approach reached
    LANDING --> COMPLETED: LANDED confirmed
    CRUISE --> CRUISE: Safety clear
    CRUISE --> CRUISE: Block edge and replan
    CRUISE --> FAILED: Replan budget exhausted or timeout
    LANDING --> FAILED: Landing timeout
    COMPLETED --> [*]
    FAILED --> [*]
```

## 电量准入与停靠充电

```mermaid
flowchart TD
    A[读取三块 SOC 路线 楼层 逐件载荷] --> B[计算每个 UAV 架次的论文功率积分]
    A --> C[计算 UGV 各道路段驱动能耗<br/>剩余货物加停靠 UAV 质量]
    B --> D[计算架次间所需充电源能量]
    C --> E[加入驱动电池预测裕量和储备]
    D --> E
    E --> F{三块电池均满足任务和储备?}
    F -- 否 --> R[车辆移动前拒绝任务]
    F -- 是 --> G[允许 UGV 前往会合点]
    G --> H[实时积分驱动功率并更新货物质量]
    H --> I[执行 UAV 架次并实时积分飞行功率]
    I --> J{UAV 已重新停靠?}
    J -- 否 --> I
    J -- 是 --> L{还有配送目标?}
    L -- 是 --> K[设置下一架次目标<br/>充电电池经转换效率向 UAV 供能]
    K --> G
    L -- 否 --> M[停止补电 UGV 返回物流中心并结束]
```

UGV 驱动电池和 UAV 充电电池互不混用。驱动功率随速度、加速度、转向、滚阻、
空气阻力以及总质量变化；总质量包含 UGV 本体、尚未送出的货物和已停靠 UAV。
充电电池只在 UAV 成功停靠时供能，并在自身达到 `10%` 储备后停止充电。

## 自动实验数据流

### PX4 统一电量与按需充电扩展

```mermaid
flowchart LR
    A[Gazebo 真实仿真运动 odom] --> B[论文功率积分与载荷修正]
    B --> C[唯一 UAV SOC]
    C --> D[状态话题与预飞行能量服务]
    C --> E[MAVLink 电池桥]
    E --> F[PX4 电池反馈与低电量保护]
    F --> G[确认同步后允许起飞]
    D --> H[道路距离 载荷 电池预算 时间与能耗排序]
    H --> I[设置下一架次充电目标]
    I --> J{到站 SOC 足够?}
    J -- 否 --> K[停靠补电 CHARGING]
    K --> J
    J -- 是 --> G
    G --> L[配送 返航 降落并确认 PX4 上锁]
    L --> M{还有物资?}
    M -- 是 --> I
    M -- 否 --> N[停止补电并结束批次]
```

`input_order`、`distance`、`energy_time` 与 `full`、`as_needed` 是显式策略参数。
能量目标函数使用三电池净存储能量减少量，充入 UAV 的能量不重复记为系统损耗。
7 至 10 件使用有限 2-opt，详细预测假设与限制见
[PX4 能量调度](px4_energy_scheduling.md)。

```mermaid
flowchart LR
    A[实验矩阵<br/>模式 密度 种子] --> B[独立启动一次仿真]
    B --> C[发送真实配送 Action]
    C --> D[订阅 odom 电池 安全 状态]
    D --> E[计算路径 误差 时间 能耗]
    E --> F[runs.json 与 runs.csv]
    F --> G[均值 标准差 成功率]
    G --> H[summary.md 与 PNG 图表]
    H --> I{还有实验组合?}
    I -- 是 --> B
    I -- 否 --> J[论文结果表和分析]
```

动态障碍的最小净空和安全悬停只在巡航、返航和目标接近阶段统计，不把正常起飞或
降落时接近地面计入避障指标。详细变量与公式见 `evaluation_method.md`。

## 当前设计限制

- 校园 `map` 坐标对齐使用 Gazebo ground truth；PX4 后端内部会融合模拟 IMU/GPS，
  但尚未用该估计独立替代 map 对齐，也没有实现 UAV SLAM。
- UAV 支持航路边重规划和局部三维绕行，但没有在线构建三维全局占据地图。
- UAV 在静止 UGV 上起降，不追踪移动平台。
- 配送使用 ROS 事件模拟卸货，没有实体机械抓取机构。
- 电量是论文模型仿真值，不是实际电池或 ESC 遥测值。

这些限制不影响当前电脑端固定会合点校园配送闭环，但必须在论文中明确说明。
