# 校园 UGV-UAV 协同配送系统架构

## 系统边界

本项目的最终交付范围是 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 和
RViz 上的电脑端物理仿真。系统不依赖开发板、PX4、真实电池或真实电流传感器。

```mermaid
flowchart LR
    UI[simulation_ui<br/>任务与仿真控制] --> CM[cooperative_delivery<br/>协同任务管理器]
    UI --> UA[uav_application<br/>独立 UAV 任务管理器]
    UI --> GA[ugvcar_application<br/>独立 UGV 任务管理器]

    CM --> NAV[Nav2<br/>UGV 全局与局部导航]
    CM --> UA
    CM --> DOCK[Gazebo 固定关节<br/>对接与解锁]

    NAV --> UGV[UGV ros2_control<br/>差速驱动与传感器]
    UA --> FC[uav_control<br/>飞行 Action 与安全控制]
    FC --> UAV[SJTU Gazebo 动力学<br/>四旋翼实体]

    BM[uav_control/battery_manager<br/>耗电 准入 充电] --> UA
    BM --> CM
    DOCK --> BM

    WORLD[campus_delivery.world<br/>建筑 道路 碰撞环境] --> UGV
    WORLD --> UAV
    UGV --> VIZ[RViz 与状态话题]
    UAV --> VIZ
    CM --> VIZ
```

## 软件包职责

| 软件包 | 职责 |
|---|---|
| `ugvcar_description` | UGV 模型、传感器、房间/校园 world、地图生成 |
| `ugvcar_navigation2` | Nav2 地图、代价地图、行为树和启动文件 |
| `ugvcar_application` | UGV 多目标顺序优化与配送执行 |
| `uav_interfaces` | UAV Action 和能量检查 Service |
| `uav_description` | UAV Xacro、碰撞体、3D 雷达和下视传感器 |
| `uav_control` | 起降、定点飞行、安全球、电池与充电模型 |
| `uav_navigation` | 配送楼层点、空中航路图和路径可视化 |
| `uav_application` | UAV 配送状态机和能量准入 |
| `uav_bringup` | 独立 UAV 仿真组合启动 |
| `cooperative_delivery_interfaces` | 空地协同 Action |
| `cooperative_delivery` | UGV-UAV 调度、对接、联合启动和能量序列规划 |
| `simulation_ui` | 五种实验模式、货物配置和进程管理 |
| `vendor/sjtu_drone_description` | 第三方 Gazebo 四旋翼力/力矩动力学插件 |

## 协同配送主流程

UGV 在固定会合点停稳后 UAV 才能解锁。UAV 返回并重新对接后，UGV 才能继续
下一个地面目标或返回物流中心。

```mermaid
flowchart TD
    A[接收目标 楼层 载荷 初始电量] --> B[校验参数并优化 UGV 访问顺序]
    B --> C{完整任务能量可行?}
    C -- 否 --> R[拒绝任务<br/>车辆保持安全状态]
    C -- 是 --> D[确认 UAV 已停靠]
    D --> E[UGV Nav2 前往建筑门口停靠点]
    E --> F{UGV 到点并稳定?}
    F -- 否 --> X[清理代价地图并有限重试]
    X --> E
    F -- 是 --> G[起飞前重新检查 UAV 电量]
    G --> H[解锁 UAV 并起飞到目标楼层高度]
    H --> I[飞往楼栋正门配送点]
    I --> J[悬停并发布 DELIVERED 事件]
    J --> K[返回 UGV 上方并下降]
    K --> L{物理降落和对接成功?}
    L -- 否 --> Q[任务失败并记录原因]
    L -- 是 --> M[恢复充电并刷新 UGV 代价地图]
    M --> N{还有目标?}
    N -- 是 --> E
    N -- 否 --> O[UGV 返回物流中心]
    O --> P[任务 COMPLETED]
```

## UAV 飞行和安全控制

UAV 使用 Gazebo 力/力矩动力学，不通过持续修改实体坐标模拟飞行。已知建筑由
固定航路图绕开，顶部 3D 雷达、下向测距和四个斜下传感器共同形成三维安全球。

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
    CRUISE --> FAILED: Persistent obstacle or timeout
    LANDING --> FAILED: Landing timeout
    COMPLETED --> [*]
    FAILED --> [*]
```

## 电量准入与停靠充电

```mermaid
flowchart TD
    A[读取当前 SOC 载荷 航路和楼层] --> B[积分起飞 巡航 悬停 下降功率]
    B --> C[加入预测裕量和安全储备]
    C --> D{剩余能量大于任务能量加储备?}
    D -- 否 --> E[拒绝 UAV 出动<br/>保持停靠充电]
    D -- 是 --> F[允许起飞并实时积分耗电]
    F --> G{UAV 已重新停靠?}
    G -- 否 --> F
    G -- 是 --> H[按充电功率和效率恢复能量]
    H --> A
```

## 当前设计限制

- UAV 定位使用 Gazebo ground truth，没有实现 SLAM 或 GPS/IMU 融合。
- 未知空中障碍会触发悬停、恢复或中止，不执行在线三维全局重规划。
- UAV 在静止 UGV 上起降，不追踪移动平台。
- 配送使用 ROS 事件模拟卸货，没有实体机械抓取机构。
- 电量是论文模型仿真值，不是实际电池或 ESC 遥测值。

这些限制不影响当前电脑端固定会合点校园配送闭环，但必须在论文中明确说明。
