# 基于 D* Lite 与预测式 DWA 的动态导航设计

## 1. 理论来源

阶段六的 UGV 动态导航基于用户提供的两篇论文：

1. Sven Koenig、Maxim Likhachev，*D* Lite*，AAAI 2002。
2. Marcell Missura、Maren Bennewitz，*Predictive Collision Avoidance for the
   Dynamic Window Approach*，ICRA 2019。

D* Lite 用于全局栅格路径的增量修复；预测式 Dynamic Window Approach（DWA）
用于局部速度选择。两者职责不同，不能只用一个算法代替另一层。

阶段 1 至 5 的静态 UGV 导航仍使用经过验证的
`SmacPlanner2D + Regulated Pure Pursuit`。本文件所述 UGV 算法只在阶段 6
`dynamic_obstacles:=true` 时启用。UAV 保留独立的三维雷达航路规划器，不把二维
DWA 错套到四旋翼飞行控制上。

## 2. D* Lite 增量全局规划

对栅格状态 `s`，D* Lite 同时保存从目标反向计算的 `g(s)` 和一步前瞻值：

```text
rhs(s) = min[c(s,s') + g(s')]
          s' in Succ(s)
```

当 `g(s) = rhs(s)` 时状态局部一致；不一致状态进入按二元键排序的优先队列：

```text
k1 = min(g(s), rhs(s)) + h(s_start, s) + km
k2 = min(g(s), rhs(s))
key(s) = [k1, k2]
```

`km` 补偿机器人起点移动。全局代价地图变化后，规划器只更新变化网格及其前驱，
随后恢复局部一致性，不从空队列重新运行一次完整 A*。从当前位置输出路径时，每步
选择使 `c(s,s') + g(s')` 最小的后继。

本项目的栅格实现支持八邻域，斜向移动使用对角代价，并禁止穿过两个致命网格之间的
对角角点。道路禁行掩膜和 Nav2 膨胀代价仍作为 `c(s,s')` 的输入，因此增量规划不会
绕过原有道路约束。

## 3. 预测碰撞局部评价

激光跟踪器发布障碍物地图位置 `p_o`、速度 `v_o` 和半径。对 DWB 生成的一条加速度
受限、非完整约束候选轨迹，将相邻轨迹点视为分段匀速运动。在每一段内，以机器人
位置和速度 `p_r, v_r` 建立相对运动：

```text
r(t) = (p_o - p_r) + (v_o - v_r) * t
R = r_robot + r_obstacle + safety_margin
```

首次碰撞时间由下式的最早有效根得到：

```text
||r(t)||^2 = R^2
a = dot(v_rel, v_rel)
b = 2 * dot(r0, v_rel)
c = dot(r0, r0) - R^2
a*t^2 + b*t + c = 0
```

若根位于当前轨迹段和预测时域内，该轨迹得到高碰撞代价；预计碰撞越早，代价越高。
无碰撞轨迹继续比较最小预测净空、路径对齐和目标进度。如果所有候选都会碰撞，控制器
优先选择预计碰撞时间最大的候选，为下一控制周期和全局重规划争取时间。

该计算使用二维向量，没有“只看车头”的假设，因此同一公式可以处理前方迎面、侧向
横穿和斜后方追赶。前向 collision monitor 仅保留为真实激光近场紧急保护层，不承担
正常动态路径规划。

## 4. 速度相关的全局风险距离

局部 DWB 只预测数秒。为了让差速车在高速障碍靠近前就得到新的黄色全局路径，地图层
跟踪器还计算更长时域的最近会遇点。安全距离采用：

```text
D_lateral = r_robot + r_obstacle + safety_margin
T_maneuver = 2 * sqrt(D_lateral / a_lateral)
D_safe = D_lateral
         + v_closing * (T_response + T_maneuver)
         + v_robot^2 / (2 * a_brake)
```

已确认的风险点固定在地图坐标系，并锁定绕行侧，直到风险连续清除后才删除。这样代价
不会跟随机器人向前跳动，也不会因跟踪 ID 短暂变化而导致黄色路线左右摆动。风险点只
写入全局代价地图；局部代价地图只接收实时激光，离开的障碍可以立即清除。

## 5. ROS 2 实现

主要源文件如下：

- `simulation_ws/src/ugvcar_navigation2/include/ugvcar_navigation2/dstar_lite.hpp`
- `simulation_ws/src/ugvcar_navigation2/src/dstar_lite.cpp`
- `simulation_ws/src/ugvcar_navigation2/src/dstar_lite_planner.cpp`
- `simulation_ws/src/ugvcar_navigation2/src/predictive_collision_model.cpp`
- `simulation_ws/src/ugvcar_navigation2/src/predictive_collision_critic.cpp`
- `simulation_ws/src/ugvcar_navigation2/src/path_heading_critic.cpp`
- `simulation_ws/src/ugvcar_navigation2/scripts/dynamic_obstacle_predictor.py`
- `simulation_ws/src/ugvcar_navigation2_interfaces/msg/DynamicObstacle.msg`
- `simulation_ws/src/ugvcar_navigation2_interfaces/msg/DynamicObstacleArray.msg`

运行时数据流：

```text
/scan + UGV odom/map pose
    -> 360 degree clustering and velocity tracking
    -> /ugv/tracked_dynamic_obstacles
       + /scan_dynamic_predictions
    -> global costmap -> D* Lite -> /plan
    -> DWB candidate trajectories
    -> PredictiveCollisionCritic -> /cmd_vel
    -> collision monitor -> /cmd_vel_safe
```

关键状态话题：

```bash
ros2 topic echo /ugv/dynamic_replanning/status
ros2 topic echo /ugv/tracked_dynamic_obstacles
ros2 topic echo /ugv/dstar_lite/status
ros2 topic echo /ugv/predictive_dwa/status
ros2 topic echo /plan
```

`PathHeadingCritic` 只在 UGV 基本静止且新路径起始方向与车头相差超过约 40 度时
生效，先选择正确方向原地对齐，再恢复正常路径评价。它解决建筑停靠后返程路径位于
车身后方时 DWB 反复选择零速度的问题；正常直行时该评价器返回零，不引入 S 形摆动。

## 6. 与论文原型的差异

- 原预测式 DWA 使用移动多边形轮廓；本项目将激光点簇拟合为运动圆，降低 VirtualBox
  中每周期的几何计算量。安全半径显式包含机器人、障碍物和安全余量。
- 论文实验使用 `7 x 7` 控制采样和约 `0.3 s` 预测时域。本项目差速车使用
  `7 x 11` 线速度/角速度采样和 `4 s` 局部时域，以覆盖校园场景中更早的会遇；这是
  面向当前速度、传感器频率和计算平台的工程参数，不应表述为论文原值。
- 短时障碍运动使用常速度假设。全局 `15 s` 风险预测用于提前触发规划，但局部控制器
  每 `0.1 s` 用新观测重新评价，不假定障碍物会长期严格匀速。
- D* Lite 工作在 Nav2 全局代价地图，而不是论文中的抽象二值栅格；网格代价、道路
  禁行掩膜和动态风险共同决定边代价。

## 7. 已完成验证

单元和配置测试覆盖 D* Lite 初始规划、移动起点、增删障碍增量修复、不可达状态、
斜向角点约束、相对运动碰撞根、前方/横穿/斜后方轨迹、路径起始方向修正以及动态
预测配置。当前全工作空间测试汇总为
`352 tests, 0 failures, 0 errors, 16 skipped`；跳过项为静态分析器不适用的条目。

定向 Gazebo 回归中，`0.70 m/s` 动态障碍从斜后方追赶 `0.40 m/s` UGV，随后反向
形成迎面会遇。导航成功且物理接触次数为 0，最小中心距 `1.523 m`，最小表面净距
`0.953 m`，移动段平均速度 `0.323 m/s`，横向绕行方向反转次数为 0。餐厅到物流
中心约 `60.8 m` 的返程回归也成功，Nav2 恢复次数为 0，终点误差约 `0.25 m`。

最终食堂二层完整协同回归还覆盖了 UGV 去程、UAV 投递与重新停靠、UGV 返程：
Action 状态为 `SUCCEEDED`，UGV 总路径 `122.44 m`，墙钟时间 `998.0 s`，平均速度
`0.329 m/s`，动态障碍物理接触为 0，最小表面净距 `0.817 m`。D* Lite 共发布
28 次规划状态，其中 16 次复用已有搜索状态。为处理动态会车后的道路边缘恢复，适配器
允许在 `0.6 m` 内选择最近可行起点；阶段六掩码同时提供 `0.30 m` 软恢复带，恢复带
外仍保持致命禁行。

正式保存的定向障碍配置为
`campus_dynamic_obstacles/config/rear_diagonal_regression.yaml`。从
`simulation_ws` 启动该场景：

```bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  gui:=true rviz:=false enable_energy_constraints:=false \
  enable_dynamic_obstacles:=true obstacle_density:=rear_catch \
  obstacle_config_file:=$PWD/src/campus_dynamic_obstacles/config/rear_diagonal_regression.yaml
```

## 8. 剩余实验与适用边界

当前结果证明了实现链路和定向冲突工况，但不能代替统计实验。论文最终结果仍需运行
`none/low/medium/high` 四密度、三个随机种子及 UGV/UAV/协同三模式矩阵，并报告
成功率、碰撞率、最小净空、时间、能耗和重规划开销。

常速度预测对突然急转或瞬时加速的障碍存在误差；二维 UGV 规划不处理可跨越障碍；
激光遮挡后只能在跟踪超时内外推。论文中应将这些写为模型假设和后续改进，而不是声称
系统已经覆盖任意动态环境。
