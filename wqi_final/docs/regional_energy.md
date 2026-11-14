# 四车九电池能量系统

## 1. 范围与入口

用于当前中央物流总站＋A/B/C 三区校园。总站 UGV-00 和区域 UGV-01/02/03
分别拥有独立驱动电池与 UAV 充电电池，加上 UAV 飞行电池，共九块电池。
不改变地图、机器人几何、Nav2 速度/控制参数或 PX4 控制；默认关闭电量约束。
不能把旧单车/双车入口的电池话题或实验结果当成本系统验收结果。

所有电池由一个 `regional_energy_manager` 使用仿真时间结算，避免各节点异步充放电
产生无来源充电。关闭旧 UAV 电池积分节点，只保留原 PX4 电量桥；PX4 收到的 SOC
与任务调度器、RViz 使用同一模型。它是建模仿真，不是实测电池或电机电流。

| 电池 | 容量 | 默认初始电量 | 安全储备 | BatteryState 话题 |
|---|---:|---:|---:|---|
| UAV | 100 Wh | 80% | 20% | `/uav/battery_state` |
| 每辆车驱动电池 | 300 Wh | 各 80% | 20% | `/<车名>/drive_battery_state` |
| 每辆车 UAV 充电电池 | 250 Wh | 各 80% | 10% | `/<车名>/charging_battery_state` |

车名为 `ugv_00`、`ugv_01`、`ugv_02`、`ugv_03`。驱动电池与充电电池不互相补电。
纯 UGV 对照不启动 UAV/PX4，仅结算八块真实存在的地面电池，不发布虚拟 UAV 电池。

## 2. 三终端运行

终端一，关闭已有仿真，再编译并启动：

```bash
cd ~/design_final/wqi_final/simulation_ws
bash ./build_workspace.sh --executor sequential
source ./setup_workspace.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch campus_regional_delivery regional_campus.launch.py \
  enable_energy_constraints:=true gui:=true rviz:=true
```

四车导航依次激活。RViz 启用 `Regional Battery States`，在各车及 UAV 附近显示电量。
自动化回归可以使用 `gui:=false rviz:=false`，不关闭物理仿真和传感器。

终端二，检查 `ready: true`、九块电池及 PX4 电量桥同步：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /regional_delivery/energy_status std_msgs/msg/String --once \
  --qos-durability transient_local
ros2 topic echo /uav/px4/battery_bridge_status std_msgs/msg/String --once \
  --qos-durability transient_local
ros2 topic echo /regional_delivery/energy_readiness std_msgs/msg/String --once \
  --qos-durability transient_local
```

桥状态应包含 `SYNCED:paper_energy_model`，任务端能量就绪状态应为 `READY`。
能量节点先于任务管理器启动，两者各自完成订阅；只看到积分器 ready 不代表任务端
已收到最新电量。未同步时不要起飞，不能关闭保护绕过。
启动地图本身不会下单；等待 Action 服务和新鲜定位后，在终端三发送订单：

```bash
cd ~/design_final/wqi_final/simulation_ws
source ./setup_workspace.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 action send_goal /regional_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['innovation_center'], target_floors: [3], payload_masses_kg: [0.30], return_home: true}" \
  --feedback
```

省略 `target_floors` 或使用空数组表示地面卸货；显式填写楼层时须为正整数，不能
把 `0` 放进显式楼层数组。每件载荷须与楼栋、楼层
数组一一对应，单次区域运输总载荷不超过 1 kg，管理器沿用原分批规则。
按原顺序分区分批，不声称本次新增了全局最优任务分配算法。

切换纯 UGV 能量对照时先关闭上述仿真，改用：

```bash
ros2 launch campus_regional_delivery regional_campus.launch.py \
  delivery_mode:=ugv_only enable_energy_constraints:=true gui:=true rviz:=true
```

发送相同接口、不填写楼层数组的任务。两模式电池总量不同，应比较实际积分能耗，
不能直接比较剩余电量百分比或把初始可用容量差误称为节能效果。

## 3. 配置和低电量测试

新增统一配置：
`simulation_ws/src/campus_regional_delivery/config/regional_energy.yaml`。
`uav_initial_soc`、`initial_drive_soc`、`initial_charging_soc` 使用 0～1 比例，
可分别配置四车八块电池，不是给所有车共用一个剩余能量。
修改初始电量后须重启场景；不提供重置 SOC 的运行时服务。

充电策略 `charging_policy` 支持 `as_needed` 与 `full`。空闲充电目标由
`idle_charge_target_soc` 设置，默认 80%；架次前按任务需求切换目标。
自定义配置可通过以下参数加载，避免修改已调好的其他模型：

```bash
ros2 launch campus_regional_delivery regional_campus.launch.py \
  enable_energy_constraints:=true energy_file:=/绝对路径/regional_energy.yaml \
  gui:=true rviz:=true
```

建议测试：某车驱动初始 SOC=0.10，应拒绝整单；UAV SOC=0.25 且总站充电电池 SOC=0.10，
应因无法补足安全架次拒绝整单；UAV 低电且总站供电充足时允许先真实充电再起飞。
低 UAV 电量不一定拒绝，因为停靠补电本身就是系统功能。
读取 `/regional_delivery/task_admission` 查看拒绝原因，读取
`/regional_delivery/energy_plan` 查看九电池预测、各段能耗与充电等待。
这两者都是保留的 `std_msgs/msg/String` JSON 话题，可用上面的 echo 方式检查。

## 4. 模型与结算

UAV 沿用已有论文启发的旋翼功率模型：水平/悬停、垂直运动、加速度和载荷修正。
配置仍在 `uav_control/config/battery_model.yaml`，详细公式见
[`uav_battery.md`](uav_battery.md)。实时速度、加速度来自现有 odom。

UGV 沿用 `cooperative_delivery/config/ugv_energy_model.yaml`：

```text
m_total = m_ugv + m_cargo + I_docked * (m_uav + m_uav_payload)
P_drive = P_idle + k_v*|v| + k_w*|w|
          + (C_rr*m_total*g*|v| + 0.5*rho*C_DA*|v|^3
             + m_total*max(a_parallel,0)*|v|) / eta_drive
E_drive_next = max(0, E_drive - P_drive*dt/3600)
```

模型中货物只能位于某一辆车、UAV 或已送达集合；总站装货、空中运输、区域交接、
楼层投递分别更新所有权。携带载货 UAV 时，车的总质量包含无人机和机上货物，
但货物不会再重复放入该车货物清单。货物交接和卸货是逻辑事件，未给 Gazebo
创建逐件包裹或改变 PX4 机体惯量，不能写成已经验证真实变质量动力学。

实际充电必须同时具备：捕获插件报告锁定、有效承载车名、UAV 状态为 LANDED、
新鲜的两机位置、水平差不超过 0.12 m、中心高度差相对平台偏移 0.42 m 不超过 0.05 m。
仅靠近车或仍在空中不充电；已物理锁定时允许 UGV 载机行驶中供电，不支持行驶中着陆。

```text
E_source_next = E_source - (P_converter_in / eta_transfer + P_converter_idle)*dt/3600
E_uav_next = E_uav + (eta_charge*P_converter_in - P_uav_docked_idle)*dt/3600
```

转换器最大输入 180 W，车辆供电效率 0.90，UAV 充电效率 0.90，UAV 停靠待机 5 W。
最大净入电约 157 W；来源电池到 10% 储备时停止供电，不能凭空维持无人机电量。
接近目标 SOC 时限流，空闲仍需为实际待机功率付能量。
每个仿真步先从来源电池扣除能量，再将实际可供功率交给 UAV；四车同时结算待机/行驶。

`system_net_consumed_wh = sum(所有电池初始 Wh) - sum(所有电池当前 Wh)`。
它包含车辆、电机、设备和充电损失，内部充电转移不重复计入系统净能耗。
单独的 source consumed / UAV charged 是能量流记录，不应直接再次相加作为净消耗。
对照实验应取相同任务窗口的净能耗差值（任务结束累计值减任务开始累计值），而不是
直接比较启动后累计值；否则不同启动/等待时长也会被误计为算法节能。

## 5. 预检与安全边界

整单准入使用现有 Dijkstra 道路图计算地面路程，使用现有飞行能量规划器预测
区域运输与楼层升降。考虑四车等待、服务时间、载荷变化、有限充电源及九电池储备。
`return_home:=false` 也保留安全返航预算，但不强制改变原最终任务停靠位置。
每次地面出发复查目标＋回所属站点电量；每次起飞复查该架次＋空载回总站＋储备。
不足时只能在实际停靠来源上补电，发布 `UAV_CHARGING` 并等待 SOC 实际增长。

运行中若定位/能量状态过期、能量节点重启、车辆到达驱动储备或飞行电量触及储备，
任务停止并沿用原安全回收逻辑；回收成功不把失败订单改成成功。
取消后若仍有未送达货物，拒绝新单，需人工处理/重新准备初始场景，不静默丢弃货物。
能源配置使用哈希一致性校验，避免任务管理器与积分器使用不同容量、效率或初始参数。

预算是静态名义预测，包含裕度，但不是对任意绕障、排队和无限等待的严格保证。
模型参数未经实机标定，不是 PX4 提供的真实电池实验；本次不增加四车动态障碍验收。

## 6. 状态与测试

主状态 `/regional_delivery/energy_status` 包括 `energy_wh`、`cargo_kg`、`payload_kg`、
`charging_carrier`、`charging_input_power_w`、`charge_target_soc`、`system_net_consumed_wh`、
`ready`、`missing_telemetry`、配置哈希和指令确认序号。
能量是 Wh，功率是 W；BatteryState.percentage 为 0～1，`/uav/battery_percentage` 为 0～100。
ROS BatteryState 电流正值表示充电、负值表示放电；转换 MAVLink 时沿用原桥的符号转换。

```bash
ros2 topic echo /ugv_01/drive_battery_state sensor_msgs/msg/BatteryState --once \
  --qos-durability transient_local
ros2 topic echo /ugv_01/charging_battery_state sensor_msgs/msg/BatteryState --once \
  --qos-durability transient_local
ros2 topic echo /regional_delivery/energy_plan std_msgs/msg/String --once \
  --qos-durability transient_local
```

新增测试覆盖九电池独立性、能量收支、实际来源耗尽、四个停靠来源、载荷转移、
全部配送点道路连通、任务预算、低电拒单、充电等待及旧接口兼容。
真实仿真和测试数字以 [`evaluation_report.md`](evaluation_report.md) 的 2026-11-12
记录为准；源码测试通过不代表多区域、多轮动态实验已经完成。

## 7. 文件职责

新增内容均在 `simulation_ws/src/campus_regional_delivery/`：

- `config/regional_energy.yaml`：九电池初始 SOC、充电策略、遥测与预测裕度。
- `campus_regional_delivery/energy_model.py`：统一积分器、有限来源和载荷账本。
- `campus_regional_delivery/energy_plan.py`：复用现有道路/飞行规划器的整单预算适配。
- `campus_regional_delivery/energy_manager.py`：ROS 状态、BatteryState、RViz 标签和指令确认。
- `campus_regional_delivery/energy_mission.py`：任务准入、架次预检与真实充电等待。
- `scripts/regional_energy_manager`：节点入口。
- `test/test_regional_energy_*.py`：能量模型、预算和任务接线回归。

已有 `mission_manager.py`、`fleet.py`、`regional_campus.launch.py` 接入可选开关、
任务载荷事件及显示；`CMakeLists.txt`、`package.xml` 安装节点和声明复用依赖。
共享 `cooperative_delivery/relay_energy.py` 兼容扩展到四个角色并增加等待段，默认
双车调用不变；`uav_control/battery_model.py` 增加可选有限外部输入功率，旧调用
不传此参数时不变；`uav_control.launch.py` 增加关闭原独立积分器的选项，默认仍启用。
