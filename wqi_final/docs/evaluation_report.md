# v1.0.0 基线验证与实验记录

## 报告范围

本报告记录 `wqi_final` 在 `v1.0.0` 源代码基线建立时能够复现的构建、自动测试、
模型检查和开发阶段运行证据。它不是最终论文中的统计实验结果。尚未执行的三次重复
实验、成功率、均值和标准差均明确标为待采集，不使用推测值代替实测值。

验证日期为 2026-07-20，环境为 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic
和 VirtualBox。系统仅面向电脑端仿真，不包含开发板、PX4 或真实电池遥测。

## 源代码基线

清理教学遗留内容后，工作空间保留 13 个与毕设直接相关的软件包：

| 子系统 | 软件包 |
|---|---|
| UGV | `ugvcar_description`、`ugvcar_navigation2`、`ugvcar_application` |
| UAV | `uav_interfaces`、`uav_description`、`uav_control`、`uav_navigation`、`uav_application`、`uav_bringup` |
| 协同 | `cooperative_delivery_interfaces`、`cooperative_delivery` |
| 操作界面 | `simulation_ui` |
| 第三方动力学 | `vendor/sjtu_drone_description` |

已删除巡逻教学包、示例 Action 客户端、未使用的自定义 Nav2 插件、第一版教学 URDF
以及对应演示启动文件。第三方 SJTU 四旋翼插件保留其 GPL-3.0 许可证和原始目录；
项目默认不对第三方源码执行本项目风格检查，但可通过 CMake 选项单独启用。

## 构建验证

为排除旧 `build/` 和 `install/` 缓存影响，使用全新的临时目录执行：

```bash
cd ~/design_final/wqi_final/simulation_ws
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/wqi_final_v1c_log build \
  --build-base /tmp/wqi_final_v1c_build \
  --install-base /tmp/wqi_final_v1c_install \
  --executor sequential
```

结果：`13 packages finished`，退出码为 0，总用时约 2 分 12 秒。

## 自动测试

在同一套干净构建产物上执行全工作空间测试：

```bash
source /opt/ros/humble/setup.bash
source /tmp/wqi_final_v1c_install/setup.bash
colcon test \
  --build-base /tmp/wqi_final_v1c_build \
  --install-base /tmp/wqi_final_v1c_install \
  --executor sequential
colcon test-result \
  --test-result-base /tmp/wqi_final_v1c_build --verbose
```

结果如下：

| 指标 | 结果 |
|---|---:|
| 测试记录 | 143 |
| 错误 | 0 |
| 失败 | 0 |
| 跳过 | 0 |

测试期间出现的 `SelectableGroups` 提示来自 Python 测试依赖的弃用警告，不影响
退出码和测试结果。

## 模型与地图检查

1. UGV Xacro 展开后通过 `check_urdf`，根链路为 `base_footprint`。
2. UAV Xacro 展开后通过 `check_urdf`，模型包含 3D 雷达、下视相机、下向测距、
   四个斜下测距传感器和 IMU 链路。
3. 校园生成器在隔离临时目录成功重建场景：11 栋建筑、4 组闭合道路、0 棵树，
   地图尺寸为 1800 x 1600 像素，同时生成占据地图和禁行掩膜。

## 已有运行证据

开发阶段的教学楼协同无界面运行已经验证以下链路：

1. UAV 在物流中心以固定关节停靠于 UGV。
2. UGV 通过 Nav2 到达教学楼门口 `(-40.0, 9.5)`，实测位置约为
   `(-39.874, 9.337)`，水平误差约 `0.21 m`。
3. UGV 稳定后才解锁 UAV；UAV 完成物理起飞、悬停、三层 `8.0 m` 高度配送、
   返回、降落并在约 `z=0.421 m` 重新对接。
4. 运行日志记录了 `UAV detached`、`Quadrotor takes off`、`Quadrotor lands`、
   `Landed` 和再次 `UAV docked`，说明飞行不是持续修改实体坐标的瞬移流程。

该次运行暴露的 UGV 返回墙钟超时已改为仿真时间超时，并增加独立无进展看门狗。
上述证据能够证明单次交接和 UAV 子任务闭环，但不能替代最终要求的连续三次完整
往返统计。

## 五阶段完成状态

| 阶段 | 当前状态 | 基线证据 | 仍需正式采集 |
|---|---|---|---|
| 1. 房间 UGV | 功能已实现 | 模型、Nav2 启动和手动标点流程已保留 | 三次路线、耗时、误差 |
| 2. 校园 UGV | 功能已实现 | 校园地图、配送点、多目标路线和 Nav2 测试通过 | 代表路线三次完整记录 |
| 3. 校园 UAV | 功能已实现 | 动力学、传感器、航路、状态机和测试通过 | 各目标三次成功率和端点误差 |
| 4. 空地协同 | 闭环已实现 | 教学楼交接、飞行、降落、重新对接已有运行证据 | UGV 返航在内的连续三次整程 |
| 5. 电量约束协同 | 模型已实现 | 功率积分、载荷、任务准入和停靠充电测试通过 | 不同载荷/SOC 的实测对照数据 |

## 正式实验记录模板

后续每个场景至少运行三次，并在论文中保留原始记录。没有实际运行的数据不得填入
结果列。

| 场景 | 次数 | 成功 | 总时间 (s) | UGV 路径 (m) | UAV 路径 (m) | 端点误差 (m) | 初始/结束 SOC | 失败原因 |
|---|---:|---|---:|---:|---:|---:|---|---|
| 房间 UGV | 1-3 | 待测 | 待测 | 待测 | 不适用 | 待测 | 不适用 | 待测 |
| 校园 UGV | 1-3 | 待测 | 待测 | 待测 | 不适用 | 待测 | 不适用 | 待测 |
| 校园 UAV | 1-3 | 待测 | 待测 | 不适用 | 待测 | 待测 | 待测 | 待测 |
| 教学楼协同 | 1-3 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| 多目标电量协同 | 1-3 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |

完整场景命令见根目录 `README.md`，统计指标、对照组和验收条件见
[`next_stage_task_spec.md`](next_stage_task_spec.md)。

## 结论与剩余风险

`v1.0.0` 已满足源代码基线的编译、接口生成、单元测试、lint、模型解析和地图生成
要求，可以作为后续论文实验的固定版本。毕业设计的软件主体已经形成，但最终论文
证据仍缺少统一采集的多轮整程数据、成功率/标准差图表、能耗预测误差标定以及加载
与空载对照。UAV 仍使用 Gazebo ground truth 定位和固定航路，未知障碍只触发安全
悬停/中止，不包含在线三维全局重规划；这些边界必须在论文中如实说明。
