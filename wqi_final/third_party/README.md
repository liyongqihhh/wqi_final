# 固定版本的第三方飞控源码

本目录通过 Git 子模块管理完整 PX4 和通信依赖，不包含编译产物。
PX4 是唯一 UAV 飞控；旧 SJTU 控制插件已删除。Gazebo Classic 仍用于传感器、
电机、旋翼、接触和机体动力学仿真，不能把它与旧替代飞控一起删除。

| 目录 | 上游 | 固定基线 |
| --- | --- | --- |
| `PX4-Autopilot` | https://github.com/PX4/PX4-Autopilot | v1.16.0 |
| `px4_msgs` | https://github.com/PX4/px4_msgs | v1.16.0 |
| `Micro-XRCE-DDS-Agent` | https://github.com/eProsima/Micro-XRCE-DDS-Agent | v2.4.2 |

每个子模块及其嵌套依赖的精确提交由 Git gitlink 固定；不跟随上游最新分支。
原始许可证、作者和版权信息保留在各源码目录中。

当前固定提交为 PX4 `6ea3539157ca358c70a515878b77077af7d4611d`、
`px4_msgs` `392e831c1f659429ca83902e66820d7094591410`、Agent
`57d086216d01ec43121845d385894a25987f8a2c`。消息仓库的 v1.16.0、v1.16.1、v1.16.2
标签指向同一提交，因此 `git describe` 可能显示 v1.16.2；以固定提交为准。

## 克隆和复现

```bash
git clone --recurse-submodules https://github.com/liyongqihhh/wqi_final.git
cd wqi_final/wqi_final/simulation_ws
bash ./setup_px4_backend.sh --install-system-deps
bash ./build_workspace.sh
source ./setup_workspace.bash
ros2 launch uav_bringup uav_sim.launch.py gui:=true rviz:=false
```

普通 clone 之后，在仓库根目录执行 `git submodule update --init --recursive`。
不要使用 GitHub 的源码 ZIP 来代替递归克隆，因为 ZIP 不包含子模块源码。

安装脚本从这些固定源码生成外部构建副本，应用项目内
`simulation_ws/src/uav_px4_bringup/patches/` 的兼容补丁，完成 PX4 和 ROS 2 依赖编译。
副本、构建产物、虚拟环境、ULog 和 ROS 日志仍位于 `WQI_BUILD_ROOT`，默认是仓库
同级的 `*_artifacts` 目录。它们不是另一份独立维护的飞控源码。
已有外部副本版本不一致时脚本会拒绝覆盖，应选择新的构建目录并保留原修改。
