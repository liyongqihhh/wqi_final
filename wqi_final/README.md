# wqi_final

ROS 2 + Gazebo/RViz simulation project for a graduation design about campus
logistics with independently validated UGV and UAV subsystems and a combined
UGV-UAV delivery workflow.

Release `v1.0.0` is the PC-simulation baseline. It contains only the packages
used by the five graduation-design stages; tutorial patrol, example action
clients, and unused custom Nav2 plugin packages have been removed.

The current codebase keeps the simulation and navigation parts that are useful for the final project, and removes chapter-style tutorial examples such as hello world, basic topic/service demos, rosbag exercises, and hardware-board bringup code.

## Project Layout

```text
wqi_final/
└── simulation_ws/
    └── src/
        ├── ugvcar_description       # UGV model, Gazebo worlds, campus map generator, ros2_control config
        ├── ugvcar_navigation2       # Nav2 maps, parameters, launch files
        ├── ugvcar_application       # Optimized campus delivery manager
        ├── uav_interfaces            # FlyToPose and ExecuteDelivery actions
        ├── uav_description           # Quadrotor, 3D lidar, down sensors, IMU, TF, RViz config
        ├── uav_control               # Flight actions, sensor fusion, and 3D safety envelope
        ├── uav_navigation            # UAV pads, 15 m air corridors, RViz markers
        ├── uav_application           # Autonomous delivery mission state machine
        ├── uav_bringup               # Standalone campus UAV launch
        ├── cooperative_delivery_interfaces # Combined mission action
        ├── cooperative_delivery      # UGV-UAV manager, docking plugin, joint launch
        ├── simulation_ui             # Five-mode PyQt5 simulation control panel
        └── vendor/sjtu_drone_description # GPL-3.0 Gazebo force/torque dynamics plugin
```

## Design Documents

- [`docs/system_architecture.md`](docs/system_architecture.md): package
  boundaries, deployment architecture, cooperative state flow, and battery
  decision flow.
- [`docs/evaluation_report.md`](docs/evaluation_report.md): reproducible build,
  test, and simulation evidence, with formal experiments clearly marked.
- [`docs/cooperative_delivery.md`](docs/cooperative_delivery.md): docking and
  cooperative mission implementation.
- [`docs/uav_subsystem.md`](docs/uav_subsystem.md): UAV dynamics, sensors,
  frames, safety control, and limitations.
- [`docs/uav_battery.md`](docs/uav_battery.md): propulsion, payload, battery,
  preflight admission, and charging equations.

## Current Capability

- Simulate a UGVcar-style ground vehicle in Gazebo.
- Run Nav2 localization, planning, and navigation in RViz.
- Send single goals or waypoint goals through ROS 2 nodes.
- Provide a campus logistics delivery scene with 11 major buildings, a connected circulation network without duplicate or dead-end roads, an occupancy map, and a keepout mask.
- Run a standalone UAV mission from the logistics center to a named campus
  delivery pad, including physical takeoff, hover, 15 m corridor cruise,
  delivery, return, and landing.
- Publish a top-mounted 3D lidar point cloud, a down-view RGB camera, one
  downward range, four diagonal short-range sensors, IMU, and safety status.
- Fuse the lidar and short-range sensors into a 1.8 m 3D safety sphere that
  holds position before collision and resumes after a transient obstacle.
- Use complete body, arm, rotor, and landing-skid collisions with controlled
  XY-held descent instead of cutting thrust above the ground.
- Carry the UAV on the UGV with a Gazebo fixed joint, release it at a named
  launch point, execute an aerial delivery, redock it, and return the UGV home.
- Accept up to ten cargo items with independent destinations, floors, and
  masses, then compute an exact shortest visit order without breaking the
  item-to-payload mapping.
- Simulate distinct UAV takeoff, cruise, hover, landing, and idle power use;
  reject missions that cannot preserve a safe-return reserve, and charge the
  UAV automatically whenever it is docked on the UGV.
- Start the UGV, UAV, Nav2, docking plugin, task managers, and combined RViz
  view from one launch file while creating only one Gazebo server.

## Build

```bash
cd ~/design_final/wqi_final/simulation_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

To build only the standalone UAV subsystem:

```bash
colcon build --packages-select \
  sjtu_drone_description uav_interfaces uav_description uav_control \
  uav_navigation uav_application uav_bringup
source install/setup.bash
```

To build the combined UGV-UAV subsystem and its dependencies:

```bash
colcon build --packages-up-to cooperative_delivery uav_bringup
source install/setup.bash
```

## Desktop Simulation Control Panel

The desktop control panel starts any one of the five graduation-design test
stages and sends its real ROS 2 route command. It configures up to ten cargo
items, with an independent destination, floor, and mass for every item, plus
the initial UAV battery, return behavior, sensor-ray display, and whether RViz,
Gazebo, or both viewers are opened.

Build and start it with:

```bash
cd ~/design_final/wqi_final/simulation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to simulation_ui
source install/setup.bash
ros2 run simulation_ui simulation_dashboard
```

Use the left mode list to select Stage 1 through Stage 5. Set `件数`, complete
one row in the cargo table for every package, then
choose `RViz`, `Gazebo`, or `两者`, then click `启动仿真`. After the simulation
interfaces are ready, click `运行配送任务`. Stage 1 intentionally has no route
button and continues to use an RViz Nav2 goal. `停止任务` sends `SIGINT` to the
ROS 2 Action client, which requests cancellation of an accepted goal without
closing the simulation. `停止全部` closes every process started by the control
panel.

The selectable floor is checked against the actual floor count of each campus
building. The UAV cruises horizontally at `15 m`; near the selected facade it
uses the floor-center altitude `1.6 + (floor - 1) * 3.2 m`. Package mass and
floor are included in both the preflight energy estimate and the executed UAV
or cooperative Action goal. The three Action arrays remain index-aligned while
the execution layer reorders complete cargo records.

Routes with at most ten items are optimized exactly with Held-Karp dynamic
programming. Standalone UAV missions minimize distance on the configured air
corridor graph. UGV-only and cooperative missions minimize the closed tour
through the required ground stop coordinates. Items sharing one UGV stop are
kept together, and the cooperative manager launches the next UAV sortie there
without issuing a redundant UGV navigation goal. The selected route is
published on `/uav/optimized_route` or
`/cooperative_delivery/optimized_route` and is also written to the launch log.

For a normal end-to-end test, set the Stage 5 initial battery to `80%`, click
`启动仿真`, and only then send the task. The battery value is an initial launch
parameter, so changing it after Gazebo is already running does not reset the
live battery; restart the simulation to apply the new value. Values below
`40%` trigger a UI warning, while a value at or below the configured `20%`
safety reserve is intended mainly for rejection and charging tests.

If an Action is accepted but immediately aborts, read the task-status box in
the control panel. For an energy rejection it reports the three decisive
values: predicted energy at takeoff, sortie energy, and the mandatory reserve.
The admission condition is `takeoff energy >= sortie energy + reserve`; the
UGV does not begin moving when the complete mission cannot satisfy this
condition.

## Five-Stage Graduation-Design Test Guide

Run the stages in order. Before entering commands in a new terminal, prepare
the ROS 2 environment with:

```bash
cd ~/design_final/wqi_final/simulation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Gazebo and RViz can be enabled together for final screenshots. In VirtualBox,
use only one 3D viewer at a time if the simulation becomes slow.

| Stage | System under test | Route input |
|---|---|---|
| 1 | UGV in room | Manual RViz goal |
| 2 | UGV on campus | `delivery_task.launch.py` |
| 3 | UAV on campus | `/uav/execute_delivery` Action |
| 4 | UGV-UAV cooperation on campus | Cooperative Action, battery held at full start |
| 5 | Energy-aware UGV-UAV cooperation | Cooperative Action with 30% and 1% battery cases |

### Stage 1: UGV Room Obstacle Avoidance And Navigation

This stage validates the original room world, UGV sensors, localization, Nav2
planning, and local obstacle avoidance. It intentionally uses manual RViz goal
selection, so no route command is required.

Terminal 1, open Gazebo and spawn the UGV in the room:

```bash
ros2 launch ugvcar_description gazebo_sim.launch.py
```

Terminal 2, open Nav2 and RViz with the room map:

```bash
ros2 launch ugvcar_navigation2 navigation2.launch.py
```

In RViz, first use `2D Pose Estimate` when localization needs initialization,
then use `Nav2 Goal` or `2D Goal Pose` to select a free point beyond an
obstacle. The UGV must plan around the obstacle and reach the selected point
without touching a wall.

### Stage 2: UGV Campus Obstacle Avoidance And Navigation

Terminal 1, open the campus world in Gazebo:

```bash
ros2 launch ugvcar_description campus_delivery_sim.launch.py \
  gui:=true visualize_sensor_rays:=false
```

Terminal 2, open Nav2 and RViz with the campus map and keepout mask:

```bash
ros2 launch ugvcar_navigation2 campus_navigation.launch.py \
  rviz:=true localization_mode:=ground_truth
```

On a resource-constrained VirtualBox VM, keep only one 3D viewer active. For
automated delivery while watching RViz, start Gazebo without its GUI:

```bash
ros2 launch ugvcar_description campus_delivery_sim.launch.py gui:=false
```

To watch Gazebo instead, start navigation without RViz:

```bash
ros2 launch ugvcar_navigation2 campus_navigation.launch.py rviz:=false
```

The campus launch defaults to deterministic Gazebo ground-truth localization so
Gazebo and RViz share the same pose. To run an AMCL localization experiment
instead, use:

```bash
ros2 launch ugvcar_navigation2 campus_navigation.launch.py localization_mode:=amcl
```

Terminal 3, run the short standard route after Nav2 is active:

```bash
ros2 launch ugvcar_application delivery_task.launch.py \
  delivery_targets:="['teaching_building']"
```

For an extended route, send three campus destinations. The task manager chooses
the next nearest stop, waits at every destination, and returns to the logistics
center after the final stop:

```bash
ros2 launch ugvcar_application delivery_task.launch.py \
  delivery_targets:="['teaching_building','laboratory','dormitory_2']" \
  wait_duration:=10.0
```

Available destinations are `cafeteria`, `teaching_building`,
`innovation_center`, `library`, `laboratory`, `gymnasium`, `dormitory_1`,
`dormitory_2`, `dormitory_3`, and `dormitory_4`. All four dormitory names use
the central road intersection at `(30.0, 11.0)` as their shared stop. Both
local and global costmaps use a `1.0 m` inflation radius. The generated keepout
mask also reserves a recoverable `1.0 m` low-cost shoulder along each side of
normal-width roads before the lethal off-road area. The campus roads consist of
a southern logistics/cafeteria service loop, one main logistics-to-dormitory
trunk, a separated eastern connector, and one northern campus loop. Every road
segment belongs to a closed circulation route, and the dormitory stop remains
at the central junction `(30.0, 11.0)`.
The UGV uses the validated stable cruise speed of `0.22 m/s`. The regulated
controller reduces speed automatically near corners and goals. Road keepout is
enforced by the global costmap; the rolling local costmap can continue steering
the UGV back to the planned path after a small boundary error. The test passes
when the UGV stays on the road, avoids occupied cells, reaches every requested
stop, and returns to the logistics center.

### Stage 3: UAV Campus Obstacle Avoidance And Navigation

The UAV launch reuses `campus_delivery.world` but does not spawn or start the
UGV. A lightweight map server publishes the campus occupancy map for RViz;
UGV localization, planning, and controller nodes are not started.

Terminal 1, open the campus world in Gazebo and the UAV view in RViz:

```bash
ros2 launch uav_bringup uav_sim.launch.py \
  gui:=true rviz:=true visualize_sensor_rays:=false \
  initial_battery_percentage:=1.0
```

For better VirtualBox performance, use either headless Gazebo with RViz or
Gazebo without RViz:

```bash
ros2 launch uav_bringup uav_sim.launch.py gui:=false rviz:=true
ros2 launch uav_bringup uav_sim.launch.py gui:=true rviz:=false
```

Terminal 2, run the standard teaching-building route:

```bash
cd ~/design_final/wqi_final/simulation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 action send_goal /uav/execute_delivery \
  uav_interfaces/action/ExecuteDelivery \
  "{targets: ['teaching_building'], return_home: true}" --feedback
```

An extended obstacle-corridor route can be tested with:

```bash
ros2 action send_goal /uav/execute_delivery \
  uav_interfaces/action/ExecuteDelivery \
  "{targets: ['teaching_building','library'], return_home: true, \
  target_floors: [3,5], payload_masses_kg: [0.30,0.25]}" --feedback
```

Available UAV targets are `teaching_building`, `laboratory`, `library`,
`innovation_center`, `cafeteria`, `gymnasium`, `dormitory_1`, `dormitory_2`,
`dormitory_3`, and `dormitory_4`. The mission sequence is takeoff, five-second
hover, climb to the `15 m` cruise altitude, flight along the closed road-center
air-corridor graph, vertical approach, five-second delivery hover, return, and
landing. Because several buildings are taller than 15 m, direct
target-to-target straight lines are not used.

The main UAV interfaces are:

- Actions: `/uav/fly_to_pose`, `/uav/execute_delivery`
- Services: `/uav/takeoff`, `/uav/land`
- State topics: `/uav/odom`, `/uav/imu`, `/uav/flight_state`, `/uav/mission_status`
- Battery topics: `/uav/battery_state`, `/uav/battery_percentage`,
  `/uav/battery_status`, `/uav/battery_power_w`,
  `/uav/battery_consumed_wh`, `/uav/battery_charged_wh`,
  `/uav/propulsion_power_w`, `/uav/auxiliary_power_w`,
  `/uav/payload_mass`, `/uav/remaining_energy`
- Energy check service: `/uav/check_delivery_energy`
- Perception topics: `/uav/lidar/points`, `/uav/down_camera/image_raw`,
  `/uav/range/down`, `/uav/range/front_down`, `/uav/range/rear_down`,
  `/uav/range/left_down`, `/uav/range/right_down`, `/uav/imu`
- Safety topics: `/uav/safety/blocked`, `/uav/safety/status`,
  `/uav/safety/min_distance`, `/uav/safety/ground_clearance`
- Visualization topics: `/uav/planned_path`, `/uav/path`,
  `/uav/delivery_points`, `/uav/safety_sphere`, `/uav/optimized_route`

RViz shows the straight node-to-node plan in cyan (`/uav/planned_path`) and
the physically flown odometry trace in orange (`/uav/path`). The orange trace
can contain small turn arcs because the force/torque model has inertia; it is
not the route used by the planner. The RobotModel uses the `uav` RViz TF
prefix so the model follows the flight trace instead of remaining at the map
origin.

The UAV uses a configurable `100 Wh` simulated battery. Propulsion power is a
nonlinear function of horizontal and vertical velocity, acceleration, turning,
and current package mass; onboard electronics are added separately. Before
accepting a mission, the manager integrates the complete safe-return route,
applies a `25%` prediction margin, and preserves a `20%` battery reserve.
Stage 3 starts at 100% so battery is not an experimental variable; Stage 5
explicitly validates the battery constraint and UGV charging behavior.

The top lidar publishes `sensor_msgs/msg/PointCloud2` with 240 horizontal by 24
vertical rays at 10 Hz. Its configured elevation field is `-45 deg` to
`+89 deg`, so it scans from diagonal-down to almost vertical-up without
increasing the previous 5760-ray load. Point clouds contain obstacle returns,
not artificial points in empty sky; Gazebo shows the configured ray field when
`gui:=true`. RViz also displays the down camera and the green/red safety sphere.
The simulated IMU includes orientation, gyroscope, and accelerometer data.

Check that all perception streams are active with:

```bash
ros2 topic hz /uav/lidar/points
ros2 topic hz /uav/down_camera/image_raw
ros2 topic echo /uav/range/down
ros2 topic echo /uav/safety/status
ros2 topic hz /uav/imu
```

When Gazebo is the active viewer, the 3D lidar and short-range ray patterns are
rendered in blue. Keep only one 3D viewer active in VirtualBox:

```bash
ros2 launch uav_bringup uav_sim.launch.py gui:=true rviz:=false
```

The latest targeted library-corridor regression started from the `north_east`
rendezvous node at `(48, 40)`, completed delivery at the roadway UAV pad at
`(76, 40)`, returned through the final safety corridor, and landed physically.
The action returned `success: true`; final UAV odometry was approximately
`(47.989, 39.992, 0.0)`, with flight state `LANDED` and safety state `CLEAR`. See
[`docs/uav_subsystem.md`](docs/uav_subsystem.md) for architecture, frames,
validation evidence, limitations, and third-party licensing details.

The test passes when the UAV takes off physically, follows the planned
corridors, holds before unsafe 3D sensor clearances, completes every delivery,
returns to the logistics center, and lands with `LANDED` and `CLEAR` states.

### Stage 4: UGV-UAV Cooperative Campus Navigation

The cooperative launch starts one campus Gazebo world, one UGV, one UAV, Nav2,
the UAV control stack, the docking plugin, and the high-level mission manager.
This baseline starts at 100% so the test focuses on navigation, obstacle
avoidance, vehicle transfer, and docking rather than battery admission.

Terminal 1, open the combined simulation in Gazebo and RViz:

```bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  gui:=true rviz:=true visualize_sensor_rays:=false \
  initial_battery_percentage:=1.0
```

For VirtualBox, use `gui:=false rviz:=true` to watch RViz or
`gui:=true rviz:=false` to watch Gazebo.

Terminal 2, run the complete teaching-building cooperative route:

```bash
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['teaching_building'], return_home: true}" --feedback
```

Multiple targets can be handled in one mission:

```bash
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['laboratory','library','dormitory_4'], return_home: true, \
  target_floors: [4,5,11], payload_masses_kg: [0.35,0.20,0.25]}" --feedback
```

The combined state sequence is:

```text
PREPARING -> UGV_TRANSIT -> UGV_SETTLING -> UAV_DETACHING
-> UAV_DELIVERING -> UAV_DOCKING -> RETURNING_HOME -> COMPLETED
```

The public combined interfaces are:

- Action: `/cooperative_delivery/execute_mission`
- Status: `/cooperative_delivery/mission_status`
- Optimized order: `/cooperative_delivery/optimized_route`
- Docking services: `/uav/attach_uav`, `/uav/detach_uav`
- Docking state: `/uav/docked`

The UGV navigation timeout scales with the Nav2 route length to tolerate a
reduced Gazebo real-time factor in VirtualBox. `/odom` is the UGV controller's
local accumulated odometry; inspect `/ground_truth/odom` or the `map` TF frame
when comparing the vehicle with Gazebo.

The teaching-building regression completed UGV transport, UAV takeoff,
delivery, return, platform landing, redocking, and UGV return to the logistics
center. The action returned `success: true`, `completed_targets: 1`; final
ground-truth positions were approximately `(-0.053, -43.346)` for the UGV and
`(-0.053, -43.345, 0.421)` for the docked UAV. See
[`docs/cooperative_delivery.md`](docs/cooperative_delivery.md) for design and
validation details.

Laboratory, library, and dormitory mappings have also been exercised. The final
library safety corridor routes around the building at `x=78`, `y=65`, and
`x=42`; its physical UAV round trip completed with `LANDED` and `CLEAR` states.
The dormitory combined mission completed UGV transport, UAV delivery, return,
landing, and redocking with `/uav/docked: true`. The complete final three-target
sequence remains an experiment to record three times for the thesis results.

The Stage 4 test passes when the UGV reaches the building stop, the UAV
detaches only after the UGV settles, the UAV completes its aerial route and
redocks, and the UGV returns to the logistics center with mission state
`COMPLETED`.

### Stage 5: Battery-Constrained UGV-UAV Cooperative Navigation

The following tests verify dock charging, preflight energy admission, flight
discharge, low-energy rejection, and the complete cooperative route with energy
as an explicit experimental variable.

The battery model is based on the Zeng rotary-wing horizontal power equation,
the Gong multi-rotor vertical ascent/descent equation, and the Dai dynamic
thrust-to-weight correction for acceleration and turns. It uses the simulated
quadrotor's `1.477 kg` unloaded mass, four `0.12 m` rotors, current package
mass, and `25 W` of computer/lidar/camera/communication loads.

Package mass is not a linear multiplier. At each time step:

```text
m = m_airframe + m_sensor + m_payload
P_induced = (1 + k) * (m*g)^(3/2) / sqrt(2*rho*A)
P_battery = (P_horizontal + P_vertical - P_hover + P_auxiliary)
            / discharge_efficiency
```

Every route edge uses a trapezoidal speed profile, or a triangular profile when
the edge is too short to reach maximum speed. The nonlinear power equation is
sampled every `0.1 s`; delivered package mass is removed before planning the
next leg:

```text
E_raw = sum(P_battery(v_j, a_j, payload_j) * dt_j / 3600)
E_predicted = 1.25 * E_raw
E_required = E_predicted + battery_capacity * 0.20
mission accepted only if E_available >= E_required
```

Before the UGV moves, the cooperative planner checks every UAV sortie in the
requested sequence. It subtracts predicted sortie energy and conservatively
adds only the minimum dock charging guaranteed by UGV travel:

```text
E_takeoff[j] = min(capacity, E_landing[j-1] + E_charge_min[j])
E_takeoff[j] >= E_sortie[j] + E_reserve
```

The complete derivation, aircraft calibration, and timing equations are in
[`docs/uav_battery.md`](docs/uav_battery.md).

First build and source the affected packages:

```bash
cd ~/design_final/wqi_final/simulation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to cooperative_delivery uav_bringup
source install/setup.bash
```

#### Test 5A: Automatic Charging On The UGV

Terminal 1, open Gazebo and RViz with the UAV at 30%:

```bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  gui:=true rviz:=true visualize_sensor_rays:=false \
  initial_battery_percentage:=0.30
```

For VirtualBox, change this to `gui:=false rviz:=true` when both 3D viewers are
too slow.

In a second sourced terminal, inspect the charging state:

```bash
cd ~/design_final/wqi_final/simulation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /uav/docked --once
ros2 topic echo /uav/battery_status
```

`/uav/docked` must report `true`. The status must report `CHARGING`, and the
percentage must increase with simulation time. The default net charging power
is approximately `-157 W`; negative battery power means energy is entering the
battery. Additional values can be inspected with:

```bash
ros2 topic echo /uav/battery_percentage
ros2 topic echo /uav/battery_power_w
ros2 topic echo /uav/battery_charged_wh
ros2 topic echo /uav/payload_mass
ros2 topic echo /uav/total_mass_kg
ros2 topic echo /uav/propulsion_power_w
ros2 topic echo /uav/auxiliary_power_w
```

#### Test 5B: Preflight Energy Budget And Normal Route

Query the teaching-building UAV sortie without starting it:

```bash
ros2 service call /uav/check_delivery_energy \
  uav_interfaces/srv/CheckDeliveryEnergy \
  "{targets: ['teaching_building'], return_home: true, \
  home_name: 'teaching_building', landing_height: 0.42, \
  payload_masses_kg: [0.30], target_floors: [3]}"
```

With sufficient energy, the response must contain `feasible: true` together
with propulsion energy, auxiliary energy, payload penalty, reserve, required
energy, and predicted final state of charge.

Run the complete cooperative delivery:

```bash
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['teaching_building'], return_home: true}" --feedback
```

Charging continues while the UAV is carried by the UGV. After release, battery
status changes through takeoff, hover, cruise, and landing discharge modes.
After redocking it must return to `CHARGING`. Inspect the energy records with:

```bash
ros2 topic echo /uav/battery_consumed_wh
ros2 topic echo /uav/battery_charged_wh
ros2 topic echo /uav/energy_preflight --once
ros2 topic echo /cooperative_delivery/energy_plan --once
```

#### Test 5C: Low-Energy Cooperative Mission Rejection

Stop the previous launch with `Ctrl+C`, then start the combined system at 1%:

```bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  gui:=true rviz:=true visualize_sensor_rays:=false \
  initial_battery_percentage:=0.01
```

As soon as the cooperative action server is ready, submit the route in another
sourced terminal:

```bash
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['teaching_building'], return_home: true}" --feedback
```

The result must report `success: false` with a UAV battery-plan rejection.
The UGV must not start its route, and the UAV must remain docked and continue
charging. The planner accounts for guaranteed charging during UGV transit, so
`10%` may legitimately be accepted for a long ground leg. Submit the same goal
again only after `/uav/battery_percentage` exceeds the reported sequence
threshold; it must then be accepted.

#### Test 5D: Automated Regression Tests

```bash
cd ~/design_final/wqi_final/simulation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon test --event-handlers console_cohesion+
colcon test-result --verbose
```

The clean `v1.0.0` baseline contains 13 thesis packages. On 2026-07-20 a
build from empty build/install directories completed for all 13 packages, and
the full-workspace test run reported `143 tests, 0 errors, 0 failures,
0 skipped`. Functional, route-planning, copyright, lint, launch, and interface
tests all passed. Both UGV and UAV Xacro models passed `check_urdf`,
and the campus generator reproduced 11 buildings, 4 closed road groups, and
the occupancy/keepout maps. See
[`docs/evaluation_report.md`](docs/evaluation_report.md) for the exact evidence
and remaining formal experiment work.

## Map Regeneration

Regenerate the campus world, occupancy map, and keepout mask after editing the
layout:

```bash
python3 src/ugvcar_description/scripts/generate_campus_delivery.py
colcon build
```

## Next Graduation-Design Work

The control baseline is frozen. Remaining work is formal experiment coverage
and thesis evidence rather than another vehicle-control rewrite. The exact
experiment matrix and acceptance criteria are in
[`docs/next_stage_task_spec.md`](docs/next_stage_task_spec.md):

```text
repeat representative missions at least three times
record mission duration, path length, endpoint error, and success rate
compare UGV-only, UAV-only, and cooperative delivery results
use the simulated UAV battery traces in the comparison
calibrate the paper-model parameters and report prediction error
compare loaded and unloaded UAV sorties
```
