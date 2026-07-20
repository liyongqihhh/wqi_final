# Cooperative UGV-UAV Delivery Subsystem

## Scope

This subsystem combines the validated UGV and UAV without replacing either
vehicle controller. The UGV transports the UAV from the logistics center to the
configured service point at the target building door. Only after Nav2 reaches
that pose and the UGV settles does the UAV detach, ascend to the configured
floor, return to the same stopped vehicle, land, and redock.

The combined launch creates one Gazebo server and reuses the existing campus
world, occupancy map, keepout mask, Nav2 configuration, UAV corridor graph,
and vehicle packages.

## Packages

| Package | Responsibility |
|---|---|
| `cooperative_delivery_interfaces` | `ExecuteCooperativeDelivery` action |
| `cooperative_delivery` | Mission state machine, waypoint mapping, docking plugin, combined launch and RViz |
| `ugvcar_*` | Existing UGV model, Nav2, maps, localization and control |
| `uav_*` | Existing UAV model, flight control, safety, routing and delivery action |

## Control Boundaries

The cooperative manager sends a standard Nav2 `NavigateToPose` goal to the UGV
and an `ExecuteDelivery` goal to the UAV. It never publishes wheel commands,
rotor forces, or repeated entity poses.

The Gazebo docking plugin provides:

- `/uav/attach_uav`: validate capture distance, align the UAV with the platform,
  and create a fixed joint;
- `/uav/detach_uav`: remove the fixed joint after the UAV is pre-armed;
- `/uav/docked`: reliable transient-local docking state.

The plugin uses a `0.42 m` platform offset and a `0.8 m` capture limit. The UAV
dynamics plugin resolves its controlled link inside the UAV model, because UGV
and UAV link names such as `base_footprint` are not globally unique in Gazebo.

## Mission State Machine

```text
IDLE
-> PREPARING
-> UGV_TRANSIT
-> UGV_SETTLING
-> UAV_DETACHING
-> UAV_DELIVERING
-> UAV_DOCKING
-> RETURNING_HOME
-> COMPLETED
```

Any rejected action, failed docking operation, persistent UAV obstacle, Nav2
failure, or timeout moves the mission to `FAILED` and returns the reason in the
action result. A client-requested cancellation moves the mission to `CANCELED`
instead. Only one cooperative goal is accepted at a time.

For each target, `cooperative_waypoints.yaml` maps three names:

- the public delivery target;
- the UGV launch waypoint;
- the UAV corridor node used as its temporary home.

For cooperative building delivery, the UGV waypoint and temporary UAV home are
the same door-service node. The target entry in
`uav_delivery_waypoints.yaml` supplies the floor altitude. This creates the
required sequence of ground transport to the door followed by local vertical
delivery to the selected floor.

## VirtualBox Timing

The UGV total timeout is computed from the largest Nav2 route distance observed
for the current goal:

```text
timeout = clamp(180 s + route_distance * 6 s/m, 180 s, 900 s)
```

This total timeout is measured with the ROS simulation clock, so a valid route
is not aborted merely because Gazebo runs slower than real time. A separate
wall-clock watchdog aborts only when remaining distance fails to improve by at
least `0.05 m` for `180 s`. Nav2 action acknowledgements use a `5000 ms`
timeout. Cooperative feedback is limited to `2 Hz` to avoid unnecessary
terminal and DDS load.

## Build And Run

```bash
cd ~/design_final/wqi_final/simulation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to cooperative_delivery uav_bringup
source install/setup.bash
ros2 launch cooperative_delivery cooperative_delivery.launch.py \
  gui:=false rviz:=true visualize_sensor_rays:=false
```

Send a mission from another sourced terminal:

```bash
ros2 action send_goal /cooperative_delivery/execute_mission \
  cooperative_delivery_interfaces/action/ExecuteCooperativeDelivery \
  "{targets: ['teaching_building'], return_home: true}" --feedback
```

Use `gui:=true rviz:=false` when Gazebo visuals are required. The blue lidar and
range-ray graphic is disabled by default; set `visualize_sensor_rays:=true` to
show it without changing the sensor topics. Do not enable both 3D viewers in
the VirtualBox environment.

## Validation Evidence

The clean `v1.0.0` source baseline built all 13 thesis packages successfully.
The full-workspace test run reported 143 tests with no errors, failures, or
skipped checks. Both UAV and UGV Xacro files expanded and passed `check_urdf`.
The complete release evidence is recorded in
[`evaluation_report.md`](evaluation_report.md).

The latest teaching-building headless run verified the corrected handoff:

1. The UAV started attached to the UGV at the logistics center.
2. Nav2 drove the UGV to the teaching-building door stop `(-40, 9.5)`.
3. Gazebo ground truth reached approximately `(-39.874, 9.337)`, an XY error of
   about `0.21 m`, before the state changed to `UGV_SETTLING`.
4. Only after settling did the manager detach the UAV.
5. The UAV completed takeoff, hover, floor-3 approach at `8.0 m`, delivery,
   return, landing, and redocking at approximately `z=0.421 m`.

During the subsequent UGV return, the old implementation aborted at `900.1 s`
of wall time even though Nav2 still reported movement and `54.1 m` remaining.
Gazebo simulation time was only about `807.9 s`, identifying low real-time
factor rather than a stopped robot. The manager now uses simulation time for
the route timeout and the separate no-progress watchdog described above. A
focused runtime cancellation check after this change returned
`Cooperative mission canceled by client`, action status `CANCELED`, and topic
status `CANCELED:teaching_building`.

Door and floor configuration currently used by the manager:

| Target | Door XY (m) | Floor | Delivery altitude (m) |
|---|---:|---:|---:|
| `teaching_building` | `(-40, 9.5)` | 3 | `8.0` |
| `laboratory` | `(62, 19.5)` | 4 | `11.2` |
| `library` | `(62, 40.5)` | 3 | `8.0` |
| `innovation_center` | `(-20, 40.5)` | 3 | `8.0` |
| `cafeteria` | `(38, -20)` | 2 | `4.8` |
| `gymnasium` | `(14, 39.5)` | 2 | `4.8` |
| `dormitory` | `(30, 11)` | 3 | `8.0` |

## Remaining Evaluation

- Repeat representative missions at least three times and report success rate,
  elapsed time, endpoint error and recovery count.
- Record one uninterrupted teaching-building round trip after the
  simulation-time timeout change. The corrected outbound handoff and complete
  UAV subtask are verified, but the post-fix UGV return has not yet been run to
  completion.
- Run the final `laboratory -> library -> dormitory` command uninterrupted after
  the door/floor mapping change.
- Record the implemented UAV cumulative consumption/charging topics and add a
  separate UGV energy estimate for the final three-mode efficiency comparison.
- Online three-dimensional replanning and moving-platform landing are outside
  the current fixed-corridor, fixed-rendezvous design.
