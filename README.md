# wqi_final

ROS 2 + Gazebo/RViz simulation project for a graduation design about UGV-UAV cooperative delivery.

The current codebase keeps the simulation and navigation parts that are useful for the final project, and removes chapter-style tutorial examples such as hello world, basic topic/service demos, rosbag exercises, and hardware-board bringup code.

## Project Layout

```text
wqi_final/
└── simulation_ws/
    └── src/
        ├── uavcar_description       # UGV model, Gazebo world, ros2_control config
        ├── uavcar_navigation2       # Nav2 map, parameters, launch files
        ├── uavcar_application       # Python Nav2 application examples
        ├── uavcar_application_cpp   # C++ Nav2 action client example
        ├── autopatrol_interfaces     # Service interface used by patrol demo
        ├── autopatrol_robot          # Patrol node, speech node, waypoint config
        ├── nav2_custom_planner       # Custom Nav2 global planner plugin
        └── nav2_custom_controller    # Custom Nav2 controller plugin
```

## Current Capability

- Simulate a UAVcar-style UGV in Gazebo.
- Run Nav2 localization, planning, and navigation in RViz.
- Send single goals or waypoint goals through ROS 2 nodes.
- Run a patrol workflow with speech and image capture.
- Provide custom Nav2 planner/controller plugin examples for later algorithm expansion.

## Build

```bash
cd wqi_final/simulation_ws
colcon build
source install/setup.bash
```

## Run

Start Gazebo simulation:

```bash
ros2 launch uavcar_description gazebo_sim.launch.py
```

Start Nav2 and RViz:

```bash
ros2 launch uavcar_navigation2 navigation2.launch.py
```

Run patrol application:

```bash
ros2 launch autopatrol_robot autopatrol.launch.py
```

## Next Development Modules

The next graduation-design work should add these ROS 2 packages under `simulation_ws/src`:

```text
uav_simulator          # Simulated UAV pose, battery, takeoff, delivery, return, landing
air_ground_scheduler   # Cooperative task scheduler for UGV and UAV
delivery_logger        # CSV logging for task time, path length, battery, success rate
```

The minimal final demonstration should be:

```text
task goal
-> UGV navigates to rendezvous point with Nav2
-> UAV takes off from UGV
-> UAV flies to delivery target
-> UAV returns and lands
-> UAV battery is charged
-> task result is logged
```

