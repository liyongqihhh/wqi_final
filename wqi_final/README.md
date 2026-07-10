# wqi_final

ROS 2 + Gazebo/RViz simulation project for a graduation design about UGV autonomous navigation and delivery.

The current codebase keeps the simulation and navigation parts that are useful for the final project, and removes chapter-style tutorial examples such as hello world, basic topic/service demos, rosbag exercises, and hardware-board bringup code.

## Project Layout

```text
wqi_final/
└── simulation_ws/
    └── src/
        ├── ugvcar_description       # UGV model, Gazebo worlds, campus map generator, ros2_control config
        ├── ugvcar_navigation2       # Nav2 maps, parameters, launch files
        ├── ugvcar_application       # Python Nav2 application examples
        ├── ugvcar_application_cpp   # C++ Nav2 action client example
        ├── autopatrol_interfaces     # Service interface used by patrol demo
        ├── autopatrol_robot          # Patrol node, speech node, waypoint config
        ├── nav2_custom_planner       # Custom Nav2 global planner plugin
        └── nav2_custom_controller    # Custom Nav2 controller plugin
```

## Current Capability

- Simulate a UGVcar-style ground vehicle in Gazebo.
- Run Nav2 localization, planning, and navigation in RViz.
- Send single goals or waypoint goals through ROS 2 nodes.
- Run a patrol workflow with speech and image capture.
- Provide a campus logistics delivery scene with connected roads, buildings, park area, perimeter roads, occupancy map, and keepout mask.
- Provide custom Nav2 planner/controller plugin examples for later algorithm expansion.

## Build

```bash
cd wqi_final/simulation_ws
colcon build
source install/setup.bash
```

## Run

Start the original room Gazebo simulation:

```bash
ros2 launch ugvcar_description gazebo_sim.launch.py
```

Start Nav2 and RViz with the original room map:

```bash
ros2 launch ugvcar_navigation2 navigation2.launch.py
```

Start the campus logistics delivery simulation:

```bash
ros2 launch ugvcar_description campus_delivery_sim.launch.py
```

Start Nav2 and RViz with the campus delivery map:

```bash
ros2 launch ugvcar_navigation2 campus_navigation.launch.py
```

Regenerate the campus world, occupancy map, and keepout mask after editing the layout:

```bash
python3 src/ugvcar_description/scripts/generate_campus_delivery.py
colcon build
```

Run patrol application:

```bash
ros2 launch autopatrol_robot autopatrol.launch.py
```

## Next Development Modules

The next graduation-design work should add these ROS 2 packages under `simulation_ws/src`:

```text
ugv_task_manager       # Dispatch navigation, patrol, and delivery tasks
navigation_logger      # CSV logging for task time, path length, success rate
delivery_logger        # Delivery task records and experiment summaries
```

The minimal final demonstration should be:

```text
task goal
-> UGV initializes localization in RViz/Nav2
-> UGV navigates to pickup point with Nav2
-> UGV navigates to delivery target
-> UGV records image or patrol evidence
-> UGV returns to start or charging point
-> task result is logged
```
