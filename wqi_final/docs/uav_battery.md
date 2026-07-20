# UAV Propulsion, Payload, Battery, And Cooperative Energy Model

## Scope

The campus UAV uses a mission-level physical energy model rather than a fixed
percentage loss or a phase lookup table. The same implementation is used for:

- runtime battery discharge from Gazebo odometry;
- payload-aware preflight prediction;
- low-energy mission rejection;
- UGV dock charging; and
- complete cooperative mission-sequence admission.

The model combines:

1. Zeng, Xu, and Zhang's rotary-wing horizontal propulsion model;
2. Dai et al.'s acceleration and direction-change thrust-to-weight ratio;
3. Gong et al.'s multi-rotor vertical-flight momentum model;
4. a dynamic package mass schedule;
5. constant onboard electronics loads; and
6. battery energy integration and UGV charging.

Primary references:

- [Energy Minimization for Wireless Communication with Rotary-Wing UAV](https://arxiv.org/abs/1804.02238)
- [Modelling Power Consumptions for Multi-rotor UAVs](https://arxiv.org/abs/2209.04128)
- [Energy-Efficient UAV Communications: A Generalised Propulsion Energy Consumption Model](https://arxiv.org/abs/2202.08486)

## Aircraft Calibration

The paper example parameters are not copied directly. The simulated aircraft
geometry and mass are used:

| Parameter | Value | Source or role |
|---|---:|---|
| Airframe mass | `1.200 kg` | UAV mass decomposition |
| Sensor mass | `0.277 kg` | Completes the URDF mass |
| Unloaded total mass | `1.477 kg` | `uav.urdf.xacro` |
| Maximum package mass | `1.000 kg` | Mission constraint |
| Rotor count | `4` | Quadrotor |
| Rotor radius | `0.120 m` | UAV rotor geometry |
| Total rotor area | `4*pi*0.12^2 = 0.181 m^2` | Derived |
| Air density | `1.225 kg/m^3` | Sea-level simulation |
| Blade profile power | `105 W` | Hover calibration |
| Rotor tip speed | `120 m/s` | Small-quadrotor assumption |
| Discharge efficiency | `0.92` | Battery-to-load efficiency |

The `105 W` blade-profile term is calibrated so that the unloaded hover battery
draw is approximately `240 W` after induced power, electronics, and discharge
efficiency are included. It is a calibration parameter, not a measured motor
constant.

All values are in:

```text
simulation_ws/src/uav_control/config/battery_model.yaml
```

## Dynamic Mass And Payload

At time step `k`:

```text
m(k) = m_airframe + m_sensor + m_payload(k)
W(k) = m(k) * g
A = n_rotor * pi * R_rotor^2
```

The payload schedule is discrete:

```text
before takeoff: m_payload = sum(all package masses)
after target i: m_payload = m_payload - package_mass[i]
after final delivery: m_payload = 0
```

The corrected hover-induced power and mean induced velocity are:

```text
P_i(W) = (1 + k) * W^(3/2) / sqrt(2 * rho * A)
v_0(W) = sqrt(W / (2 * rho * A))
```

Payload power is therefore nonlinear. A 20% mass increase produces more than a
20% induced-power increase because the exponent is `3/2`.

## Horizontal Propulsion

Let:

```text
V_h = sqrt(v_x^2 + v_y^2)
```

The dynamic thrust-to-weight ratio from Dai et al. is:

```text
kappa(a_h, v_h) =
  sqrt(
    1 +
    (
      4*m^2*|a_h|^2
      + rho^2*S_FP^2*|v_h|^4
      + 4*m*rho*S_FP*(a_h dot v_h)*|v_h|
    ) / (4*W^2)
  )
```

This term distinguishes:

- acceleration along the route;
- deceleration;
- perpendicular acceleration during a turn; and
- steady straight flight.

The horizontal propulsion power is the generalized Zeng model:

```text
P_h =
  P_0 * (1 + 3*V_h^2/U_tip^2)
  + P_i * kappa
    * sqrt(
        sqrt(kappa^2 + V_h^4/(4*v_0^4))
        - V_h^2/(2*v_0^2)
      )
  + 0.5*d_0*rho*s*A*V_h^3
```

At zero speed and acceleration:

```text
P_hover = P_0 + P_i
```

The power-speed curve is not linear or monotonic over its full range. Induced
power initially falls with forward speed, while blade-profile and parasite
power eventually dominate.

## Vertical Propulsion

Gong et al. derive separate ascent and descent behavior from thrust and
momentum theory. This implementation uses a signed vertical velocity `v_z`.
The signed vertical drag is:

```text
D_z = 0.5 * rho * S_perpendicular * v_z * |v_z|
```

For constant-speed flight, the thrust reduces to the paper's ascent/descent
force balance. Runtime acceleration is added with Newton's second law:

```text
T = W + m*a_z + D_z
```

The vertical propulsion power is:

```text
P_z =
  P_0
  + (1 + k) * T/2
    * (v_z + sqrt(v_z^2 + 2*T/(rho*A)))
```

Consequences:

- positive `v_z` raises climb power;
- negative `v_z` lowers steady descent power;
- positive `a_z` during landing braking raises thrust and power;
- payload raises `W`, `T`, and induced power; and
- ascent and descent are no longer assigned arbitrary fixed coefficients.

Gong et al. validate their constant-speed model experimentally. The `m*a_z`
term is an explicit engineering extension for this simulation and must be
described as such in the thesis.

## Combined Three-Dimensional Power

Horizontal and vertical models both include hover power. It is subtracted once:

```text
P_propulsion = P_h + P_z - P_hover
```

A small lower bound prevents an invalid negative result outside the model's
normal operating envelope:

```text
P_propulsion = max(0.2*P_hover, P_propulsion)
```

The constant auxiliary load is:

```text
P_auxiliary =
  P_computer + P_lidar + P_camera + P_communication
```

Default auxiliary values are:

| Device | Power |
|---|---:|
| Computer | `12 W` |
| 3D lidar | `8 W` |
| Down camera | `3 W` |
| Communication | `2 W` |
| Total | `25 W` |

The battery-side discharge power is:

```text
P_battery = (P_propulsion + P_auxiliary) / eta_discharge
```

## Runtime Energy Integration

`battery_manager` reads the full velocity vector from `/uav/odom`. It computes
the three acceleration components and applies a first-order low-pass filter:

```text
a_filtered(k) =
  a_filtered(k-1) + alpha * (a_raw(k) - a_filtered(k-1))
alpha = 0.25
```

The battery update in watt-hours is:

```text
E(k+1) = E(k) - P_battery(k) * dt / 3600
SOC(k) = E(k) / C_battery
```

When the UAV is physically docked:

```text
P_charge_net =
  eta_charge * P_charge - P_docked_idle

E(k+1) =
  min(C_battery, E(k) + P_charge_net * dt / 3600)
```

With the default values:

```text
P_charge_net = 0.90*180 - 5 = 157 W
```

## Preflight Route Integration

Each horizontal corridor edge and vertical altitude change uses a trapezoidal
speed profile. For distance `d`, speed limit `v`, and acceleration `a`:

```text
d_ramp = v^2 / (2*a)
```

For `d >= 2*d_ramp`:

```text
t_accel = t_decel = v/a
t_cruise = (d - 2*d_ramp)/v
```

For a short triangular segment:

```text
v_peak = sqrt(a*d)
t_accel = t_decel = v_peak/a
t_cruise = 0
```

The nonlinear physical power model is evaluated at `0.1 s` midpoint samples
through every acceleration, cruise, deceleration, climb, descent, and hover
segment:

```text
E_raw = sum(P_battery(v_j, a_j, payload_j) * dt_j / 3600)
E_predicted = 1.25 * E_raw
E_reserve = 0.20 * C_battery
E_required = E_predicted + E_reserve
```

The UAV sortie is admitted only when:

```text
E_available >= E_required
```

Safe return is included in admission even when a request sets
`return_home=false`.

## Cooperative Mission-Sequence Planning

The cooperative manager requests a payload-specific UAV energy estimate for
every target before the UGV starts. It then simulates the sequence in the
requested order.

For UGV leg `j`, Euclidean distance is used as a lower bound on road distance:

```text
t_ugv_min[j] = d_euclidean[j] / v_ugv_limit
E_charge_min[j] = P_charge_net * t_ugv_min[j] / 3600
```

Since the real road path is at least as long and Nav2 normally drives below the
configured `0.22 m/s` cap, this intentionally underestimates available charging.

For every sortie:

```text
E_takeoff[j] =
  min(C_battery, E_landing[j-1] + E_charge_min[j])

accept j only if:
E_takeoff[j] >= E_sortie[j] + E_reserve

E_landing[j] = E_takeoff[j] - E_sortie[j]
```

If any target violates the constraint, the combined action is aborted before
the UGV begins moving. The same live preflight check is repeated at the
building immediately before UAV release.

## ROS 2 Interfaces

| Topic or service | Type | Meaning |
|---|---|---|
| `/uav/payload_mass` | `std_msgs/msg/Float32` | Current undelivered payload |
| `/uav/total_mass_kg` | `std_msgs/msg/Float32` | Airframe, sensors, and payload |
| `/uav/power_consumption` | `std_msgs/msg/Float32` | Battery-side power |
| `/uav/propulsion_power_w` | `std_msgs/msg/Float32` | Propulsion component |
| `/uav/auxiliary_power_w` | `std_msgs/msg/Float32` | Electronics component |
| `/uav/remaining_energy` | `std_msgs/msg/Float32` | Remaining watt-hours |
| `/uav/battery_state` | `sensor_msgs/msg/BatteryState` | Standard battery state |
| `/uav/battery_percentage` | `std_msgs/msg/Float32` | SOC in percent |
| `/uav/battery_status` | `std_msgs/msg/String` | Mode, SOC, energy, power, payload |
| `/uav/can_execute_task` | `std_msgs/msg/Bool` | Last UAV preflight decision |
| `/uav/check_delivery_energy` | `uav_interfaces/srv/CheckDeliveryEnergy` | Payload-aware sortie estimate |
| `/cooperative_delivery/energy_plan` | `std_msgs/msg/String` | Full sequence decision |

`ExecuteDelivery.action` and `CheckDeliveryEnergy.srv` accept an optional
`payload_masses_kg` array. An empty array uses the per-target defaults in
`uav_delivery_waypoints.yaml`.

## Validation Record

The following headless ROS 2 and Gazebo checks were run on 2026-07-16 after a
clean rebuild of all packages up to `cooperative_delivery` and `uav_bringup`.

For a `0.30 kg` teaching-building package at approximately 80% SOC, the local
sortie preflight returned:

| Quantity | Result |
|---|---:|
| Raw integrated mission energy | `2.803 Wh` |
| Margin-adjusted propulsion energy | `3.138 Wh` |
| Margin-adjusted auxiliary energy | `0.366 Wh` |
| Margin-adjusted payload penalty | `0.267 Wh` |
| Total predicted mission energy | `3.503 Wh` |
| Safety reserve | `20.000 Wh` |
| Required energy | `23.503 Wh` |

A separate runtime Action test completed takeoff, hover, delivery, return, and
landing with `success: true`. The payload topic changed from `0.30 kg` to
`0.00 kg`, and odometry-driven integration recorded `3.663 Wh` consumed. This
runtime check proves state and energy integration, but it is not used as a
calibration-error result because its physical launch pose was intentionally
different from the local cooperative preflight profile.

At 1% initial SOC in the combined simulation, the docking topic remained
`true` and battery status reported `CHARGING` at `157 W` net input. Even after
including guaranteed charging during the UGV leg, sequence planning estimated
only `15.06 Wh` at takeoff. It rejected the teaching-building mission because
the sortie plus reserve required `23.50 Wh`; completed targets remained zero
and the UGV stayed at the logistics-center pose.

A normal 30% SOC cooperative smoke test used the logistics-center target to
exercise the complete manager chain without a long campus drive. Sequence
planning passed (`30.4 -> 28.5 Wh`), the combined Action completed one target,
and the UAV detached, flew, landed, and redocked. Post-mission topics reported
`docked: true`, `CHARGING`, `1.124 Wh` consumed, and `0.685 Wh` charged. This
short test validates the acceptance and redocking branches; final thesis data
must still use representative building routes rather than this zero-distance
UGV leg.

Package-scoped automated results were:

| Package | Passing records |
|---|---:|
| `uav_control` | `30` |
| `uav_application` | `14` |
| `uav_navigation` | `12` |
| `uav_description` | `2` |
| `cooperative_delivery` | `21` Python cases (`25` with CTest wrappers) |

## Calibration And Thesis Experiments

The current defaults are defensible simulation assumptions, not hardware
measurements. For the final thesis:

1. Record modeled unloaded hover power for at least 60 seconds.
2. Repeat hover with several payload masses.
3. Record steady horizontal power at several speeds.
4. Record climb and descent power at several vertical speeds.
5. Compare predicted and simulated integrated energy for three routes.
6. Adjust `blade_profile_power_w`, drag areas, and efficiency from those data.
7. Report both the parameter table and prediction error.

This model does not simulate cell voltage sag, temperature, battery aging,
motor RPM, ESC efficiency maps, wind, or a vortex-ring-state descent. Those are
explicit limitations, not hidden assumptions.
