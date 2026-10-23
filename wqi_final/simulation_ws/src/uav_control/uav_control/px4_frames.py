import math


def map_enu_to_local_ned(position, reference):
    """Convert a map ENU position to the PX4 local NED frame."""
    east, north, up = (float(value) for value in position)
    ref_east, ref_north, ref_up = (float(value) for value in reference)
    return north - ref_north, east - ref_east, ref_up - up


def local_ned_to_map_enu(position, reference):
    """Convert a PX4 local NED position to the campus map ENU frame."""
    north, east, down = (float(value) for value in position)
    ref_east, ref_north, ref_up = (float(value) for value in reference)
    return east + ref_east, north + ref_north, ref_up - down


def reference_from_positions(map_position, local_ned_position):
    east, north, up = (float(value) for value in map_position)
    local_north, local_east, local_down = (
        float(value) for value in local_ned_position
    )
    return (
        east - local_east,
        north - local_north,
        up + local_down,
    )


def yaw_enu_to_ned(yaw_enu):
    """Convert ROS yaw (CCW from east) to PX4 yaw (CW from north)."""
    yaw = math.pi / 2.0 - float(yaw_enu)
    return math.atan2(math.sin(yaw), math.cos(yaw))


def landing_setpoint_ned(local_position, descent_speed, velocity_bias=None, hold_altitude=False):
    """Keep XY position control while commanding positive-down Z velocity."""
    north, east, down = local_position
    if not all(math.isfinite(v) for v in (north, east, descent_speed)) or descent_speed < 0:
        raise ValueError("Landing position and descent speed must be valid")
    position = [float(north), float(east), float(down) if hold_altitude else math.nan]
    if hold_altitude and not math.isfinite(down):
        raise ValueError("Alignment altitude must be finite")
    if velocity_bias is None:
        return position, [math.nan, math.nan, float(descent_speed)]
    if len(velocity_bias) != 3 or not all(math.isfinite(v) for v in velocity_bias):
        raise ValueError("Landing velocity correction must be finite")
    vertical_velocity = descent_speed + velocity_bias[2]
    if descent_speed > 0:
        # PX4 needs an explicit downward request for native landing detection.
        vertical_velocity = max(descent_speed, vertical_velocity)
    return position, [float(velocity_bias[0]), float(velocity_bias[1]),
                      float(vertical_velocity)]


def map_reference_velocity_bias(map_velocity_enu, local_velocity_ned):
    """Derivative of the continuously updated map-to-local position offset."""
    if not all(math.isfinite(v) for v in (*map_velocity_enu, *local_velocity_ned)):
        raise ValueError("Frame velocities must be finite")
    east, north, up = map_velocity_enu
    return (local_velocity_ned[0] - north, local_velocity_ned[1] - east,
            local_velocity_ned[2] + up)


def landing_is_aligned(position, target, velocity, position_tolerance=.08, speed_tolerance=.15):
    values = (*position[:2], *target[:2], *velocity[:2])
    return (all(math.isfinite(v) for v in values)
            and math.hypot(position[0] - target[0], position[1] - target[1]) <= position_tolerance
            and math.hypot(velocity[0], velocity[1]) <= speed_tolerance)


def landing_alignment_gate(position, target, velocity, descending,
                           position_tolerance=.08, speed_tolerance=.15):
    # Keep descent through small contact/estimator disturbances, but suspend it
    # for a real departure from the deck alignment envelope.
    factor = 2.5 if descending else 1.0
    return landing_is_aligned(position, target, velocity,
                              position_tolerance * factor, speed_tolerance * factor)


def landing_deadline_expired(start_ns, now_ns, timeout_s):
    return start_ns is not None and (now_ns-start_ns)/1e9 >= timeout_s


def px4_position_is_ready(local):
    return (local is not None and local.xy_valid and local.z_valid
            and local.v_xy_valid and local.v_z_valid
            and all(math.isfinite(value) for value in
                    (local.x, local.y, local.z, local.vx, local.vy, local.vz, local.heading)))


def map_velocity_to_ned(velocity):
    if len(velocity) != 3 or not all(math.isfinite(v) for v in velocity):
        raise ValueError("Simulation velocity must be finite")
    east, north, up = velocity
    return [float(north), float(east), float(-up)]


def velocity_sample_is_fresh(sample_ns, now_ns, last_sample_ns=None):
    return (sample_ns > 0 and -.25 <= (now_ns-sample_ns)/1e9 <= .5
            and (last_sample_ns is None or sample_ns > last_sample_ns))


def deck_contact_is_stable(position, target, velocity, quaternion, deck_height):
    values = (*position, *target[:2], *velocity, *quaternion, deck_height)
    if not all(math.isfinite(v) for v in values) or not .2 <= deck_height <= 2.0:
        return False
    qx, qy, qz, qw = quaternion
    norm_squared = qx*qx + qy*qy + qz*qz + qw*qw
    return (norm_squared > 1e-6
            and abs(position[2]-deck_height) <= .025
            and math.hypot(position[0]-target[0], position[1]-target[1]) <= .08
            and math.hypot(velocity[0], velocity[1]) <= .05
            and abs(velocity[2]) <= .05
            and 1-2*(qx*qx+qy*qy)/norm_squared >= math.cos(math.radians(15)))
