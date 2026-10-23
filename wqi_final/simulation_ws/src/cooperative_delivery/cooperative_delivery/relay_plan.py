"""Pure relay planning and guards, independent of ROS and Gazebo."""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml

from cooperative_delivery.mission_config import GroundWaypoint


@dataclass(frozen=True)
class RelayPlatform:
    node: str
    stop: GroundWaypoint


@dataclass(frozen=True)
class RelayConfig:
    platform_a: RelayPlatform
    platform_b: RelayPlatform
    transfer_altitude: float
    landing_height: float
    transfer_speed_mps: float
    transfer_vertical_speed_mps: float
    transfer_hover_seconds: float
    transfer_timeout_wall_s: float
    telemetry_timeout_wall_s: float

    @classmethod
    def load(cls, path, waypoint_map):
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("Relay configuration must be a mapping")
        platforms = []
        for role in ("a", "b"):
            raw = data[f"platform_{role}"]
            node = str(raw["node"])
            if node not in waypoint_map.corridor_nodes:
                raise ValueError(f"Unknown relay corridor node: {node}")
            anchor = waypoint_map.corridor_nodes[node]
            x, y, yaw = (float(raw[key]) for key in ("x", "y", "yaw"))
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise ValueError("Relay coordinates must be finite")
            if math.hypot(x - anchor.x, y - anchor.y) > 6.0:
                raise ValueError("Relay platform must remain near its corridor anchor")
            platforms.append(RelayPlatform(node, GroundWaypoint(f"platform_{role}", x, y, yaw)))
        if math.hypot(platforms[0].stop.x - platforms[1].stop.x,
                      platforms[0].stop.y - platforms[1].stop.y) < 2.5:
            raise ValueError("Relay platforms must be at least 2.5 m apart")
        values = {}
        for name in cls.__dataclass_fields__:
            if name.startswith("platform_"):
                continue
            value = float(data[name])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
            values[name] = value
        if abs(values["landing_height"] - .42) > 1e-6:
            raise ValueError("Relay landing height must match the existing 0.42 m physical deck")
        if not values["landing_height"] + .8 <= values["transfer_altitude"] <= 20:
            raise ValueError("Relay cruise altitude is outside the safe configured band")
        waypoint_map.plan_route(platforms[0].node, platforms[1].node)
        return cls(*platforms, **values)

    def platform(self, role):
        if role not in ("a", "b"):
            raise ValueError("Unknown carrier role")
        return getattr(self, f"platform_{role}")

    def air_route(self, waypoint_map, source, destination):
        first, last = self.platform(source), self.platform(destination)
        route = [(first.stop.x, first.stop.y, self.transfer_altitude)]
        interior = waypoint_map.plan_route(first.node, last.node)
        for point in interior[:-1]:
            route.append((point.x, point.y, self.transfer_altitude))
        route.append((last.stop.x, last.stop.y, self.transfer_altitude))
        return tuple(route)


def partition_payloads(targets, capacity):
    """Preserve input order and split cargo into capacity-feasible sorties."""
    if not math.isfinite(capacity) or capacity <= 0:
        raise ValueError("Payload capacity must be positive")
    batches, current, mass = [], [], 0.0
    for target in targets:
        payload = float(target.payload_mass_kg)
        if not math.isfinite(payload) or not 0 <= payload <= capacity:
            raise ValueError(f"Package {target.name} exceeds UAV payload capacity")
        if current and mass + payload > capacity + 1e-9:
            batches.append(tuple(current))
            current, mass = [], 0.0
        current.append(target)
        mass += payload
    if current:
        batches.append(tuple(current))
    return tuple(batches)


def can_move_carrier(role, docked_carrier, landed):
    # An empty receiver may reposition, but a carrier may not drive with live rotors.
    return role != docked_carrier or bool(landed)


def capture_pose_is_safe(car, aircraft, deck_height, filtered_velocity=None):
    """Conservative admission to the unchanged physical capture limits."""
    cp, up = car.pose.pose.position, aircraft.pose.pose.position
    cv, uv = car.twist.twist, aircraft.twist.twist
    values = [cp.x, cp.y, cp.z, up.x, up.y, up.z, deck_height]
    for velocity in (cv, uv):
        values.extend((velocity.linear.x, velocity.linear.y, velocity.linear.z,
                       velocity.angular.x, velocity.angular.y, velocity.angular.z))
    if not all(math.isfinite(v) for v in values):
        return False
    velocities = values[7:] if filtered_velocity is None else filtered_velocity
    if len(velocities) != 12 or not all(math.isfinite(v) for v in velocities):
        return False
    speeds = [math.sqrt(sum(v*v for v in velocities[i:i+3])) for i in range(0, 12, 3)]
    return (math.dist((cp.x, cp.y, cp.z + deck_height), (up.x, up.y, up.z)) <= .17
            and speeds[0] <= .04 and speeds[1] <= .08
            and speeds[2] <= .15 and speeds[3] <= .15)


@dataclass
class CaptureVelocityFilter:
    time_constant_s: float = .25
    stamp: float | None = None
    values: tuple | None = None

    def observe(self, stamp, values):
        if not math.isfinite(stamp) or not all(math.isfinite(v) for v in values):
            self.stamp, self.values = None, None
            return None
        if self.stamp is None or stamp < self.stamp:
            self.stamp, self.values = stamp, tuple(values)
        elif stamp > self.stamp:
            alpha = 1.0 - math.exp(-(stamp - self.stamp) / self.time_constant_s)
            self.values = tuple(old + alpha * (new - old) for old, new in zip(self.values, values))
            self.stamp = stamp
        return self.values


@dataclass
class CaptureSettleWindow:
    duration_s: float = 1.0
    since: float | None = None
    previous: float | None = None

    def observe(self, now, safe):
        if not math.isfinite(now) or (self.previous is not None and now < self.previous):
            self.since, self.previous = None, None
            return False
        self.previous = now
        if not safe:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.duration_s


