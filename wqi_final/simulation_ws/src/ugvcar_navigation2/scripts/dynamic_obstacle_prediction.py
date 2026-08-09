from dataclasses import dataclass, field, replace
import math


@dataclass(frozen=True)
class ScanCluster:
    x: float
    y: float
    diameter: float
    samples: int


@dataclass
class TrackedObstacle:
    identifier: int
    x: float
    y: float
    vx: float
    vy: float
    observations: int
    updated_at: float
    diameter: float = 0.0
    fit_residual: float = math.inf
    motion_confirmed: bool = False
    history: list = field(default_factory=list, repr=False)

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    @property
    def motion_duration(self) -> float:
        if len(self.history) < 2:
            return 0.0
        return max(0.0, self.history[-1][0] - self.history[0][0])

    @property
    def net_displacement(self) -> float:
        if len(self.history) < 2:
            return 0.0
        return _distance(self.history[0][1:], self.history[-1][1:])

    @property
    def direction_consistency(self) -> float:
        if len(self.history) < 2:
            return 0.0
        travelled = sum(
            _distance(first[1:], second[1:])
            for first, second in zip(self.history, self.history[1:])
        )
        if travelled <= 1.0e-9:
            return 0.0
        return self.net_displacement / travelled


@dataclass(frozen=True)
class PoseSample:
    timestamp: float
    x: float
    y: float
    yaw: float

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw


def _angle_delta(start: float, end: float) -> float:
    return math.atan2(math.sin(end - start), math.cos(end - start))


class PoseHistory:
    """Interpolate map-frame poses at sensor timestamps."""

    def __init__(
        self,
        history_duration: float = 3.0,
        maximum_extrapolation: float = 0.12,
    ) -> None:
        self.history_duration = float(history_duration)
        self.maximum_extrapolation = float(maximum_extrapolation)
        self.samples = []

    def add(self, timestamp: float, pose) -> None:
        timestamp = float(timestamp)
        x, y, yaw = (float(value) for value in pose)
        if not all(math.isfinite(value) for value in (timestamp, x, y, yaw)):
            return
        if self.samples and timestamp < self.samples[-1].timestamp - 1.0e-6:
            self.samples.clear()
        sample = PoseSample(timestamp, x, y, yaw)
        if (
            self.samples
            and abs(timestamp - self.samples[-1].timestamp) <= 1.0e-9
        ):
            self.samples[-1] = sample
        else:
            self.samples.append(sample)
        oldest = timestamp - self.history_duration
        while len(self.samples) > 2 and self.samples[1].timestamp < oldest:
            self.samples.pop(0)

    def nearest_time_gap(self, timestamp: float) -> float:
        if not self.samples:
            return math.inf
        return min(
            abs(float(timestamp) - sample.timestamp) for sample in self.samples
        )

    def pose_at(self, timestamp: float):
        target = float(timestamp)
        if not self.samples or not math.isfinite(target):
            return None
        first = self.samples[0]
        if target <= first.timestamp:
            if first.timestamp - target <= self.maximum_extrapolation:
                return first.pose
            return None

        for left, right in zip(self.samples, self.samples[1:]):
            if left.timestamp <= target <= right.timestamp:
                span = right.timestamp - left.timestamp
                if span <= 1.0e-9:
                    return right.pose
                ratio = (target - left.timestamp) / span
                return (
                    left.x + ratio * (right.x - left.x),
                    left.y + ratio * (right.y - left.y),
                    left.yaw + ratio * _angle_delta(left.yaw, right.yaw),
                )

        latest = self.samples[-1]
        age = target - latest.timestamp
        if age > self.maximum_extrapolation:
            return None
        if len(self.samples) < 2:
            return latest.pose
        previous = self.samples[-2]
        span = latest.timestamp - previous.timestamp
        if span <= 1.0e-6:
            return latest.pose
        return (
            latest.x + (latest.x - previous.x) * age / span,
            latest.y + (latest.y - previous.y) * age / span,
            latest.yaw + _angle_delta(previous.yaw, latest.yaw) * age / span,
        )

    def velocity_at(self, timestamp: float):
        """Return map-frame linear velocity around a sensor timestamp."""
        target = float(timestamp)
        if len(self.samples) < 2 or not math.isfinite(target):
            return None
        pair = None
        for left, right in zip(self.samples, self.samples[1:]):
            if left.timestamp <= target <= right.timestamp:
                pair = left, right
                break
        if pair is None:
            if (
                target < self.samples[0].timestamp
                and self.samples[0].timestamp - target
                > self.maximum_extrapolation
            ):
                return None
            if (
                target > self.samples[-1].timestamp
                and target - self.samples[-1].timestamp
                > self.maximum_extrapolation
            ):
                return None
            pair = (
                (self.samples[0], self.samples[1])
                if target < self.samples[0].timestamp
                else (self.samples[-2], self.samples[-1])
            )
        left, right = pair
        elapsed = right.timestamp - left.timestamp
        if elapsed <= 1.0e-6:
            return None
        return (
            (right.x - left.x) / elapsed,
            (right.y - left.y) / elapsed,
        )


def _distance(first, second) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


def effective_prediction_radius(
    configured_radius: float,
    obstacle_radius: float,
    safety_margin: float,
) -> float:
    """Keep a predictive obstacle outside the physical contact boundary."""
    values = tuple(
        float(value)
        for value in (configured_radius, obstacle_radius, safety_margin)
    )
    if (
        not all(math.isfinite(value) for value in values)
        or configured_radius <= 0.0
        or obstacle_radius <= 0.0
        or safety_margin < 0.0
    ):
        raise ValueError("Prediction-radius parameters must be finite and valid")
    return max(configured_radius, obstacle_radius + safety_margin)


def choose_open_avoidance_side(
    left_costs,
    right_costs,
    blocked_cost: float = 65.0,
    preference_margin: float = 15.0,
) -> str | None:
    """Select a road side only when its static clearance is clearly better."""
    left = tuple(float(value) for value in left_costs)
    right = tuple(float(value) for value in right_costs)
    limits = (float(blocked_cost), float(preference_margin))
    if (
        not left
        or not right
        or not all(math.isfinite(value) for value in (*left, *right, *limits))
        or blocked_cost <= 0.0
        or preference_margin < 0.0
    ):
        raise ValueError("Road-side cost samples must be finite and valid")

    left_peak = max(left)
    right_peak = max(right)
    if left_peak < blocked_cost <= right_peak:
        return "left"
    if right_peak < blocked_cost <= left_peak:
        return "right"

    left_score = left_peak + sum(left) / len(left)
    right_score = right_peak + sum(right) / len(right)
    if left_score + preference_margin < right_score:
        return "left"
    if right_score + preference_margin < left_score:
        return "right"
    return None


def plan_goal_changed(previous_points, current_points, tolerance: float = 0.50) -> bool:
    """Return true when a new plan starts a different navigation task."""
    tolerance = float(tolerance)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("Plan-goal tolerance must be finite and non-negative")
    previous = tuple(previous_points)
    current = tuple(current_points)
    if not current:
        return False
    if not previous:
        return True
    return _distance(previous[-1], current[-1]) > tolerance


def time_to_collision_boundary(
    relative_position,
    relative_velocity,
    collision_radius: float,
    horizon: float,
) -> float:
    """
    Return first entry into a relative-motion collision circle.

    Both bodies are represented by their combined safety radius.  The result
    uses their measured map-frame velocities, so the same calculation covers
    an obstacle approaching from the front, crossing the route, or catching
    the UGV from behind.
    """
    position = tuple(float(value) for value in relative_position)
    velocity = tuple(float(value) for value in relative_velocity)
    collision_radius = float(collision_radius)
    horizon = float(horizon)
    if (
        len(position) != 2
        or len(velocity) != 2
        or not all(math.isfinite(value) for value in (*position, *velocity))
        or not math.isfinite(collision_radius)
        or not math.isfinite(horizon)
        or collision_radius <= 0.0
        or horizon <= 0.0
    ):
        raise ValueError("Collision-boundary parameters must be valid")

    c = position[0] ** 2 + position[1] ** 2 - collision_radius ** 2
    if c <= 0.0:
        return 0.0
    a = velocity[0] ** 2 + velocity[1] ** 2
    if a <= 1.0e-12:
        return math.inf
    b = 2.0 * (
        position[0] * velocity[0] + position[1] * velocity[1]
    )
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0.0:
        return math.inf
    square_root = math.sqrt(max(0.0, discriminant))
    roots = (
        (-b - square_root) / (2.0 * a),
        (-b + square_root) / (2.0 * a),
    )
    entries = tuple(value for value in roots if 0.0 <= value <= horizon)
    return min(entries) if entries else math.inf


@dataclass(frozen=True)
class CollisionRisk:
    identifier: int
    zone: str
    avoidance_side: str
    obstacle_position: tuple[float, float]
    obstacle_velocity: tuple[float, float]
    robot_velocity: tuple[float, float]
    obstacle_speed: float
    closing_speed: float
    current_distance: float
    safe_distance: float
    maneuver_time: float
    time_to_collision: float
    time_to_closest: float
    closest_distance: float
    collision_point: tuple[float, float]
    safe_point: tuple[float, float]
    guard_points: tuple[tuple[float, float], ...]
    threatening: bool


def choose_drivable_collision_risk(
    primary: CollisionRisk,
    alternative: CollisionRisk,
    primary_cost: float,
    alternative_cost: float,
    blocked_cost: float,
    preference_margin: float,
) -> CollisionRisk:
    """Select an alternate passing side only when its safe point is drivable."""
    costs = (primary_cost, alternative_cost, blocked_cost, preference_margin)
    if (
        not all(math.isfinite(float(value)) for value in costs)
        or blocked_cost <= 0.0
        or preference_margin < 0.0
    ):
        raise ValueError("Road-side selection costs must be valid")
    if primary.avoidance_side == alternative.avoidance_side:
        return primary
    alternate_is_open = float(alternative_cost) < float(blocked_cost)
    primary_is_blocked = float(primary_cost) >= float(blocked_cost)
    clearly_better = (
        float(alternative_cost) + float(preference_margin)
        <= float(primary_cost)
    )
    if alternate_is_open and (primary_is_blocked or clearly_better):
        return alternative
    return primary


@dataclass
class _LatchedCollisionEvent:
    identifier: int
    risk: CollisionRisk
    obstacle_position: tuple[float, float]
    clear_observations: int = 0
    lost_observations: int = 0
    track_identifiers: set = field(default_factory=set)


class CollisionRiskLatch:
    """Hold independent physical collision events while Nav2 detours."""

    def __init__(
        self,
        clear_confirmations: int = 8,
        lost_confirmations: int = None,
        release_distance: float = 3.50,
        reassociation_distance: float = 0.90,
    ) -> None:
        self.clear_confirmations = max(1, int(clear_confirmations))
        self.lost_confirmations = max(
            1,
            int(
                clear_confirmations
                if lost_confirmations is None
                else lost_confirmations
            ),
        )
        self.release_distance = float(release_distance)
        self.reassociation_distance = float(reassociation_distance)
        if (
            not math.isfinite(self.release_distance)
            or self.release_distance <= 0.0
        ):
            raise ValueError("Risk release distance must be positive")
        if (
            not math.isfinite(self.reassociation_distance)
            or self.reassociation_distance <= 0.0
        ):
            raise ValueError("Risk reassociation distance must be positive")
        self.events = {}
        self.track_to_event = {}
        self.next_event_identifier = 1
        self.last_avoidance_side = None

    def reset(self) -> None:
        self.events.clear()
        self.track_to_event.clear()
        self.next_event_identifier = 1
        self.last_avoidance_side = None

    @property
    def preferred_avoidance_side(self):
        if self.events:
            first_identifier = min(self.events)
            return self.events[first_identifier].risk.avoidance_side
        return self.last_avoidance_side

    @staticmethod
    def _force_avoidance_side(
        risk: CollisionRisk, avoidance_side: str
    ) -> CollisionRisk:
        if risk.avoidance_side == avoidance_side:
            return risk
        center_x, center_y = risk.collision_point
        reflected_guards = tuple(
            (
                2.0 * center_x - point[0],
                2.0 * center_y - point[1],
            )
            for point in risk.guard_points[1:]
        )
        return replace(
            risk,
            avoidance_side=avoidance_side,
            guard_points=(
                risk.collision_point,
                *reflected_guards,
            ),
        )

    @staticmethod
    def _urgency(risk: CollisionRisk):
        return (
            risk.time_to_closest,
            risk.current_distance,
            risk.identifier,
        )

    def _find_event(self, risk: CollisionRisk):
        mapped_identifier = self.track_to_event.get(risk.identifier)
        if mapped_identifier in self.events:
            return mapped_identifier

        candidates = []
        for identifier, event in self.events.items():
            distance = _distance(
                risk.obstacle_position,
                event.obstacle_position,
            )
            if distance <= self.reassociation_distance:
                candidates.append((distance, identifier))
        return min(candidates)[1] if candidates else None

    def _new_event(self, risk: CollisionRisk) -> int:
        identifier = self.next_event_identifier
        self.next_event_identifier += 1
        self.events[identifier] = _LatchedCollisionEvent(
            identifier=identifier,
            risk=risk,
            obstacle_position=risk.obstacle_position,
            track_identifiers={risk.identifier},
        )
        self.last_avoidance_side = risk.avoidance_side
        return identifier

    @classmethod
    def _anchor_to_event(
        cls,
        risk: CollisionRisk,
        event: _LatchedCollisionEvent,
    ) -> CollisionRisk:
        risk = cls._force_avoidance_side(
            risk,
            event.risk.avoidance_side,
        )
        return replace(
            risk,
            collision_point=event.risk.collision_point,
            safe_point=event.risk.safe_point,
            guard_points=event.risk.guard_points,
        )

    def _safely_released(self, risk: CollisionRisk) -> bool:
        # Release only after measured separation is safe and the relative
        # velocity predicts no closer encounter.  This prevents a momentary
        # zero-speed estimate at a route endpoint from removing the detour.
        return (
            risk.current_distance >= self.release_distance
            and risk.closest_distance >= self.release_distance
        )

    def update(self, risks) -> tuple[CollisionRisk, ...]:
        observed_events = set()
        expired_events = set()
        for risk in sorted(
            risks,
            key=lambda risk: (not risk.threatening, *self._urgency(risk)),
        ):
            event_identifier = self._find_event(risk)
            if event_identifier is None:
                if not risk.threatening:
                    continue
                event_identifier = self._new_event(risk)
            event = self.events[event_identifier]
            event.track_identifiers.add(risk.identifier)
            self.track_to_event[risk.identifier] = event_identifier
            if event_identifier in observed_events:
                continue

            event.obstacle_position = risk.obstacle_position
            event.lost_observations = 0
            # Keep the original world-frame collision strip and chosen side,
            # but refresh velocity, distance and TTC for status/release logic.
            event.risk = replace(
                self._anchor_to_event(risk, event),
                threatening=True,
            )
            if risk.threatening:
                event.clear_observations = 0
            elif self._safely_released(risk):
                event.clear_observations += 1
                if event.clear_observations >= self.clear_confirmations:
                    expired_events.add(event_identifier)
            else:
                event.clear_observations = 0
            observed_events.add(event_identifier)

        for identifier, event in self.events.items():
            if identifier in observed_events:
                continue
            event.clear_observations = 0
            event.lost_observations += 1
            if event.lost_observations >= self.lost_confirmations:
                expired_events.add(identifier)

        for identifier in expired_events:
            self.events.pop(identifier, None)
        if expired_events:
            self.track_to_event = {
                track_identifier: event_identifier
                for track_identifier, event_identifier
                in self.track_to_event.items()
                if event_identifier in self.events
            }

        return tuple(
            self.events[identifier].risk
            for identifier in sorted(self.events)
        )


def _risk_zone(relative_body) -> str:
    x, y = (float(value) for value in relative_body)
    longitudinal = "front" if x >= 0.0 else "rear"
    center_width = max(0.25, 0.30 * abs(x))
    if abs(y) <= center_width:
        return longitudinal
    return f"{longitudinal}_{'left' if y > 0.0 else 'right'}"


def path_tangent(
    points,
    robot_position,
    lookahead_distance: float = 2.0,
) -> tuple[float, float] | None:
    """Return the forward map-frame tangent of a path near the robot."""
    path = tuple(
        (float(point[0]), float(point[1]))
        for point in points
        if len(point) >= 2
        and math.isfinite(float(point[0]))
        and math.isfinite(float(point[1]))
    )
    robot = tuple(float(value) for value in robot_position)
    if (
        len(path) < 2
        or len(robot) != 2
        or not all(math.isfinite(value) for value in robot)
        or not math.isfinite(float(lookahead_distance))
        or lookahead_distance <= 0.0
    ):
        return None

    nearest_index = min(
        range(len(path)),
        key=lambda index: _distance(path[index], robot),
    )
    origin = path[nearest_index]
    target = None
    for point in path[nearest_index + 1:]:
        if _distance(origin, point) >= float(lookahead_distance):
            target = point
            break
    if target is None and nearest_index + 1 < len(path):
        target = path[-1]
    if target is None or _distance(origin, target) <= 1.0e-6:
        return None
    length = _distance(origin, target)
    return (
        (target[0] - origin[0]) / length,
        (target[1] - origin[1]) / length,
    )


def calculate_collision_risk(
    track: TrackedObstacle,
    robot_pose,
    robot_velocity,
    robot_radius: float,
    default_obstacle_radius: float,
    safety_margin: float,
    response_time: float,
    braking_deceleration: float,
    lateral_maneuver_acceleration: float,
    collision_horizon: float,
    planning_buffer: float,
    minimum_closing_speed: float,
    nominal_robot_speed: float,
    collision_corridor_margin: float,
    proximity_guard_distance: float,
    guard_sample_spacing: float,
    maximum_guard_length: float,
    guard_side_offset: float,
    guard_tail_length: float = 3.0,
    planning_direction=None,
    preferred_avoidance_side=None,
) -> CollisionRisk:
    """Calculate closest approach and a speed-dependent avoidance distance."""
    robot_x, robot_y, robot_yaw = (
        float(value) for value in robot_pose
    )
    robot_vx, robot_vy = (float(value) for value in robot_velocity)
    limits = (
        robot_radius,
        default_obstacle_radius,
        safety_margin,
        response_time,
        braking_deceleration,
        lateral_maneuver_acceleration,
        collision_horizon,
        planning_buffer,
        minimum_closing_speed,
        nominal_robot_speed,
        collision_corridor_margin,
        proximity_guard_distance,
        guard_sample_spacing,
        maximum_guard_length,
        guard_side_offset,
        guard_tail_length,
    )
    if (
        not all(math.isfinite(float(value)) for value in (
            robot_x,
            robot_y,
            robot_yaw,
            robot_vx,
            robot_vy,
            *limits,
        ))
        or min(
            robot_radius,
            default_obstacle_radius,
            braking_deceleration,
            lateral_maneuver_acceleration,
            collision_horizon,
            nominal_robot_speed,
            proximity_guard_distance,
            guard_sample_spacing,
            maximum_guard_length,
            guard_side_offset,
        ) <= 0.0
        or min(
            safety_margin,
            response_time,
            planning_buffer,
            minimum_closing_speed,
            collision_corridor_margin,
            guard_tail_length,
        ) < 0.0
    ):
        raise ValueError("Collision-risk parameters must be finite and valid")

    relative = (track.x - robot_x, track.y - robot_y)
    relative_body = world_to_body((track.x, track.y), robot_pose)
    current_distance = math.hypot(*relative)
    relative_velocity = (
        track.vx - robot_vx,
        track.vy - robot_vy,
    )
    relative_speed_squared = (
        relative_velocity[0] ** 2 + relative_velocity[1] ** 2
    )
    radial_rate = (
        (
            relative[0] * relative_velocity[0]
            + relative[1] * relative_velocity[1]
        )
        / current_distance
        if current_distance > 1.0e-9
        else 0.0
    )
    closing_speed = max(0.0, -radial_rate)
    raw_time_to_closest = (
        -(
            relative[0] * relative_velocity[0]
            + relative[1] * relative_velocity[1]
        )
        / relative_speed_squared
        if relative_speed_squared > 1.0e-9
        else math.inf
    )
    evaluation_time = (
        min(max(0.0, raw_time_to_closest), collision_horizon)
        if math.isfinite(raw_time_to_closest)
        else collision_horizon
    )
    closest_vector = (
        relative[0] + relative_velocity[0] * evaluation_time,
        relative[1] + relative_velocity[1] * evaluation_time,
    )
    closest_distance = math.hypot(*closest_vector)

    obstacle_radius = max(
        float(default_obstacle_radius),
        0.5 * max(0.0, float(track.diameter)),
    )
    base_clearance = (
        float(robot_radius) + obstacle_radius + float(safety_margin)
    )
    robot_speed = math.hypot(robot_vx, robot_vy)
    requested_direction = (
        tuple(float(value) for value in planning_direction)
        if planning_direction is not None
        else ()
    )
    requested_length = (
        math.hypot(*requested_direction)
        if len(requested_direction) == 2
        and all(math.isfinite(value) for value in requested_direction)
        else 0.0
    )
    if requested_length > 1.0e-6:
        route_forward = (
            requested_direction[0] / requested_length,
            requested_direction[1] / requested_length,
        )
    elif robot_speed > 0.05:
        route_forward = (
            robot_vx / robot_speed,
            robot_vy / robot_speed,
        )
    else:
        route_forward = (math.cos(robot_yaw), math.sin(robot_yaw))
    route_left = (-route_forward[1], route_forward[0])
    braking_distance = (
        robot_speed * robot_speed / (2.0 * float(braking_deceleration))
    )
    # A differential-drive robot needs finite time to create lateral
    # separation. Model the lane change as a symmetric acceleration and
    # deceleration manoeuvre: d = a*t^2/4, so t = 2*sqrt(d/a).
    maneuver_time = 2.0 * math.sqrt(
        base_clearance / float(lateral_maneuver_acceleration)
    )
    safe_distance = (
        base_clearance
        + closing_speed * (float(response_time) + maneuver_time)
        + braking_distance
    )
    time_to_collision = time_to_collision_boundary(
        relative,
        relative_velocity,
        base_clearance,
        collision_horizon,
    )
    route_lateral = (
        relative[0] * route_left[0] + relative[1] * route_left[1]
    )
    relative_velocity_route_y = (
        relative_velocity[0] * route_left[0]
        + relative_velocity[1] * route_left[1]
    )
    collision_course = (
        closing_speed >= float(minimum_closing_speed)
        and 0.0 <= raw_time_to_closest <= float(collision_horizon)
        and closest_distance
        <= base_clearance + float(collision_corridor_margin)
    )
    velocity_threatening = (
        collision_course
        and current_distance <= safe_distance + float(planning_buffer)
    )
    # A confirmed moving obstacle can decelerate or reverse at a route endpoint
    # before constant-velocity TTC predicts the new conflict.  Use the route
    # tangent instead of the body heading so this protection remains active
    # while the differential-drive robot turns into its planned detour.  The
    # same test covers an object approaching from either the front or rear.
    proximity_threatening = (
        abs(route_lateral)
        <= base_clearance + float(collision_corridor_margin)
        and current_distance <= float(proximity_guard_distance)
        and radial_rate <= float(minimum_closing_speed)
    )
    threatening = velocity_threatening or proximity_threatening

    # A same-lane rear catch can enter the 6 m proximity guard before its TTC
    # falls inside the nominal collision horizon. Project that encounter by
    # relative motion as well; otherwise all virtual occupancy remains behind
    # the UGV and a replacement path is geometrically identical to the old one.
    prediction_time = 0.0
    if threatening and collision_course:
        prediction_time = evaluation_time
    elif (
        proximity_threatening
        and closing_speed >= float(minimum_closing_speed)
        and math.isfinite(raw_time_to_closest)
        and raw_time_to_closest > 0.0
        and float(track.speed) > 1.0e-6
    ):
        prediction_time = min(
            raw_time_to_closest,
            float(maximum_guard_length) / float(track.speed),
        )
    predicted_obstacle = (
        float(track.x) + float(track.vx) * prediction_time,
        float(track.y) + float(track.vy) * prediction_time,
    )
    sweep_delta = (
        predicted_obstacle[0] - float(track.x),
        predicted_obstacle[1] - float(track.y),
    )
    route_corridor_width = (
        base_clearance + float(collision_corridor_margin)
    )
    converging_to_route = (
        abs(route_lateral) >= 0.35
        and route_lateral * relative_velocity_route_y < -0.02
    )
    if (
        abs(route_lateral) <= route_corridor_width
        and not converging_to_route
    ):
        # Once a moving object is already inside the road corridor, a short
        # lateral velocity spike is normally scan-centroid noise during a UGV
        # turn or an obstacle endpoint reversal. Keep that collision strip on
        # the road tangent. A consistently measured diagonal approach is not
        # projected away: its lateral velocity must carry the swept strip
        # across the existing route so IsPathValid requests a new Nav2 path.
        longitudinal_sweep = (
            sweep_delta[0] * route_forward[0]
            + sweep_delta[1] * route_forward[1]
        )
        sweep_delta = (
            longitudinal_sweep * route_forward[0],
            longitudinal_sweep * route_forward[1],
        )
        predicted_obstacle = (
            float(track.x) + sweep_delta[0],
            float(track.y) + sweep_delta[1],
        )
    sweep_length = math.hypot(*sweep_delta)
    collision_length = min(sweep_length, float(maximum_guard_length))
    if sweep_length > 1.0e-9:
        sweep_direction = (
            sweep_delta[0] / sweep_length,
            sweep_delta[1] / sweep_length,
        )
    else:
        sweep_direction = (0.0, 0.0)
    collision_point = (
        float(track.x) + sweep_direction[0] * collision_length,
        float(track.y) + sweep_direction[1] * collision_length,
    )
    # Do not end the virtual barrier at the mathematical closest-approach
    # point. A route that immediately returns to the lane centre there cuts in
    # front of the still-moving obstacle. Extending the swept corridor beyond
    # the conflict keeps Nav2 on the selected passing side until both bodies
    # have cleared one another.
    guarded_length = min(
        sweep_length + float(guard_tail_length),
        float(maximum_guard_length),
    )
    if guarded_length <= 1.0e-9:
        center_guard_points = ((float(track.x), float(track.y)),)
    else:
        sample_count = max(
            1,
            int(math.ceil(guarded_length / float(guard_sample_spacing))),
        )
        center_guard_points = tuple(
            (
                float(track.x)
                + sweep_direction[0]
                * min(
                    index * float(guard_sample_spacing),
                    guarded_length,
                ),
                float(track.y)
                + sweep_direction[1]
                * min(
                    index * float(guard_sample_spacing),
                    guarded_length,
                ),
            )
            for index in range(sample_count + 1)
        )
    collision_relative = (
        collision_point[0] - robot_x,
        collision_point[1] - robot_y,
    )
    collision_route_lateral = (
        collision_relative[0] * route_left[0]
        + collision_relative[1] * route_left[1]
    )
    centered_conflict_width = max(0.50, 0.60 * base_clearance)
    if abs(collision_route_lateral) <= centered_conflict_width:
        # Use stable right-hand traffic for a central front or rear conflict.
        # The road/path tangent remains valid while the differential-drive
        # body rotates into the detour.
        hazard_side_sign = 1.0
    elif abs(collision_route_lateral) > 0.10:
        # A diagonal rear approach can cross the route before catching the UGV.
        # Choose the side opposite the predicted meeting point, not opposite the
        # obstacle's stale current side.
        hazard_side_sign = (
            1.0 if collision_route_lateral > 0.0 else -1.0
        )
    elif abs(relative_velocity_route_y) > 0.05:
        # At the centre line, block the side the obstacle arrived from.
        hazard_side_sign = (
            -1.0 if relative_velocity_route_y > 0.0 else 1.0
        )
    else:
        # Deterministic right-hand detour for a centred head-on conflict.
        hazard_side_sign = 1.0
    if (
        preferred_avoidance_side is not None
        and abs(collision_route_lateral) <= centered_conflict_width
    ):
        if preferred_avoidance_side not in ("left", "right"):
            raise ValueError("Preferred avoidance side must be left or right")
        # Static road clearance breaks a tie only for a centred conflict.  If
        # the predicted meeting point is lateral to the route, crossing its
        # trajectory to reach a nominally clearer shoulder would create a new
        # collision. In that case the meeting-point geometry above wins.
        hazard_side_sign = (
            1.0 if preferred_avoidance_side == "right" else -1.0
        )
    avoidance_side = "right" if hazard_side_sign > 0.0 else "left"
    lateral_left = route_left
    # The open side is represented explicitly as a safe planning point.  The
    # connected lethal strip occupies the opposite side, causing SmacPlanner
    # to update the global path through this free corridor instead of issuing
    # a scripted steering or reversing manoeuvre.
    effective_guard_side_offset = max(
        float(guard_side_offset),
        base_clearance + 0.25,
    )
    safe_side_sign = -hazard_side_sign
    safe_point = (
        collision_point[0]
        + safe_side_sign * effective_guard_side_offset * route_left[0],
        collision_point[1]
        + safe_side_sign * effective_guard_side_offset * route_left[1],
    )
    lateral_sample_count = max(
        1,
        int(math.ceil(
            effective_guard_side_offset / float(guard_sample_spacing)
        )),
    )
    lateral_offsets = tuple(
        effective_guard_side_offset * index / lateral_sample_count
        for index in range(lateral_sample_count + 1)
    )
    guard_points = tuple(
        (
            center[0]
            + hazard_side_sign * offset * lateral_left[0],
            center[1]
            + hazard_side_sign * offset * lateral_left[1],
        )
        for center in center_guard_points
        for offset in lateral_offsets
    )
    return CollisionRisk(
        identifier=int(track.identifier),
        zone=_risk_zone(relative_body),
        avoidance_side=avoidance_side,
        obstacle_position=(float(track.x), float(track.y)),
        obstacle_velocity=(float(track.vx), float(track.vy)),
        robot_velocity=(robot_vx, robot_vy),
        obstacle_speed=float(track.speed),
        closing_speed=closing_speed,
        current_distance=current_distance,
        safe_distance=safe_distance,
        maneuver_time=maneuver_time,
        time_to_collision=time_to_collision,
        time_to_closest=raw_time_to_closest,
        closest_distance=closest_distance,
        collision_point=collision_point,
        safe_point=safe_point,
        # A connected strip from the collision corridor to the hazard-side
        # road edge prevents the global planner taking that strip's outer edge
        # and leaves only the selected passing side open.
        guard_points=guard_points,
        threatening=threatening,
    )


def _cluster_from_points(points) -> ScanCluster:
    x = sum(point[0] for point in points) / len(points)
    y = sum(point[1] for point in points) / len(points)
    diameter = max(
        (_distance(first, second) for first in points for second in points),
        default=0.0,
    )
    return ScanCluster(x=x, y=y, diameter=diameter, samples=len(points))


def scan_clusters(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    cluster_gap: float,
    minimum_samples: int,
    maximum_diameter: float,
) -> tuple[ScanCluster, ...]:
    """Extract compact obstacle clusters from a full-circle laser scan."""
    points = []
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if (
            not math.isfinite(distance)
            or distance < float(range_min)
            or distance > float(range_max)
        ):
            points.append(None)
            continue
        angle = float(angle_min) + index * float(angle_increment)
        points.append((
            distance * math.cos(angle),
            distance * math.sin(angle),
        ))

    groups = []
    current = []
    for point in points:
        if point is None:
            if current:
                groups.append(current)
                current = []
            continue
        if current and _distance(current[-1], point) > float(cluster_gap):
            groups.append(current)
            current = []
        current.append(point)
    if current:
        groups.append(current)

    if (
        len(groups) >= 2
        and points
        and points[0] is not None
        and points[-1] is not None
        and _distance(groups[-1][-1], groups[0][0]) <= float(cluster_gap)
    ):
        groups[0] = groups[-1] + groups[0]
        groups.pop()

    clusters = []
    for group in groups:
        if len(group) < int(minimum_samples):
            continue
        cluster = _cluster_from_points(group)
        if cluster.diameter <= float(maximum_diameter):
            clusters.append(cluster)
    return tuple(clusters)


def body_to_world(point, pose) -> tuple[float, float]:
    x, y = (float(value) for value in point)
    origin_x, origin_y, yaw = (float(value) for value in pose)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        origin_x + cosine * x - sine * y,
        origin_y + sine * x + cosine * y,
    )


def world_to_body(point, pose) -> tuple[float, float]:
    x, y = (float(value) for value in point)
    origin_x, origin_y, yaw = (float(value) for value in pose)
    delta_x = x - origin_x
    delta_y = y - origin_y
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
    )


def forward_escape_guard_points(
    risk: CollisionRisk,
    robot_pose,
    exclusion_radius: float,
    risk_radius: float,
    minimum_forward_distance: float,
    guard_side_offset: float,
    activation_margin: float,
    safe_side_intrusion: float = 0.0,
) -> tuple[tuple[float, float], ...]:
    """Keep a near collision event ahead of Nav2's planning start."""
    values = (
        exclusion_radius,
        risk_radius,
        minimum_forward_distance,
        guard_side_offset,
        activation_margin,
        safe_side_intrusion,
    )
    if (
        not all(math.isfinite(float(value)) for value in values)
        or min(
            exclusion_radius,
            risk_radius,
            minimum_forward_distance,
            guard_side_offset,
        ) <= 0.0
        or min(activation_margin, safe_side_intrusion) < 0.0
    ):
        raise ValueError("Forward escape guard parameters must be valid")

    # A rear catch-up is detected far enough ahead by the map-frame swept
    # corridor.  Moving a second virtual gate in front of the UGV makes every
    # controller step invalidate its own path and can stop the vehicle where
    # the faster object will catch it.
    if risk.zone.startswith("rear"):
        return ()

    # The map-frame collision corridor already handles a distant future
    # conflict.  This body-frame gate is only a near-field fallback for the
    # measured obstacle; activating it from a predicted collision point makes
    # it move with every body rotation and repeatedly invalidates a safe path.
    collision_body = world_to_body(
        risk.obstacle_position,
        robot_pose,
    )
    collision_surface_distance = (
        math.hypot(*collision_body) - float(risk_radius)
    )
    if collision_surface_distance > (
        float(exclusion_radius) + float(activation_margin)
    ):
        return ()

    # Unlike radial clipping, this creates two points only in front of the
    # robot. The first blocks the collision lane and the second preserves the
    # event's latched passing side without surrounding the planning start.
    forward = max(float(minimum_forward_distance), collision_body[0])
    hazard_sign = 1.0 if risk.avoidance_side == "right" else -1.0
    center = (forward, collision_body[1])
    points = [
        center,
        (
            center[0],
            center[1] + hazard_sign * float(guard_side_offset),
        ),
    ]
    if safe_side_intrusion > 0.0:
        points.append((
            center[0],
            center[1] - hazard_sign * float(safe_side_intrusion),
        ))
    return tuple(points)


def compose_pose(base_pose, offset_pose) -> tuple[float, float, float]:
    """Apply a fixed sensor offset to a map-frame base pose."""
    base_x, base_y, base_yaw = (float(value) for value in base_pose)
    offset_x, offset_y, offset_yaw = (
        float(value) for value in offset_pose
    )
    sensor_x, sensor_y = body_to_world(
        (offset_x, offset_y),
        (base_x, base_y, base_yaw),
    )
    return sensor_x, sensor_y, base_yaw + offset_yaw


def _fit_velocity(history) -> tuple[float, float, float]:
    if len(history) < 2:
        return 0.0, 0.0, math.inf
    origin = history[0][0]
    times = [sample[0] - origin for sample in history]
    mean_time = sum(times) / len(times)
    mean_x = sum(sample[1] for sample in history) / len(history)
    mean_y = sum(sample[2] for sample in history) / len(history)
    denominator = sum((value - mean_time) ** 2 for value in times)
    if denominator <= 1.0e-9:
        return 0.0, 0.0, math.inf
    vx = sum(
        (sample_time - mean_time) * (sample[1] - mean_x)
        for sample_time, sample in zip(times, history)
    ) / denominator
    vy = sum(
        (sample_time - mean_time) * (sample[2] - mean_y)
        for sample_time, sample in zip(times, history)
    ) / denominator
    residual = math.sqrt(sum(
        (
            sample[1] - (mean_x + vx * (sample_time - mean_time))
        ) ** 2
        + (
            sample[2] - (mean_y + vy * (sample_time - mean_time))
        ) ** 2
        for sample_time, sample in zip(times, history)
    ) / len(history))
    return vx, vy, residual


class DynamicObstacleTracker:
    """Associate compact scan clusters and estimate map-frame velocities."""

    def __init__(
        self,
        association_distance: float = 0.75,
        velocity_alpha: float = 0.35,
        track_timeout: float = 0.8,
        maximum_track_speed: float = 1.8,
        position_gate: float = 0.18,
        diameter_tolerance: float = 0.55,
        history_window: float = 1.4,
        minimum_motion_duration: float = 0.35,
        minimum_displacement: float = 0.12,
        minimum_direction_consistency: float = 0.65,
        maximum_fit_residual: float = 0.14,
        maximum_prediction_age: float = 0.25,
        reversal_minimum_speed: float = 0.08,
        reversal_cosine_threshold: float = -0.25,
    ) -> None:
        self.association_distance = float(association_distance)
        self.velocity_alpha = float(velocity_alpha)
        self.track_timeout = float(track_timeout)
        self.maximum_track_speed = float(maximum_track_speed)
        self.position_gate = float(position_gate)
        self.diameter_tolerance = float(diameter_tolerance)
        self.history_window = float(history_window)
        self.minimum_motion_duration = float(minimum_motion_duration)
        self.minimum_displacement = float(minimum_displacement)
        self.minimum_direction_consistency = float(
            minimum_direction_consistency
        )
        self.maximum_fit_residual = float(maximum_fit_residual)
        self.maximum_prediction_age = float(maximum_prediction_age)
        self.reversal_minimum_speed = float(reversal_minimum_speed)
        self.reversal_cosine_threshold = float(reversal_cosine_threshold)
        self.tracks = {}
        self.next_identifier = 1
        self.last_timestamp = None

    def reset(self) -> None:
        self.tracks.clear()
        self.last_timestamp = None

    def update(
        self, detections, timestamp: float
    ) -> tuple[TrackedObstacle, ...]:
        now = float(timestamp)
        if (
            self.last_timestamp is not None
            and now < self.last_timestamp - 1.0e-6
        ):
            self.tracks.clear()
        self.last_timestamp = now
        parsed_detections = []
        for detection in detections:
            diameter = float(detection[2]) if len(detection) >= 3 else 0.0
            parsed_detections.append((
                float(detection[0]),
                float(detection[1]),
                diameter,
            ))
        detections = parsed_detections
        self.tracks = {
            identifier: track
            for identifier, track in self.tracks.items()
            if now - track.updated_at <= self.track_timeout
        }

        available_tracks = set(self.tracks)
        available_detections = set(range(len(detections)))
        candidates = []
        for identifier, track in self.tracks.items():
            elapsed = max(0.0, now - track.updated_at)
            predicted = (
                track.x + track.vx * elapsed,
                track.y + track.vy * elapsed,
            )
            for index, detection in enumerate(detections):
                detection_xy = detection[:2]
                travel = _distance((track.x, track.y), detection_xy)
                travel_gate = min(
                    self.association_distance,
                    self.position_gate + self.maximum_track_speed * elapsed,
                )
                if travel > travel_gate:
                    continue
                if (
                    track.diameter > 0.0
                    and detection[2] > 0.0
                    and abs(track.diameter - detection[2])
                    > self.diameter_tolerance
                ):
                    continue
                distance = _distance(predicted, detection_xy)
                if distance <= self.association_distance:
                    diameter_error = abs(track.diameter - detection[2])
                    candidates.append((
                        distance + 0.15 * diameter_error,
                        identifier,
                        index,
                    ))

        for _, identifier, index in sorted(candidates):
            if (
                identifier not in available_tracks
                or index not in available_detections
            ):
                continue
            track = self.tracks[identifier]
            detection_x, detection_y, detection_diameter = detections[index]
            elapsed = now - track.updated_at
            if elapsed > 1.0e-3:
                instantaneous_vx = (detection_x - track.x) / elapsed
                instantaneous_vy = (detection_y - track.y) / elapsed
                instantaneous_speed = math.hypot(
                    instantaneous_vx, instantaneous_vy
                )
                previous_speed = track.speed
                direction_cosine = (
                    (
                        track.vx * instantaneous_vx
                        + track.vy * instantaneous_vy
                    ) / (previous_speed * instantaneous_speed)
                    if min(previous_speed, instantaneous_speed) > 1.0e-9
                    else 1.0
                )
                reversed_direction = (
                    track.motion_confirmed
                    and previous_speed >= self.reversal_minimum_speed
                    and instantaneous_speed >= self.reversal_minimum_speed
                    and direction_cosine <= self.reversal_cosine_threshold
                )
            else:
                instantaneous_vx = 0.0
                instantaneous_vy = 0.0
                reversed_direction = False

            if reversed_direction:
                # Retaining pre-turnaround samples makes the fitted velocity
                # point in the wrong direction for roughly one second. Start a
                # new segment while preserving the already confirmed identity.
                track.history = [
                    (track.updated_at, track.x, track.y),
                    (now, detection_x, detection_y),
                ]
                track.vx = instantaneous_vx
                track.vy = instantaneous_vy
                track.fit_residual = 0.0
            else:
                track.history.append((now, detection_x, detection_y))
                oldest = now - self.history_window
                while (
                    len(track.history) > 2
                    and track.history[1][0] < oldest
                ):
                    track.history.pop(0)
                fitted_vx, fitted_vy, residual = _fit_velocity(track.history)
                if elapsed > 1.0e-3 and math.isfinite(residual):
                    alpha = self.velocity_alpha
                    track.vx = alpha * fitted_vx + (1.0 - alpha) * track.vx
                    track.vy = alpha * fitted_vy + (1.0 - alpha) * track.vy
                    track.fit_residual = residual
            track.x = detection_x
            track.y = detection_y
            if detection_diameter > 0.0:
                track.diameter = (
                    0.35 * detection_diameter + 0.65 * track.diameter
                    if track.diameter > 0.0
                    else detection_diameter
                )
            track.updated_at = now
            track.observations += 1
            if self._has_consistent_motion_history(track):
                track.motion_confirmed = True
            available_tracks.remove(identifier)
            available_detections.remove(index)

        for index in sorted(available_detections):
            x, y, diameter = detections[index]
            identifier = self.next_identifier
            self.next_identifier += 1
            self.tracks[identifier] = TrackedObstacle(
                identifier=identifier,
                x=x,
                y=y,
                vx=0.0,
                vy=0.0,
                observations=1,
                updated_at=now,
                diameter=diameter,
                history=[(now, x, y)],
            )

        return tuple(self.tracks.values())

    def _has_consistent_motion_history(self, track) -> bool:
        return (
            track.motion_duration >= self.minimum_motion_duration
            and track.net_displacement >= self.minimum_displacement
            and track.direction_consistency
            >= self.minimum_direction_consistency
            and track.fit_residual <= self.maximum_fit_residual
        )

    def dynamic_tracks(
        self,
        minimum_speed: float,
        maximum_speed: float,
        minimum_observations: int,
    ) -> tuple[TrackedObstacle, ...]:
        return tuple(
            track
            for track in self.tracks.values()
            if (
                track.observations >= int(minimum_observations)
                and self.last_timestamp is not None
                and self.last_timestamp - track.updated_at
                <= self.maximum_prediction_age
                and float(minimum_speed) <= track.speed <= float(maximum_speed)
                and (
                    track.motion_confirmed
                    or self._has_consistent_motion_history(track)
                )
            )
        )

    def confirmed_tracks(
        self,
        maximum_speed: float,
        minimum_observations: int,
    ) -> tuple[TrackedObstacle, ...]:
        """Keep a motion-confirmed target through endpoint deceleration."""
        maximum_speed = float(maximum_speed)
        minimum_observations = int(minimum_observations)
        if (
            not math.isfinite(maximum_speed)
            or maximum_speed <= 0.0
            or minimum_observations <= 0
        ):
            raise ValueError("Confirmed-track limits must be positive")
        return tuple(
            track
            for track in self.tracks.values()
            if (
                track.observations >= minimum_observations
                and self.last_timestamp is not None
                and self.last_timestamp - track.updated_at
                <= self.maximum_prediction_age
                and track.speed <= maximum_speed
                and (
                    track.motion_confirmed
                    or self._has_consistent_motion_history(track)
                )
            )
        )


def predicted_swept_points(
    tracks,
    horizon: float,
    time_step: float,
    maximum_distance: float = math.inf,
) -> tuple[tuple[float, float], ...]:
    """Return current centers plus a short, distance-limited motion lead."""
    horizon = float(horizon)
    step = float(time_step)
    distance_limit = float(maximum_distance)
    if (
        horizon <= 0.0
        or step <= 0.0
        or distance_limit <= 0.0
        or math.isnan(distance_limit)
    ):
        raise ValueError(
            "Prediction horizon, time step and distance limit must be positive"
        )
    points = []
    for track in tracks:
        current = (float(track.x), float(track.y))
        points.append(current)
        speed = float(track.speed)
        effective_horizon = horizon
        if math.isfinite(distance_limit) and speed > 1.0e-9:
            effective_horizon = min(horizon, distance_limit / speed)
        sample_count = max(1, int(math.ceil(effective_horizon / step)))
        for sample in range(1, sample_count + 1):
            offset = min(sample * step, effective_horizon)
            points.append((
                current[0] + float(track.vx) * offset,
                current[1] + float(track.vy) * offset,
            ))
    return tuple(points)


def project_discs_to_scan(
    centers,
    radius: float,
    angle_min: float,
    angle_increment: float,
    sample_count: int,
    range_min: float,
    range_max: float,
    exclusion_radius: float,
) -> list[float]:
    """Rasterize predicted obstacle discs into a clearing LaserScan."""
    radius = float(radius)
    minimum = float(range_min)
    maximum = float(range_max)
    exclusion = float(exclusion_radius)
    if radius <= 0.0 or sample_count <= 0 or angle_increment <= 0.0:
        raise ValueError("Prediction scan geometry is invalid")
    ranges = [math.inf] * int(sample_count)
    for center in centers:
        center_x, center_y = (float(value) for value in center)
        center_distance = math.hypot(center_x, center_y)
        if (
            center_distance - radius <= exclusion
            or center_distance - radius > maximum
        ):
            continue
        for index in range(int(sample_count)):
            angle = float(angle_min) + index * float(angle_increment)
            cosine = math.cos(angle)
            sine = math.sin(angle)
            along = center_x * cosine + center_y * sine
            if along <= 0.0:
                continue
            lateral = -center_x * sine + center_y * cosine
            if abs(lateral) > radius:
                continue
            chord = math.sqrt(
                max(0.0, radius * radius - lateral * lateral)
            )
            near_hit = along - chord
            far_hit = along + chord
            if far_hit <= exclusion:
                continue
            # A future disc intersecting the robot envelope was rejected above.
            # Clipping it to the envelope creates an artificial obstacle ring
            # around the planning start and leaves Nav2 with no escape path.
            hit = near_hit
            if minimum <= hit <= maximum:
                ranges[index] = min(ranges[index], hit)
    return ranges


def prediction_cloud_points(
    centers,
    radius: float,
    exclusion_radius: float,
    perimeter_samples: int,
) -> tuple[tuple[float, float, float], ...]:
    """Expand every visible guard center into a non-occluding point cloud."""
    radius = float(radius)
    exclusion = float(exclusion_radius)
    samples = int(perimeter_samples)
    if (
        not math.isfinite(radius)
        or not math.isfinite(exclusion)
        or radius <= 0.0
        or exclusion < 0.0
        or samples < 4
    ):
        raise ValueError("Prediction cloud geometry is invalid")

    points = []
    for center in centers:
        center_x, center_y = (float(value) for value in center)
        if math.hypot(center_x, center_y) - radius <= exclusion:
            continue
        points.append((center_x, center_y, 0.20))
        points.extend(
            (
                center_x + radius * math.cos(2.0 * math.pi * index / samples),
                center_y + radius * math.sin(2.0 * math.pi * index / samples),
                0.20,
            )
            for index in range(samples)
        )
    return tuple(points)
