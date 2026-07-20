from dataclasses import dataclass, field
import math

from uav_control.battery_model import (
    MissionEnergySegment,
    MissionPowerProfile,
)
from uav_navigation.waypoint_navigator import Waypoint, WaypointMap

from uav_application.mission_states import uses_local_delivery_profile


@dataclass(frozen=True)
class MotionDurations:
    acceleration_seconds: float = 0.0
    cruise_seconds: float = 0.0
    deceleration_seconds: float = 0.0
    peak_speed_mps: float = 0.0


@dataclass
class _ProfileBuilder:
    segments: list[MissionEnergySegment] = field(default_factory=list)
    ascent_acceleration_seconds: float = 0.0
    climb_seconds: float = 0.0
    ascent_deceleration_seconds: float = 0.0
    horizontal_acceleration_seconds: float = 0.0
    cruise_seconds: float = 0.0
    horizontal_deceleration_seconds: float = 0.0
    hover_seconds: float = 0.0
    descent_acceleration_seconds: float = 0.0
    descent_seconds: float = 0.0
    descent_deceleration_seconds: float = 0.0
    horizontal_distance_m: float = 0.0
    ascent_m: float = 0.0
    descent_m: float = 0.0
    initial_payload_mass_kg: float = 0.0

    def add_segment(self, segment: MissionEnergySegment) -> None:
        if segment.duration_seconds > 0.0:
            self.segments.append(segment)

    def freeze(self) -> MissionPowerProfile:
        values = {
            name: value
            for name, value in self.__dict__.items()
            if name != "segments"
        }
        return MissionPowerProfile(
            segments=tuple(self.segments),
            **values,
        )


class MissionEnergyPlanner:
    """Build a payload-aware, acceleration-resolved safe-return profile."""

    def __init__(self, waypoint_map: WaypointMap, battery_parameters) -> None:
        self.waypoint_map = waypoint_map
        self.parameters = battery_parameters

    @staticmethod
    def _distance(first: Waypoint, second: Waypoint) -> float:
        return math.hypot(second.x - first.x, second.y - first.y)

    @staticmethod
    def motion_durations(
        distance_m: float,
        maximum_speed_mps: float,
        acceleration_mps2: float,
    ) -> MotionDurations:
        """Return a trapezoidal, or short-leg triangular, speed profile."""
        distance = max(0.0, float(distance_m))
        speed = float(maximum_speed_mps)
        acceleration = float(acceleration_mps2)
        if speed <= 0.0 or acceleration <= 0.0:
            raise ValueError("Motion speed and acceleration must be positive")
        if distance <= 0.0:
            return MotionDurations()
        ramp_distance = speed ** 2 / (2.0 * acceleration)
        if distance >= 2.0 * ramp_distance:
            ramp_time = speed / acceleration
            cruise_distance = distance - 2.0 * ramp_distance
            return MotionDurations(
                acceleration_seconds=ramp_time,
                cruise_seconds=cruise_distance / speed,
                deceleration_seconds=ramp_time,
                peak_speed_mps=speed,
            )
        peak_speed = math.sqrt(distance * acceleration)
        ramp_time = peak_speed / acceleration
        return MotionDurations(
            acceleration_seconds=ramp_time,
            deceleration_seconds=ramp_time,
            peak_speed_mps=peak_speed,
        )

    def resolve_payload_masses(
        self,
        targets: list[Waypoint],
        payload_masses_kg=None,
    ) -> list[float]:
        requested = list(payload_masses_kg or [])
        if requested and len(requested) != len(targets):
            raise ValueError(
                "payload_masses_kg must be empty or match the target count"
            )
        if requested:
            masses = [float(value) for value in requested]
        else:
            masses = [
                self.waypoint_map.payload_mass_for(
                    target,
                    self.parameters.default_payload_mass_kg,
                )
                for target in targets
            ]
        if not all(
            math.isfinite(value) and value >= 0.0 for value in masses
        ):
            raise ValueError("Payload masses must be finite and non-negative")
        total = sum(masses)
        if total > self.parameters.maximum_payload_mass_kg + 1e-9:
            raise ValueError(
                f"Total payload {total:.3f} kg exceeds UAV limit "
                f"{self.parameters.maximum_payload_mass_kg:.3f} kg"
            )
        return masses

    def _route_distances(self, start: str, destination: str) -> list[float]:
        route = self.waypoint_map.plan_route(start, destination)
        current = self.waypoint_map.corridor_nodes[start]
        distances = []
        for node in route:
            distances.append(self._distance(current, node))
            current = node
        return distances

    def _add_horizontal(
        self,
        builder: _ProfileBuilder,
        distance: float,
        payload_mass_kg: float,
    ) -> None:
        if distance <= 0.0:
            return
        motion = self.motion_durations(
            distance,
            self.parameters.estimated_horizontal_speed_mps,
            self.parameters.estimated_horizontal_acceleration_mps2,
        )
        peak = motion.peak_speed_mps
        builder.horizontal_distance_m += distance
        builder.horizontal_acceleration_seconds += (
            motion.acceleration_seconds
        )
        builder.cruise_seconds += motion.cruise_seconds
        builder.horizontal_deceleration_seconds += (
            motion.deceleration_seconds
        )
        builder.add_segment(MissionEnergySegment(
            phase="horizontal_acceleration",
            duration_seconds=motion.acceleration_seconds,
            horizontal_speed_start_mps=0.0,
            horizontal_speed_end_mps=peak,
            payload_mass_kg=payload_mass_kg,
        ))
        builder.add_segment(MissionEnergySegment(
            phase="cruise",
            duration_seconds=motion.cruise_seconds,
            horizontal_speed_start_mps=peak,
            horizontal_speed_end_mps=peak,
            payload_mass_kg=payload_mass_kg,
        ))
        builder.add_segment(MissionEnergySegment(
            phase="horizontal_deceleration",
            duration_seconds=motion.deceleration_seconds,
            horizontal_speed_start_mps=peak,
            horizontal_speed_end_mps=0.0,
            payload_mass_kg=payload_mass_kg,
        ))

    def _add_route(
        self,
        builder: _ProfileBuilder,
        start: str,
        destination: str,
        payload_mass_kg: float,
    ) -> None:
        for distance in self._route_distances(start, destination):
            self._add_horizontal(builder, distance, payload_mass_kg)

    def _add_vertical(
        self,
        builder: _ProfileBuilder,
        start_altitude: float,
        end_altitude: float,
        payload_mass_kg: float,
    ) -> None:
        delta = end_altitude - start_altitude
        distance = abs(delta)
        if distance <= 0.0:
            return
        motion = self.motion_durations(
            distance,
            self.parameters.estimated_vertical_speed_mps,
            self.parameters.estimated_vertical_acceleration_mps2,
        )
        peak = motion.peak_speed_mps
        if delta > 0.0:
            builder.ascent_m += distance
            builder.ascent_acceleration_seconds += (
                motion.acceleration_seconds
            )
            builder.climb_seconds += motion.cruise_seconds
            builder.ascent_deceleration_seconds += (
                motion.deceleration_seconds
            )
            builder.add_segment(MissionEnergySegment(
                phase="ascent_acceleration",
                duration_seconds=motion.acceleration_seconds,
                vertical_speed_start_mps=0.0,
                vertical_speed_end_mps=peak,
                payload_mass_kg=payload_mass_kg,
            ))
            builder.add_segment(MissionEnergySegment(
                phase="climb",
                duration_seconds=motion.cruise_seconds,
                vertical_speed_start_mps=peak,
                vertical_speed_end_mps=peak,
                payload_mass_kg=payload_mass_kg,
            ))
            builder.add_segment(MissionEnergySegment(
                phase="ascent_deceleration",
                duration_seconds=motion.deceleration_seconds,
                vertical_speed_start_mps=peak,
                vertical_speed_end_mps=0.0,
                payload_mass_kg=payload_mass_kg,
            ))
            return

        builder.descent_m += distance
        builder.descent_acceleration_seconds += motion.acceleration_seconds
        builder.descent_seconds += motion.cruise_seconds
        builder.descent_deceleration_seconds += motion.deceleration_seconds
        builder.add_segment(MissionEnergySegment(
            phase="descent_acceleration",
            duration_seconds=motion.acceleration_seconds,
            vertical_speed_start_mps=0.0,
            vertical_speed_end_mps=-peak,
            payload_mass_kg=payload_mass_kg,
        ))
        builder.add_segment(MissionEnergySegment(
            phase="descent",
            duration_seconds=motion.cruise_seconds,
            vertical_speed_start_mps=-peak,
            vertical_speed_end_mps=-peak,
            payload_mass_kg=payload_mass_kg,
        ))
        builder.add_segment(MissionEnergySegment(
            phase="descent_deceleration",
            duration_seconds=motion.deceleration_seconds,
            vertical_speed_start_mps=-peak,
            vertical_speed_end_mps=0.0,
            payload_mass_kg=payload_mass_kg,
        ))

    @staticmethod
    def _add_hover(
        builder: _ProfileBuilder,
        duration_seconds: float,
        payload_mass_kg: float,
    ) -> None:
        duration = float(duration_seconds)
        builder.hover_seconds += duration
        builder.add_segment(MissionEnergySegment(
            phase="hover",
            duration_seconds=duration,
            payload_mass_kg=payload_mass_kg,
        ))

    def plan(
        self,
        target_names,
        home_name: str,
        landing_height: float,
        return_home: bool = True,
        payload_masses_kg=None,
        target_floors=None,
    ) -> MissionPowerProfile:
        targets = self.waypoint_map.resolve_delivery_targets(
            target_names, target_floors
        )
        if not targets:
            raise ValueError("At least one UAV target is required")
        payload_masses = self.resolve_payload_masses(
            targets, payload_masses_kg
        )
        home = self.waypoint_map.resolve_home(home_name)
        settings = self.waypoint_map.flight
        local_delivery = uses_local_delivery_profile(home_name)
        delivery_altitudes = [
            self.waypoint_map.delivery_altitude_for(target)
            for target in targets
        ]
        takeoff_altitude = (
            delivery_altitudes[0]
            if local_delivery
            else float(settings["takeoff_altitude"])
        )
        current_payload = sum(payload_masses)
        builder = _ProfileBuilder(initial_payload_mass_kg=current_payload)
        self._add_vertical(
            builder,
            float(landing_height),
            takeoff_altitude,
            current_payload,
        )
        self._add_hover(
            builder,
            float(settings["takeoff_hover_duration"]),
            current_payload,
        )

        route_position = home.name
        current_altitude = takeoff_altitude
        cruise_altitude = float(settings["cruise_altitude"])
        if not local_delivery:
            self._add_vertical(
                builder,
                current_altitude,
                cruise_altitude,
                current_payload,
            )
            current_altitude = cruise_altitude

        for index, target in enumerate(targets):
            delivery_altitude = delivery_altitudes[index]
            if local_delivery:
                current_node = self.waypoint_map.corridor_nodes[
                    route_position
                ]
                self._add_horizontal(
                    builder,
                    self._distance(current_node, target),
                    current_payload,
                )
                self._add_vertical(
                    builder,
                    current_altitude,
                    delivery_altitude,
                    current_payload,
                )
                current_altitude = delivery_altitude
            else:
                self._add_route(
                    builder,
                    route_position,
                    target.name,
                    current_payload,
                )
                self._add_vertical(
                    builder,
                    current_altitude,
                    delivery_altitude,
                    current_payload,
                )
                current_altitude = delivery_altitude

            self._add_hover(
                builder,
                float(settings["delivery_hover_duration"]),
                current_payload,
            )
            current_payload = max(
                0.0, current_payload - payload_masses[index]
            )
            route_position = target.name

            # Admission always reserves a safe return, even when the request
            # asks the aircraft to remain at the destination.
            if not local_delivery:
                self._add_vertical(
                    builder,
                    current_altitude,
                    cruise_altitude,
                    current_payload,
                )
                current_altitude = cruise_altitude

        if local_delivery:
            current_node = self.waypoint_map.corridor_nodes[route_position]
            self._add_horizontal(
                builder,
                self._distance(current_node, home),
                current_payload,
            )
        else:
            self._add_route(
                builder,
                route_position,
                home.name,
                current_payload,
            )

        approach_altitude = float(settings["landing_approach_altitude"])
        self._add_vertical(
            builder,
            current_altitude,
            approach_altitude,
            current_payload,
        )
        self._add_vertical(
            builder,
            approach_altitude,
            float(landing_height),
            current_payload,
        )
        _ = return_home
        return builder.freeze()
