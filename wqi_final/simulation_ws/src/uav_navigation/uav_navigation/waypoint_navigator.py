from dataclasses import dataclass, replace
import heapq
import math
from pathlib import Path

import yaml


class WaypointConfigurationError(ValueError):
    pass


@dataclass(frozen=True)
class Waypoint:
    name: str
    label: str
    x: float
    y: float
    yaw: float
    delivery_altitude: float | None = None
    delivery_floor: int | None = None
    maximum_floor: int | None = None
    payload_mass_kg: float | None = None


class WaypointMap:
    def __init__(self, config_path) -> None:
        self.path = Path(config_path)
        with self.path.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict):
            raise WaypointConfigurationError("Waypoint file must contain a mapping")
        self.flight = data.get("flight", {})
        raw_waypoints = data.get("waypoints", {})
        if not raw_waypoints:
            raise WaypointConfigurationError("No UAV waypoints are configured")

        self.waypoints = {}
        for name, raw in raw_waypoints.items():
            try:
                raw_floor = raw.get("delivery_floor")
                delivery_floor = None
                if raw_floor is not None:
                    numeric_floor = float(raw_floor)
                    if not numeric_floor.is_integer():
                        raise ValueError("delivery_floor must be an integer")
                    delivery_floor = int(numeric_floor)
                raw_altitude = raw.get("delivery_altitude")
                raw_maximum_floor = raw.get("maximum_floor")
                maximum_floor = None
                if raw_maximum_floor is not None:
                    numeric_maximum_floor = float(raw_maximum_floor)
                    if not numeric_maximum_floor.is_integer():
                        raise ValueError("maximum_floor must be an integer")
                    maximum_floor = int(numeric_maximum_floor)
                raw_payload = raw.get("payload_mass_kg")
                waypoint = Waypoint(
                    name=name,
                    label=str(raw.get("label", name)),
                    x=float(raw["x"]),
                    y=float(raw["y"]),
                    yaw=float(raw.get("yaw", 0.0)),
                    delivery_altitude=(
                        None if raw_altitude is None else float(raw_altitude)
                    ),
                    delivery_floor=delivery_floor,
                    maximum_floor=maximum_floor,
                    payload_mass_kg=(
                        None if raw_payload is None else float(raw_payload)
                    ),
                )
            except (KeyError, TypeError, ValueError) as error:
                raise WaypointConfigurationError(
                    f"Invalid waypoint '{name}': {error}"
                ) from error
            values = [waypoint.x, waypoint.y, waypoint.yaw]
            if waypoint.delivery_altitude is not None:
                values.append(waypoint.delivery_altitude)
            if waypoint.payload_mass_kg is not None:
                values.append(waypoint.payload_mass_kg)
            if not all(math.isfinite(value) for value in values):
                raise WaypointConfigurationError(f"Waypoint '{name}' is not finite")
            if waypoint.delivery_floor is not None and waypoint.delivery_floor <= 0:
                raise WaypointConfigurationError(
                    f"Waypoint '{name}' delivery floor must be positive"
                )
            if waypoint.maximum_floor is not None and waypoint.maximum_floor <= 0:
                raise WaypointConfigurationError(
                    f"Waypoint '{name}' maximum floor must be positive"
                )
            if (
                waypoint.delivery_floor is not None
                and waypoint.maximum_floor is not None
                and waypoint.delivery_floor > waypoint.maximum_floor
            ):
                raise WaypointConfigurationError(
                    f"Waypoint '{name}' default floor exceeds its maximum floor"
                )
            if (
                waypoint.payload_mass_kg is not None
                and waypoint.payload_mass_kg < 0.0
            ):
                raise WaypointConfigurationError(
                    f"Waypoint '{name}' payload mass cannot be negative"
                )
            self.waypoints[name] = waypoint
        self.corridor_nodes = self._load_corridor_nodes(
            data.get("corridor_nodes", {})
        )
        self.corridor_edges = self._load_corridor_edges(
            data.get("corridor_edges", [])
        )
        self._route_distance_cache = {}
        self._validate_flight_settings()
        self._validate_corridors()

    @staticmethod
    def _parse_waypoint(name: str, raw, kind: str) -> Waypoint:
        try:
            waypoint = Waypoint(
                name=name,
                label=str(raw.get("label", name)),
                x=float(raw["x"]),
                y=float(raw["y"]),
                yaw=float(raw.get("yaw", 0.0)),
            )
        except (AttributeError, KeyError, TypeError, ValueError) as error:
            raise WaypointConfigurationError(
                f"Invalid {kind} '{name}': {error}"
            ) from error
        values = (waypoint.x, waypoint.y, waypoint.yaw)
        if not all(math.isfinite(value) for value in values):
            raise WaypointConfigurationError(f"{kind.title()} '{name}' is not finite")
        return waypoint

    def _load_corridor_nodes(self, raw_nodes) -> dict[str, Waypoint]:
        if not isinstance(raw_nodes, dict) or not raw_nodes:
            raise WaypointConfigurationError("No UAV corridor nodes are configured")
        return {
            name: self._parse_waypoint(name, raw, "corridor node")
            for name, raw in raw_nodes.items()
        }

    def _load_corridor_edges(self, raw_edges) -> list[tuple[str, str]]:
        if not isinstance(raw_edges, list) or not raw_edges:
            raise WaypointConfigurationError("No UAV corridor edges are configured")
        edges = []
        for index, raw_edge in enumerate(raw_edges):
            if not isinstance(raw_edge, list) or len(raw_edge) != 2:
                raise WaypointConfigurationError(
                    f"Corridor edge {index} must contain exactly two node names"
                )
            edges.append((str(raw_edge[0]), str(raw_edge[1])))
        return edges

    def _required_float(self, name: str) -> float:
        if name not in self.flight:
            raise WaypointConfigurationError(f"Missing flight setting '{name}'")
        value = float(self.flight[name])
        if not math.isfinite(value):
            raise WaypointConfigurationError(f"Flight setting '{name}' is not finite")
        return value

    def _validate_flight_settings(self) -> None:
        home = self.flight.get("home")
        if home not in self.waypoints:
            raise WaypointConfigurationError("Configured home waypoint does not exist")
        cruise = self._required_float("cruise_altitude")
        minimum_corridor = self._required_float("minimum_corridor_altitude")
        maximum_corridor = self._required_float("maximum_corridor_altitude")
        maximum_delivery = self._required_float("maximum_delivery_altitude")
        if not minimum_corridor <= cruise <= maximum_corridor:
            raise WaypointConfigurationError(
                "Cruise altitude is outside the configured corridor altitude band"
            )
        for name in (
            "takeoff_altitude",
            "delivery_altitude",
            "landing_approach_altitude",
            "position_tolerance",
            "segment_timeout",
            "maximum_obstacle_height",
            "safety_margin",
            "floor_height",
            "first_floor_altitude",
        ):
            if self._required_float(name) <= 0.0:
                raise WaypointConfigurationError(f"Flight setting '{name}' must be positive")
        for waypoint in self.waypoints.values():
            delivery_altitude = self.delivery_altitude_for(waypoint)
            if delivery_altitude <= 0.0:
                raise WaypointConfigurationError(
                    f"Waypoint '{waypoint.name}' delivery altitude must be positive"
                )
            if delivery_altitude > maximum_delivery:
                raise WaypointConfigurationError(
                    f"Waypoint '{waypoint.name}' delivery altitude exceeds the delivery ceiling"
                )

    def _validate_corridors(self) -> None:
        self._adjacency = {name: set() for name in self.corridor_nodes}
        for start, end in self.corridor_edges:
            if start not in self.corridor_nodes or end not in self.corridor_nodes:
                raise WaypointConfigurationError(
                    f"Corridor edge '{start} -> {end}' references an unknown node"
                )
            if start == end:
                raise WaypointConfigurationError(
                    f"Corridor edge '{start}' cannot connect a node to itself"
                )
            self._adjacency[start].add(end)
            self._adjacency[end].add(start)

        for name, waypoint in self.waypoints.items():
            if name not in self.corridor_nodes:
                raise WaypointConfigurationError(
                    f"Delivery waypoint '{name}' is missing from the corridor graph"
                )
            corridor = self.corridor_nodes[name]
            if math.hypot(waypoint.x - corridor.x, waypoint.y - corridor.y) > 0.05:
                raise WaypointConfigurationError(
                    f"Delivery waypoint '{name}' does not match its corridor node"
                )

        visited = set()
        pending = [self.flight["home"]]
        while pending:
            node = pending.pop()
            if node in visited:
                continue
            visited.add(node)
            pending.extend(self._adjacency[node] - visited)
        if visited != set(self.corridor_nodes):
            disconnected = ", ".join(sorted(set(self.corridor_nodes) - visited))
            raise WaypointConfigurationError(
                f"UAV corridor graph is disconnected: {disconnected}"
            )

    @property
    def home(self) -> Waypoint:
        return self.waypoints[self.flight["home"]]

    def resolve(self, names) -> list[Waypoint]:
        resolved = []
        for name in names:
            if name not in self.waypoints:
                available = ", ".join(sorted(self.waypoints))
                raise WaypointConfigurationError(
                    f"Unknown UAV target '{name}'. Available targets: {available}"
                )
            resolved.append(self.waypoints[name])
        return resolved

    def resolve_home(self, name: str) -> Waypoint:
        """Resolve an optional mission home from any connected corridor node."""
        if not name:
            return self.home
        if name not in self.corridor_nodes:
            available = ", ".join(sorted(self.corridor_nodes))
            raise WaypointConfigurationError(
                f"Unknown UAV home node '{name}'. Available nodes: {available}"
            )
        return self.corridor_nodes[name]

    def delivery_altitude_for(
        self,
        waypoint: Waypoint,
        delivery_floor: int | None = None,
    ) -> float:
        if delivery_floor is not None:
            try:
                floor = int(delivery_floor)
            except (TypeError, ValueError) as error:
                raise WaypointConfigurationError(
                    f"Invalid delivery floor for '{waypoint.name}'"
                ) from error
            if floor <= 0:
                raise WaypointConfigurationError(
                    f"Delivery floor for '{waypoint.name}' must be positive"
                )
            if waypoint.maximum_floor is None:
                raise WaypointConfigurationError(
                    f"Waypoint '{waypoint.name}' does not support floor delivery"
                )
            if floor > waypoint.maximum_floor:
                raise WaypointConfigurationError(
                    f"Delivery floor {floor} exceeds '{waypoint.name}' maximum "
                    f"floor {waypoint.maximum_floor}"
                )
            altitude = self._required_float("first_floor_altitude") + (
                floor - 1
            ) * self._required_float("floor_height")
            maximum = self._required_float("maximum_delivery_altitude")
            if altitude > maximum + 1e-9:
                raise WaypointConfigurationError(
                    f"Delivery floor {floor} for '{waypoint.name}' exceeds the "
                    "configured delivery ceiling"
                )
            return altitude
        if waypoint.delivery_altitude is not None:
            return waypoint.delivery_altitude
        return self._required_float("delivery_altitude")

    def resolve_delivery_targets(
        self,
        names,
        target_floors=None,
    ) -> list[Waypoint]:
        targets = self.resolve(names)
        requested_floors = list(target_floors or [])
        if requested_floors and len(requested_floors) != len(targets):
            raise WaypointConfigurationError(
                "target_floors must be empty or match the target count"
            )
        if not requested_floors:
            return targets
        return [
            replace(
                target,
                delivery_floor=int(floor),
                delivery_altitude=self.delivery_altitude_for(target, floor),
            )
            for target, floor in zip(targets, requested_floors)
        ]

    @staticmethod
    def payload_mass_for(
        waypoint: Waypoint,
        default_payload_mass_kg: float,
    ) -> float:
        if waypoint.payload_mass_kg is not None:
            return waypoint.payload_mass_kg
        return float(default_payload_mass_kg)

    def plan_route(self, start: str, goal: str) -> list[Waypoint]:
        if start not in self.corridor_nodes or goal not in self.corridor_nodes:
            raise WaypointConfigurationError(
                f"Cannot plan corridor route from '{start}' to '{goal}'"
            )
        if start == goal:
            return []

        distances = {start: 0.0}
        previous = {}
        queue = [(0.0, start)]
        while queue:
            distance, node = heapq.heappop(queue)
            if distance != distances.get(node):
                continue
            if node == goal:
                break
            origin = self.corridor_nodes[node]
            for neighbor in self._adjacency[node]:
                destination = self.corridor_nodes[neighbor]
                edge_length = math.hypot(
                    destination.x - origin.x,
                    destination.y - origin.y,
                )
                candidate = distance + edge_length
                if candidate < distances.get(neighbor, math.inf):
                    distances[neighbor] = candidate
                    previous[neighbor] = node
                    heapq.heappush(queue, (candidate, neighbor))

        if goal not in previous:
            raise WaypointConfigurationError(
                f"No UAV corridor route exists from '{start}' to '{goal}'"
            )
        route_names = []
        node = goal
        while node != start:
            route_names.append(node)
            node = previous[node]
        route_names.reverse()
        return [self.corridor_nodes[name] for name in route_names]

    def route_distance(self, start: str, goal: str) -> float:
        key = (str(start), str(goal))
        if key in self._route_distance_cache:
            return self._route_distance_cache[key]
        route = self.plan_route(*key)
        current = self.corridor_nodes[key[0]]
        distance = 0.0
        for waypoint in route:
            distance += math.hypot(
                waypoint.x - current.x,
                waypoint.y - current.y,
            )
            current = waypoint
        self._route_distance_cache[key] = distance
        self._route_distance_cache[(key[1], key[0])] = distance
        return distance
