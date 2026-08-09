"""Fast shortest-path distances on the configured campus road centerlines."""

from dataclasses import dataclass
import heapq
import math
from pathlib import Path

import yaml


class RoadNetworkError(ValueError):
    """Raised when a road layout or query point cannot be used safely."""


@dataclass(frozen=True)
class _Segment:
    first: int
    second: int
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    length: float


@dataclass(frozen=True)
class _Projection:
    segment_index: int
    along: float
    offset: float


class RoadNetwork:
    """Undirected weighted graph built from road centerline polylines."""

    def __init__(self, centerlines, projection_tolerance: float = 3.0) -> None:
        tolerance = float(projection_tolerance)
        if not math.isfinite(tolerance) or tolerance <= 0.0:
            raise RoadNetworkError("projection_tolerance must be positive")
        self.projection_tolerance = tolerance
        self.nodes: list[tuple[float, float]] = []
        self.adjacency: list[dict[int, float]] = []
        self.segments: list[_Segment] = []
        self._node_indices: dict[tuple[float, float], int] = {}
        self._distance_cache: dict[int, tuple[float, ...]] = {}

        for centerline in centerlines:
            points = [self._point(value) for value in centerline]
            if len(points) < 2:
                raise RoadNetworkError(
                    "Every road centerline must contain at least two points"
                )
            for start, end in zip(points, points[1:]):
                self._add_segment(start, end)
        if not self.segments:
            raise RoadNetworkError("Road layout contains no usable segments")

    @classmethod
    def from_yaml(cls, path, projection_tolerance: float = 3.0):
        layout_path = Path(path)
        try:
            with layout_path.open(encoding="utf-8") as stream:
                data = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as error:
            raise RoadNetworkError(
                f"Could not load road layout '{layout_path}': {error}"
            ) from error
        roads = data.get("roads") if isinstance(data, dict) else None
        if not isinstance(roads, list) or not roads:
            raise RoadNetworkError(
                "Road layout must define a non-empty roads list"
            )
        try:
            centerlines = [road["centerline"] for road in roads]
        except (KeyError, TypeError) as error:
            raise RoadNetworkError(
                "Every road must define a centerline"
            ) from error
        return cls(centerlines, projection_tolerance)

    @staticmethod
    def _point(value) -> tuple[float, float]:
        try:
            point = (float(value[0]), float(value[1]))
        except (IndexError, TypeError, ValueError) as error:
            raise RoadNetworkError(f"Invalid road point: {value!r}") from error
        if not all(math.isfinite(coordinate) for coordinate in point):
            raise RoadNetworkError("Road points must be finite")
        return point

    def _node(self, point: tuple[float, float]) -> int:
        key = (round(point[0], 6), round(point[1], 6))
        index = self._node_indices.get(key)
        if index is not None:
            return index
        index = len(self.nodes)
        self._node_indices[key] = index
        self.nodes.append(point)
        self.adjacency.append({})
        return index

    def _add_segment(
        self,
        start: tuple[float, float],
        end: tuple[float, float],
    ) -> None:
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        if length <= 1e-9:
            return
        first = self._node(start)
        second = self._node(end)
        previous = self.adjacency[first].get(second, math.inf)
        edge_cost = min(previous, length)
        self.adjacency[first][second] = edge_cost
        self.adjacency[second][first] = edge_cost
        self.segments.append(_Segment(
            first,
            second,
            start[0],
            start[1],
            end[0],
            end[1],
            length,
        ))

    def _project(self, point: tuple[float, float]) -> _Projection:
        best = None
        for index, segment in enumerate(self.segments):
            dx = segment.end_x - segment.start_x
            dy = segment.end_y - segment.start_y
            ratio = (
                (point[0] - segment.start_x) * dx
                + (point[1] - segment.start_y) * dy
            ) / (segment.length * segment.length)
            ratio = min(1.0, max(0.0, ratio))
            projected_x = segment.start_x + ratio * dx
            projected_y = segment.start_y + ratio * dy
            offset = math.hypot(
                point[0] - projected_x,
                point[1] - projected_y,
            )
            candidate = _Projection(index, ratio * segment.length, offset)
            if best is None or candidate.offset < best.offset:
                best = candidate
        if best is None or best.offset > self.projection_tolerance:
            offset = math.inf if best is None else best.offset
            raise RoadNetworkError(
                f"Point {point} is {offset:.2f} m from the nearest road "
                f"(limit {self.projection_tolerance:.2f} m)"
            )
        return best

    def _shortest_from(self, start: int) -> tuple[float, ...]:
        cached = self._distance_cache.get(start)
        if cached is not None:
            return cached
        distances = [math.inf] * len(self.nodes)
        distances[start] = 0.0
        queue = [(0.0, start)]
        while queue:
            cost, node = heapq.heappop(queue)
            if cost > distances[node]:
                continue
            for neighbor, edge_cost in self.adjacency[node].items():
                candidate = cost + edge_cost
                if candidate < distances[neighbor]:
                    distances[neighbor] = candidate
                    heapq.heappush(queue, (candidate, neighbor))
        result = tuple(distances)
        self._distance_cache[start] = result
        return result

    @staticmethod
    def _endpoint_costs(
        projection: _Projection,
        segment: _Segment,
    ) -> tuple[tuple[int, float], tuple[int, float]]:
        return (
            (segment.first, projection.offset + projection.along),
            (
                segment.second,
                projection.offset + segment.length - projection.along,
            ),
        )

    def distance(self, first, second) -> float:
        """Return centerline distance between two ``(x, y)`` points."""
        start = self._point(first)
        goal = self._point(second)
        if math.hypot(goal[0] - start[0], goal[1] - start[1]) <= 1e-9:
            return 0.0

        start_projection = self._project(start)
        goal_projection = self._project(goal)
        start_segment = self.segments[start_projection.segment_index]
        goal_segment = self.segments[goal_projection.segment_index]
        candidates = []
        if start_projection.segment_index == goal_projection.segment_index:
            candidates.append(
                start_projection.offset
                + abs(start_projection.along - goal_projection.along)
                + goal_projection.offset
            )

        for start_node, start_cost in self._endpoint_costs(
            start_projection, start_segment
        ):
            graph_distances = self._shortest_from(start_node)
            for goal_node, goal_cost in self._endpoint_costs(
                goal_projection, goal_segment
            ):
                graph_cost = graph_distances[goal_node]
                if math.isfinite(graph_cost):
                    candidates.append(start_cost + graph_cost + goal_cost)
        if not candidates:
            raise RoadNetworkError("Road network is disconnected")
        return min(candidates)
