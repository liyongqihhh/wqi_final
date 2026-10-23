"""Single-coordinate-source geometry for the regional campus scene."""
from copy import deepcopy
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt, label
import yaml


class RegionalLayout:
    def __init__(self, path):
        self.path = Path(path)
        self.data = yaml.safe_load(self.path.read_text(encoding="utf-8"))
        self.regions = self.data["regions"]
        self.buildings = self.data["buildings"]
        self.hub = self.data["central_hub"]
        self.resolution = float(self.data["resolution"])
        b = self.data["world_bounds"]
        self.bounds = tuple(float(b[n]) for n in ("x_min", "x_max", "y_min", "y_max"))
        self._validate()

    @property
    def vehicles(self):
        return {"H":self.hub,**self.regions}

    def _validate(self):
        if set(self.regions) != {"A", "B", "C"}:
            raise ValueError("Exactly three regions A/B/C are required")
        names = [r["namespace"] for r in self.regions.values()]
        if len(set(names)) != 3 or any(n != f"ugv_{i:02}" for i, n in enumerate(names, 1)):
            raise ValueError("Expected independent namespaces ugv_01, ugv_02, ugv_03")
        if self.hub.get("namespace")!="ugv_00":
            raise ValueError("The central logistics carrier must use namespace ugv_00")
        if self.hub["depot_pose"][:2]!=self.hub["uav_pad"][:2] or abs(
                self.hub["uav_pad"][2]-self.hub["depot_pose"][2]-.42)>.02:
            raise ValueError("The initial UAV must rest on the central carrier's physical deck")
        targets = [t for r in self.regions.values() for t in r["targets"]]
        if len(targets) != len(set(targets)) or set(targets) != set(self.buildings):
            raise ValueError("Every building must belong to exactly one region")
        def finite(value):
            if isinstance(value, dict):
                return all(finite(v) for v in value.values())
            if isinstance(value, list):
                return all(finite(v) for v in value)
            return not isinstance(value, (float, int)) or math.isfinite(value)
        if not finite(self.data) or not .05 <= self.resolution <= .2:
            raise ValueError("Coordinates must be finite and map resolution in [0.05, 0.2]")
        if abs(self.data["flight"]["landing_height"]-.42)>1e-6:
            raise ValueError("The retained physical Iris/carrier deck requires 0.42 m landing height")
        for r in self.regions.values():
            if len(r["depot_pose"]) != 4:
                raise ValueError("Depot pose requires x, y, z, yaw")
        self.roads = []
        for road in self.data["roads"]:
            if road["width"] < 2 * (self.data["navigation"]["vehicle_radius"] +
                                    self.data["navigation"]["inflation_radius"]) + 1:
                raise ValueError("Road too narrow for the docked footprint and inflation")
            points = rounded_polyline(road["points"], road.get("corner_radius", 0),
                                      road.get("closed", False))
            self.roads.append(dict(road, sampled_points=points))

    def region_for(self, target):
        return next(k for k, r in self.regions.items() if target in r["targets"])

    def paved_areas(self):
        result = [(self.hub["uav_pad"][:2], self.hub["paved_size"])]
        result += [(r["depot_pose"][:2], r["paved_size"]) for r in self.regions.values()]
        result += [(b["delivery_pose"][:2], [6., 6.]) for b in self.buildings.values()]
        return result

    def all_stops(self):
        result = {k: b["delivery_pose"] for k, b in self.buildings.items()}
        result["logistics_center"] = self.hub["uav_pad"][:2] + [self.hub["uav_pad"][3]]
        for key, r in self.regions.items():
            result[f"depot_{key.lower()}"] = r["depot_pose"][:2] + [r["depot_pose"][3]]
        return result

    def raster(self, obstacles):
        xmin, xmax, ymin, ymax = self.bounds
        x = np.arange(xmin + self.resolution / 2, xmax, self.resolution)
        y = np.arange(ymax - self.resolution / 2, ymin, -self.resolution)
        xx, yy = np.meshgrid(x, y)
        paved = np.zeros(xx.shape, dtype=bool)
        for road in self.roads:
            points = road["sampled_points"]
            for a, b in zip(points, points[1:]):
                paved |= segment_distance(xx, yy, a, b) <= road["width"] / 2
        for center, size in self.paved_areas():
            paved |= (np.abs(xx-center[0]) <= size[0]/2) & (np.abs(yy-center[1]) <= size[1]/2)
        physical = np.zeros(xx.shape, dtype=bool)
        for obstacle in obstacles:
            x0, x1, y0, y1 = obstacle["bounds"]
            physical |= (xx >= x0) & (xx <= x1) & (yy >= y0) & (yy <= y1)
        free = paved & ~physical
        edge_distance = distance_transform_edt(free) * self.resolution
        return free, physical, edge_distance

    def validate_raster(self, free, distance, obstacles):
        connected, count = label(distance > self.data["navigation"]["vehicle_radius"] + .15)
        components = {}
        for name, pose in self.all_stops().items():
            row, col = self.cell(pose[:2])
            if not free[row, col] or distance[row, col] < 1.0:
                raise ValueError(f"Stop {name} lacks paved turning/footprint clearance")
            components[name] = int(connected[row, col])
        if 0 in components.values() or len(set(components.values())) != 1:
            raise ValueError(f"Disconnected road network: {components}")
        hub = self.hub["uav_pad"][:2]
        clearance = self.data["flight"]["corridor_clearance"]
        altitude = self.data["flight"]["cruise_altitude"]
        flight_clearances = {}
        for key, region in self.regions.items():
            destination = region["depot_pose"][:2]
            points = np.linspace(hub, destination, max(2, int(math.dist(hub, destination) / .05)))
            minimum = math.inf
            for obs in obstacles:
                if obs["height"] < altitude - clearance:
                    continue
                x0, x1, y0, y1 = obs["bounds"]
                dx = np.maximum(np.maximum(x0-points[:, 0], points[:, 0]-x1), 0)
                dy = np.maximum(np.maximum(y0-points[:, 1], points[:, 1]-y1), 0)
                minimum = min(minimum, float(np.hypot(dx, dy).min()))
            if minimum < clearance:
                raise ValueError(f"UAV corridor {key} intersects an inflated obstacle")
            flight_clearances[key] = minimum
        return {"road_components": count, "all_stop_component": next(iter(components.values())),
                "stop_count": len(components), "uav_corridor_clearance_m": flight_clearances}

    def cell(self, point):
        return (int((self.bounds[3]-point[1])/self.resolution),
                int((point[0]-self.bounds[0])/self.resolution))


def segment_distance(x, y, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = np.clip(((x-a[0])*dx + (y-a[1])*dy) / (dx*dx+dy*dy), 0, 1)
    return np.hypot(x-a[0]-t*dx, y-a[1]-t*dy)


def rounded_polyline(points, radius=0, closed=False):
    points = [np.array(p, dtype=float) for p in points]
    if radius <= 0:
        return [p.tolist() for p in points + ([points[0]] if closed else [])]
    output = []
    for i, p in enumerate(points):
        if not closed and i in (0, len(points)-1):
            output.append(p.tolist())
            continue
        previous, following = points[i-1], points[(i+1) % len(points)]
        incoming, outgoing = p-previous, following-p
        incoming /= np.linalg.norm(incoming)
        outgoing /= np.linalg.norm(outgoing)
        if abs(float(np.dot(incoming, outgoing))) > 1e-6:
            raise ValueError("Rounded corners currently require perpendicular segments")
        tangent = min(radius, np.linalg.norm(p-previous)/3, np.linalg.norm(following-p)/3)
        start, end = p-incoming*tangent, p+outgoing*tangent
        center = start + outgoing*tangent
        a0 = math.atan2(*(start-center)[::-1])
        sign = float(np.cross(incoming, outgoing))
        for a in np.linspace(a0, a0+math.copysign(math.pi/2, sign), 17):
            output.append((center+tangent*np.array([math.cos(a), math.sin(a)])).tolist())
    if closed:
        output.append(output[0])
    return output


def partition_orders(layout, targets, masses, floors=None, enforce_uav_capacity=True):
    if not targets or len(targets) > 10 or len(targets) != len(masses):
        raise ValueError("Need 1--10 targets with matching payload masses")
    if floors and len(floors) != len(targets):
        raise ValueError("Floor count must match targets")
    maximum = float(layout.data["flight"]["maximum_payload_kg"]) if enforce_uav_capacity else math.inf
    groups = {region: [] for region in layout.regions}
    for index, (name, mass) in enumerate(zip(targets, masses)):
        if name not in layout.buildings:
            raise ValueError(f"Unknown target '{name}'; use one of: {', '.join(layout.buildings)}")
        if not math.isfinite(mass) or mass < 0:
            raise ValueError(f"Invalid parcel mass for '{name}': {mass}; require a finite nonnegative kg value")
        if mass > maximum:
            raise ValueError(f"Parcel for '{name}' weighs {mass:g} kg; UAV limit is {maximum:g} kg per parcel")
        floor = int(floors[index]) if floors else 0
        if floor < 0 or (floors and floor == 0):
            raise ValueError("Explicit floor values must be positive")
        groups[layout.region_for(name)].append({"target": name, "mass": mass, "floor": floor})
    batches = []
    for region, orders in groups.items():
        batch, total = [], 0.
        for order in orders:
            if batch and total+order["mass"] > maximum + 1e-6:
                batches.append((region, batch))
                batch, total = [], 0.
            batch.append(deepcopy(order))
            total += order["mass"]
        if batch:
            batches.append((region, batch))
    return batches
