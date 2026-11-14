"""Nominal nine-pack admission; not an optimal scheduler or a dynamic bound."""
from dataclasses import asdict
import math

from cooperative_delivery.relay_energy import DriveLeg, FlightLeg, WaitLeg, evaluate_relay_energy
from uav_application.mission_energy import MissionEnergyPlanner
from uav_control.battery_model import BatteryModel
from uav_navigation.road_network import RoadNetwork

from campus_regional_delivery.energy_model import NAMESPACES


def regional_road_network(layout):
    # Split T-junctions before using the existing proven Dijkstra road graph.
    lines = [road["sampled_points"] for road in layout.roads]
    junctions = [p for line in lines for p in line]
    segments = []
    for line in lines:
        for start, end in zip(line, line[1:]):
            dx, dy = end[0]-start[0], end[1]-start[1]
            squared = dx*dx+dy*dy
            if squared < 1e-12:
                continue
            points = [(0., start), (1., end)]
            for p in junctions:
                t = ((p[0]-start[0])*dx+(p[1]-start[1])*dy)/squared
                if 1e-7 < t < 1-1e-7 and math.hypot(p[0]-start[0]-t*dx, p[1]-start[1]-t*dy) < 1e-6:
                    points.append((t, p))
            points.sort(key=lambda value: value[0])
            segments += [[a[1], b[1]] for a, b in zip(points, points[1:]) if b[0]-a[0] > 1e-7]
    return RoadNetwork(segments, projection_tolerance=3.)


class RegionalEnergyPlanner:
    def __init__(self, layout, settings, uav, ugv):
        self.layout, self.settings, self.uav, self.ugv = layout, settings, uav, ugv
        self.road = regional_road_network(layout)
        self.flight_planner = MissionEnergyPlanner(None, uav)
        self.battery = BatteryModel(uav)

    def flight(self, source, destination, mass, floor=0):
        height = (1.6 + 3.2*(floor-1)) if floor else self.layout.data["flight"]["cruise_altitude"]
        profile = self.flight_planner.plan_transfer(
            [(*source[:2], height), (*destination[:2], height)],
            self.layout.data["flight"]["landing_height"], mass,
            self.layout.data["flight"]["hover_seconds"])
        energy = self.battery.estimate(profile).estimated_mission_energy_wh
        seconds = sum(segment.duration_seconds for segment in profile.segments)
        return energy, seconds*self.settings["flight_time_margin"]

    def build_steps(self, batches, return_home, poses, ground_only=False):
        current = {ns: list(poses[ns][:2]) for ns in NAMESPACES}
        steps = []
        hub = self.layout.hub["namespace"]
        hub_position = self.layout.hub["depot_pose"][:2]
        reserve = self.uav.capacity_wh*self.uav.reserve_percentage
        service = self.settings["parked_service_allowance_s"]
        def drive(ns, target, cargo, owner):
            distance = self.road.distance(current[ns], target)
            steps.append(DriveLeg(ns, distance, cargo, owner))
            if distance:
                steps.append(WaitLeg(self.settings["drive_turn_allowance_s"], owner))
            current[ns] = list(target[:2])
        if ground_only:
            cargo = sum(order["mass"] for _, orders in batches for order in orders)
            drive(hub, hub_position, cargo, "")
            for _, orders in batches:
                for order in orders:
                    drive(hub, self.layout.buildings[order["target"]]["delivery_pose"][:2], cargo, "")
                    steps.append(WaitLeg(self.layout.data["flight"]["ground_service_seconds"]+service, ""))
                    cargo -= order["mass"]
            if return_home:
                drive(hub, hub_position, 0., "")
            return steps
        for index, (region, orders) in enumerate(batches):
            spec = self.layout.regions[region]
            ns, depot = spec["namespace"], spec["depot_pose"][:2]
            drive(ns, depot, 0., hub)
            steps.append(WaitLeg(service, hub))
            back_energy, back_time = self.flight(depot, hub_position, 0.)
            mass = sum(order["mass"] for order in orders)
            out_energy, out_time = self.flight(hub_position, depot, mass)
            departure = self.uav.capacity_wh if self.settings["charging_policy"]=="full" else out_energy+back_energy+reserve+.5
            steps.append(FlightLeg(hub, ns, out_energy, out_time, departure))
            steps.append(WaitLeg(service, ns))
            for order in orders:
                target = self.layout.buildings[order["target"]]["delivery_pose"][:2]
                drive(ns, target, mass, ns)
                if order["floor"]:
                    floor_energy, floor_time = self.flight(target, target, order["mass"], order["floor"])
                    safe_return = self.flight(target, hub_position, 0.)[0]
                    departure = self.uav.capacity_wh if self.settings["charging_policy"]=="full" else floor_energy+safe_return+reserve+.5
                    steps.append(FlightLeg(ns, ns, floor_energy, floor_time, departure))
                else:
                    steps.append(WaitLeg(self.layout.data["flight"]["ground_service_seconds"]+service, ns))
                mass = max(0., mass-order["mass"])
            drive(ns, depot, 0., ns)
            steps.append(WaitLeg(service, ns))
            # Budget safe return also when the requested final pose is a regional car.
            departure = self.uav.capacity_wh if self.settings["charging_policy"]=="full" else back_energy+reserve+.5
            steps.append(FlightLeg(ns, hub, back_energy, back_time, departure))
            steps.append(WaitLeg(service, hub))
        return steps

    def assess(self, batches, return_home, poses, initial_wh, ground_only=False):
        steps = self.build_steps(batches, return_home, poses, ground_only)
        energy = dict(initial_wh)
        if ground_only:
            # The common forecast includes UAV standby; use a zero-load virtual
            # UAV only inside this prediction, never publish it as a ninth pack.
            from dataclasses import replace
            uav = replace(self.uav, docked_idle_power_w=0., landed_idle_power_w=0.)
            energy["uav"] = uav.capacity_wh
        else:
            uav = self.uav
        result = evaluate_relay_energy(steps, self.ugv, uav, energy, roles=NAMESPACES)
        data = asdict(result)
        if ground_only:
            for name in ("remaining_wh", "consumed_wh"):
                data[name].pop("uav", None)
        data["steps"] = [{"type": type(step).__name__, **asdict(step)} for step in steps]
        data["scope"] = "Nominal static routing with margins; unbounded obstacle waits are not guaranteed"
        return data
