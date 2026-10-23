"""Deterministically derive world, Nav2 maps and preview from one layout."""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml

from campus_regional_delivery.layout import RegionalLayout


def write_yaml(path, data):
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")


def add_shape(link, name, pose, size, color, collision=False, cylinder=False):
    element = ET.SubElement(link, "visual", {"name": name})
    ET.SubElement(element, "pose").text = " ".join(str(v) for v in pose)
    geometry = ET.SubElement(element, "geometry")
    if cylinder:
        shape = ET.SubElement(geometry, "cylinder")
        ET.SubElement(shape, "radius").text = str(size[0])
        ET.SubElement(shape, "length").text = str(size[1])
    else:
        shape = ET.SubElement(geometry, "box")
        ET.SubElement(shape, "size").text = " ".join(str(v) for v in size)
    material = ET.SubElement(element, "material")
    for kind in ("ambient", "diffuse"):
        ET.SubElement(material, kind).text = " ".join(str(v) for v in color)
    if collision:
        copied = deepcopy(element)
        copied.tag = "collision"
        copied.set("name", name + "_collision")
        copied.remove(copied.find("material"))
        link.append(copied)


def model(world, name):
    element = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(element, "static").text = "true"
    return ET.SubElement(element, "link", {"name": "body"})


def collision_bounds(source_model, center):
    boxes = []
    for link in source_model.findall("link"):
        link_pose = [float(v) for v in link.findtext("pose", "0 0 0 0 0 0").split()]
        for collision in link.findall("collision"):
            raw = collision.findtext("geometry/box/size")
            if raw is None:
                raise ValueError("Existing campus building collision must remain a box")
            size = [float(v) for v in raw.split()]
            p = [float(v) for v in collision.findtext("pose", "0 0 0 0 0 0").split()]
            if any(abs(v) > 1e-8 for v in p[3:] + link_pose[3:]):
                raise ValueError("Rotated building collision requires explicit support")
            x, y, z = p[0]+link_pose[0]+center[0], p[1]+link_pose[1]+center[1], p[2]+link_pose[2]
            boxes.append((x-size[0]/2, x+size[0]/2, y-size[1]/2, y+size[1]/2, z+size[2]/2))
    return [min(b[0] for b in boxes), max(b[1] for b in boxes),
            min(b[2] for b in boxes), max(b[3] for b in boxes)], max(b[4] for b in boxes)


def generate(layout_path, source_world, source_layout, output, preview=True):
    layout = RegionalLayout(layout_path)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    originals = ET.parse(source_world).getroot().find("world")
    metadata = {b["model_name"]: b for b in yaml.safe_load(Path(source_layout).read_text())["buildings"]}
    root = ET.Element("sdf", {"version": "1.7"})
    world = ET.SubElement(root, "world", {"name": "campus_regional"})
    for item in originals:
        if item.tag in ("physics", "light") or item.get("name") in ("campus_ground", "campus_wall"):
            world.append(deepcopy(item))
    buildings = dict(layout.buildings)
    buildings["logistics_center"] = dict(layout.hub, center=layout.hub["building_center"])
    obstacles, derived = [], {}
    for name, building in buildings.items():
        old = originals.find(f"model[@name='{building['model_name']}']")
        if old is None:
            raise ValueError(f"Missing source building {name}")
        copied = deepcopy(old)
        center = building["center"]
        copied.find("pose").text = f"{center[0]} {center[1]} 0 0 0 0"
        world.append(copied)
        bounds, height = collision_bounds(copied, center)
        record = dict(name=name, model_name=building["model_name"], bounds=bounds, height=height,
                      floors=metadata[building["model_name"]]["floors"],
                      size=metadata[building["model_name"]]["size"],
                      source_geometry_sha256=hashlib.sha256(ET.tostring(old.find("link"))).hexdigest())
        obstacles.append(record)
        derived[name] = record
    # Reject overlaps, rather than masking them out of the navigation raster.
    for i, a in enumerate(obstacles):
        for b in obstacles[i+1:]:
            ax0, ax1, ay0, ay1 = a["bounds"]
            bx0, bx1, by0, by1 = b["bounds"]
            if min(ax1, bx1) > max(ax0, bx0) and min(ay1, by1) > max(ay0, by0):
                raise ValueError(f"Buildings overlap: {a['name']}, {b['name']}")
    road_link = model(world, "regional_roads")
    asphalt = [.18, .20, .21, 1]
    for road in layout.roads:
        points, width = road["sampled_points"], road["width"]
        for i, (a, b) in enumerate(zip(points, points[1:])):
            length = math.dist(a, b)
            add_shape(road_link, f"{road['name']}_{i}",
                      [(a[0]+b[0])/2, (a[1]+b[1])/2, .002, 0, 0,
                       math.atan2(b[1]-a[1], b[0]-a[0])], [length, width, .004], asphalt)
        for i, p in enumerate(points):
            add_shape(road_link, f"{road['name']}_joint_{i}", [*p, .002, 0, 0, 0],
                      [width/2, .004], asphalt, cylinder=True)
    for i, (center, size) in enumerate(layout.paved_areas()):
        add_shape(road_link, f"loading_and_turning_{i}", [*center, .002, 0, 0, 0],
                  [*size, .004], asphalt)
    marks = model(world, "regional_loading_markers")
    for key, region in layout.regions.items():
        x, y, _, _ = region["depot_pose"]
        color = region["color"]
        for dx, dy, w, h in ((0, -2, 4, .12), (0, 2, 4, .12), (-2, 0, .12, 4), (2, 0, .12, 4)):
            add_shape(marks, f"{key}_bay_{dx}_{dy}", [x+dx, y+dy, .012, 0, 0, 0], [w, h, .01], color)
        sx, sy = region["sign_position"]
        sign = model(world, f"regional_{key}_sign")
        add_shape(sign, "post", [sx, sy, 1, 0, 0, 0], [.10, .10, 2], color, collision=True)
        add_shape(sign, "board", [sx, sy, 2, 0, 0, 0], [1.8, .15, 1.2], color, collision=True)
        strokes = {"A": [(-.5,-.4,0,.4),(0,.4,.5,-.4),(-.3,-.1,.3,-.1)],
                   "B": [(-.4,-.4,-.4,.4),(-.4,.4,.3,.4),(.3,.4,.4,.1),
                         (.4,.1,-.4,0),(-.4,0,.4,-.1),(.4,-.1,.3,-.4),(.3,-.4,-.4,-.4)],
                   "C": [(.4,.4,-.4,.4),(-.4,.4,-.4,-.4),(-.4,-.4,.4,-.4)]}[key]
        for i, (x0,z0,x1,z1) in enumerate(strokes):
            add_shape(sign, f"letter_{i}", [sx+(x0+x1)/2, sy-.09, 2+(z0+z1)/2,
                                            0, -math.atan2(z1-z0,x1-x0), 0],
                      [math.hypot(x1-x0,z1-z0), .025, .08], [1,1,1,1])
        obstacles.append(dict(name=f"sign_{key}", bounds=[sx-.9,sx+.9,sy-.075,sy+.075], height=2.6))
    hub = layout.hub["uav_pad"]
    for dx in (-1, 1):
        add_shape(marks, f"hub_H_{dx}", [hub[0]+dx,hub[1],.015,0,0,0], [.14,3,.01], [1,1,1,1])
    add_shape(marks, "hub_H_cross", [hub[0],hub[1],.015,0,0,0], [2,.14,.01], [1,1,1,1])
    free, physical, distance = layout.raster(obstacles)
    # Rasterize the uncut pavement to detect any road/building intersection.
    raw_pavement, _, _ = layout.raster([])
    buildings_only = np.zeros(free.shape, dtype=bool)
    for b in obstacles[:len(buildings)]:
        row0, col0 = layout.cell([b["bounds"][0], b["bounds"][3]])
        row1, col1 = layout.cell([b["bounds"][1], b["bounds"][2]])
        buildings_only[row0:row1+1, col0:col1+1] = True
    if (raw_pavement & buildings_only).any():
        raise ValueError("A road or loading area overlaps a building")
    from scipy.ndimage import distance_transform_edt
    minimum_road_clearance=float((distance_transform_edt(~buildings_only)*layout.resolution)[raw_pavement].min())
    if minimum_road_clearance < layout.data["navigation"]["road_building_clearance"]:
        raise ValueError("Pavement has insufficient separation from a building collision")
    report = layout.validate_raster(free, distance, obstacles)
    report["minimum_pavement_building_clearance_m"]=minimum_road_clearance
    report.update(building_count=len(buildings), regions=list(layout.regions),
                  namespaces=[r["namespace"] for r in layout.vehicles.values()],
                  source_world_sha256=hashlib.sha256(Path(source_world).read_bytes()).hexdigest(),
                  layout_sha256=hashlib.sha256(Path(layout_path).read_bytes()).hexdigest(),
                  buildings=derived, obstacles=obstacles)
    for filename, cost in (("regional_map", np.where(free, 0., 100.)),
                           ("regional_keepout", np.where(~free, 100.,
                              np.where(distance < layout.data["navigation"]["road_edge_soft_width"],
                                       layout.data["navigation"]["road_edge_soft_cost"], 0.)))):
        Image.fromarray(np.rint(255*(1-cost/100)).astype(np.uint8)).save(output/f"{filename}.pgm")
        write_yaml(output/f"{filename}.yaml", {"image": f"{filename}.pgm", "mode": "scale",
                   "resolution": layout.resolution, "origin": [layout.bounds[0],layout.bounds[2],0.],
                   "negate": 0, "occupied_thresh": 1.0, "free_thresh": 0.0})
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output/"campus_regional.world", encoding="unicode")
    write_yaml(output/"building_geometry.yaml", derived)
    write_yaml(output/"delivery_waypoints.yaml", {n:{"x":p[0],"y":p[1],"yaw":p[2]}
               for n,p in layout.all_stops().items()})
    write_yaml(output/"uav_targets.yaml", uav_targets(layout, derived))
    (output/"layout_validation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    if preview:
        render_preview(layout, derived, free, output/"campus_layout_preview.png")
    return report


def uav_targets(layout, geometry):
    # Preserve the existing floor manager's schema without reusing old coordinates.
    points = {"logistics_center": layout.hub["uav_pad"][:2] + [0.]}
    points.update({f"depot_{key.lower()}": region["depot_pose"][:2] + [region["depot_pose"][3]]
                   for key, region in layout.regions.items()})
    waypoints = {}
    for name, p in points.items():
        waypoints[name] = dict(x=p[0], y=p[1], yaw=p[2], label=name,
                               maximum_floor=geometry.get(name, {"floors":1})["floors"],
                               delivery_floor=1, payload_mass_kg=.3)
    # Only certified hub/depot corridors are offered as cross-region air edges.
    # Floor sorties are local vertical flights from the parked carrier.
    nodes = {name: {"x":p[0],"y":p[1]} for name,p in points.items()}
    edges = [["logistics_center", f"depot_{key.lower()}"] for key in layout.regions]
    return {"flight": {"home":"logistics_center", "takeoff_altitude":1.5,
            "cruise_altitude":layout.data["flight"]["cruise_altitude"], "minimum_corridor_altitude":8., "maximum_corridor_altitude":20.,
            "maximum_delivery_altitude":42., "first_floor_altitude":1.6, "floor_height":3.2,
            "delivery_altitude":1.6, "landing_approach_altitude":.8,
            "takeoff_hover_duration":5., "delivery_hover_duration":5., "position_tolerance":.4,
            "segment_timeout":240., "maximum_obstacle_height":max(b["height"] for b in geometry.values()),
            "safety_margin":2.5}, "waypoints":waypoints, "corridor_nodes":nodes, "corridor_edges":edges}


def render_preview(layout, geometry, free, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.patches import Rectangle, FancyArrowPatch
    font = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(font)).get_name()
    fig, ax = plt.subplots(figsize=(16, 14))
    xmin,xmax,ymin,ymax=layout.bounds
    background = np.zeros((*free.shape, 3), dtype=np.uint8)
    background[:] = [222,237,219]
    background[free] = [100,109,114]
    ax.imshow(background, extent=[xmin,xmax,ymin,ymax], origin="upper")
    for key, region in layout.regions.items():
        x0,x1,y0,y1 = region["bounds"]
        ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0, fill=False,
                              edgecolor=region["color"], linewidth=2, linestyle="--"))
        if key == "C":
            ax.text(x1+4,(y0+y1)/2,f"{key}区 {region['label']}",ha="center",va="center",
                    rotation=90,fontsize=14,color=region["color"])
        else:
            ax.text((x0+x1)/2,y1+2,f"{key}区 {region['label']}",ha="center",fontsize=14,
                    color=region["color"])
    for name,b in geometry.items():
        x0,x1,y0,y1=b["bounds"]
        color = [.45,.50,.58,1] if name == "logistics_center" else layout.regions[layout.region_for(name)]["color"]
        ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,facecolor=color,edgecolor="white"))
        label = layout.hub["label"] if name=="logistics_center" else layout.buildings[name]["label"]
        ax.text((x0+x1)/2,(y0+y1)/2,f"{label}\n{b['floors']}层",ha="center",va="center",
                fontsize=11,color="white")
    for name,b in layout.buildings.items():
        x,y,_=b["delivery_pose"]
        ax.plot(x,y,"o",color="gold",markersize=5)
    hx,hy=layout.hub["uav_pad"][:2]
    ax.plot(hx,hy,"s",markersize=10,color="black")
    ax.annotate(f"总站车载起降点\nUGV-00 / UAV\n({hx:g}, {hy:g})",(hx,hy),
        xytext=(hx-22,hy+14),fontsize=10,arrowprops={"arrowstyle":"-","color":"black"})
    offsets={"A":(-40,-10),"B":(20,-16),"C":(40,12)}
    for key,r in layout.regions.items():
        x,y=r["depot_pose"][:2]
        ax.add_patch(FancyArrowPatch((hx,hy),(x,y),arrowstyle="<->",mutation_scale=12,
                                     linewidth=2,linestyle="--",color="#d12f8b"))
        ax.plot(x,y,"s",color=r["color"],markeredgecolor="white",markersize=12)
        dx,dy=offsets[key]
        ax.annotate(f"区域配送点{key}\n{r['vehicle_label']}\n({x:g}, {y:g})",(x,y),
                    xytext=(x+dx,y+dy),fontsize=10,ha="left",
                    arrowprops={"arrowstyle":"-","color":"black"})
    ax.plot([],[],color="#646d72",linewidth=7,label="UGV 地面道路 / 装卸回转区")
    ax.plot([],[],"--",color="#d12f8b",label="UAV 15 m 安全空运通道（双向示意）")
    ax.plot([],[],"o",color="gold",label="建筑独立末端配送点")
    ax.text(2,-8,"中心绿地\n（UGV不可通行）",ha="center",va="center",fontsize=12,color="#376944")
    ax.text(2,12,"中央闭合主干道",ha="center",va="center",fontsize=10,color="white")
    ax.set(xlim=(xmin,xmax),ylim=(ymin,ymax),xlabel="Gazebo / map X (m)",ylabel="Gazebo / map Y (m)",
           title="校园三区物流布局：实际 world 与 Nav2 同源坐标")
    ax.set_aspect("equal")
    ax.legend(loc="lower center",ncol=2,fontsize=10)
    ax.grid(alpha=.16)
    fig.tight_layout()
    fig.savefig(path,dpi=140)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--layout",required=True)
    parser.add_argument("--source-world",required=True)
    parser.add_argument("--source-layout",required=True)
    parser.add_argument("--output",required=True)
    args=parser.parse_args()
    print(json.dumps(generate(args.layout,args.source_world,args.source_layout,args.output),
                     ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
