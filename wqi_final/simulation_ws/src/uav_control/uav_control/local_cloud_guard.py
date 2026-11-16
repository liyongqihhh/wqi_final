"""Measured-cloud voxel routing and command validation, not global 3-D SLAM."""
import math
from functools import lru_cache

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from uav_control.dynamic_path_planner import DynamicPathPlan,resample_polyline
from uav_control.safety_geometry import is_ground_return


@lru_cache(maxsize=4)
def voxel_topology(shape=(25,25,17),spacing=.75):
    """Cache geometry only; occupancy, edge validity and costs are always fresh."""
    center=np.array(tuple(n//2 for n in shape))
    offsets=(np.indices(shape).reshape(3,-1).T-center)*spacing
    indices=np.arange(len(offsets)).reshape(shape)
    neighbors=[]
    for delta in np.ndindex((3,3,3)):
        delta=np.array(delta)-1
        if not np.any(delta):continue
        first=tuple(slice(max(0,-int(d)),min(n,n-int(d))) for d,n in zip(delta,shape))
        second=tuple(slice(max(0,int(d)),min(n,n+int(d))) for d,n in zip(delta,shape))
        a,b=indices[first].ravel(),indices[second].ravel()
        a.setflags(write=False);b.setflags(write=False)
        neighbors.append((a,b,spacing*float(np.linalg.norm(delta)),.2*abs(delta[2])*spacing))
    offsets.setflags(write=False)
    return offsets,int(np.ravel_multi_index(tuple(center),shape)),tuple(neighbors)


def motion_clearance(base, speed, cloud_age, obstacle_speed=1., response=.8, braking=.8):
    """Conservative sampled-surface buffer, not an obstacle velocity estimate."""
    values=(base,speed,cloud_age,obstacle_speed,response,braking)
    if not all(math.isfinite(v) for v in values) or min(base,obstacle_speed,response,braking)<=0 or speed<0:
        raise ValueError("Invalid motion-clearance parameters")
    # Safety radius + own response/braking + bounded obstacle travel + sampling error.
    return max(base,1.8+speed*response+speed*speed/(2*braking)
               +obstacle_speed*(response+max(0.,cloud_age))+.35)


def filtered_cloud(points,lidar_height,center_height,ground_clearance,lidar_to_down_sensor,
                   ground_tolerance,self_radius,maximum_range=12.,voxel=.3):
    cells={}
    for raw in points:
        x,y,z=(float(raw[i]) for i in range(3))
        if not all(math.isfinite(v) for v in (x,y,z)):
            continue
        if is_ground_return(z,ground_clearance,lidar_to_down_sensor,ground_tolerance):
            continue
        body=(x,y,z+lidar_height)
        distance=math.sqrt(x*x+y*y+(body[2]-center_height)**2)
        if self_radius<distance<=maximum_range:
            cell=tuple(math.floor(v/voxel) for v in body)
            if cell not in cells or distance<cells[cell][0]:
                cells[cell]=(distance,body)
    # Nearest measured return per voxel, never an unmeasured averaged location.
    return tuple(v[1] for v in sorted(cells.values())[:3000])


class LocalCloudGuard:
    def __init__(self,body_points,translation,quaternion,stamp_ns,received_wall):
        # Humble's read_points returns structured NumPy records, not always
        # ordinary xyz tuples. Convert fields explicitly before spatial math.
        source=np.asarray([tuple(float(point[i]) for i in range(3)) for point in body_points],dtype=float).reshape((-1,3))
        self.points=Rotation.from_quat(quaternion).apply(source)+np.asarray(translation)
        self.tree=cKDTree(self.points) if len(self.points) else None
        self.measured_tree=self.tree
        self.stamp_ns,self.received_wall=stamp_ns,received_wall
        self.motion_tracks=0

    def include_prediction(self,points):
        predicted=np.asarray(points,dtype=float).reshape((-1,3))
        if len(predicted):
            self.points=np.vstack((self.points,predicted))
            self.tree=cKDTree(self.points)

    def fresh(self,now_ns,now_wall):
        return -.25<=(now_ns-self.stamp_ns)/1e9<=.6 and 0<=now_wall-self.received_wall<=30.

    def first_conflict(self,current,target,clearance=2.):
        if self.tree is None:return None
        line=resample_polyline((current,target),.25)
        distances,indices=self.tree.query(line)
        for d,index in zip(distances,indices):
            if d<clearance:
                return tuple(self.points[index])
        return None

    def path_clear(self,points,clearance=2.):
        if self.tree is None:return True
        samples=np.asarray(resample_polyline(points,.2))
        distances,_=self.tree.query(samples)
        current_distance=float(distances[0])
        progress=np.linalg.norm(samples-samples[0],axis=1)
        # Future occupancy is soft enough to leave; measured surfaces retain
        # a separate hard clearance bound even inside a predicted tube.
        required=np.minimum(clearance,current_distance+.15*progress)
        physical=(self.measured_tree.query(samples)[0] if self.measured_tree is not None
            else np.full(len(samples),math.inf))
        hard=np.minimum(min(2.2,clearance),float(physical[0])+.2*progress)
        return bool(np.all(distances>=required-.02) and np.all(physical>=hard-.02))

    def plan(self,current,target,clearance=3.,minimum_altitude=.15,maximum_altitude=45.,
             preferred_side=1,previous_path=(),previous_egress=False):
        start=np.asarray(current,dtype=float);goal=np.asarray(target,dtype=float)
        if self.path_clear((current,target),clearance):
            return DynamicPathPlan(resample_polyline((current,target),.4),False,preferred_side,"clear_3d_sensor_path")
        initial=float(self.tree.query(start)[0])
        urgent=initial<clearance+.4
        if previous_path and (not urgent or previous_egress) and (math.dist(current,previous_path[-1])>2. or math.dist(previous_path[-1],target)<.1):
            nearest=min(range(len(previous_path)),key=lambda i:math.dist(current,previous_path[i]))
            remaining=(tuple(start),*previous_path[nearest:])
            if self.path_clear(remaining,clearance):
                reason="retained_egress_3d" if previous_egress else "retained_3d_sensor_path"
                return DynamicPathPlan(remaining,True,preferred_side,reason)
        # A bounded 26-connected local voxel graph; shortest-path computation
        # is SciPy's compiled Dijkstra, not a hand-written flight planner.
        offsets,source,neighbors=voxel_topology()
        points=start+offsets
        distances,_=self.tree.query(points)
        initial=float(self.tree.query(start)[0])
        threshold=np.minimum(clearance+.35,initial+.15*np.linalg.norm(offsets,axis=1))
        physical=(self.measured_tree.query(points)[0] if self.measured_tree is not None
            else np.full(len(points),math.inf))
        measured_start=float(self.measured_tree.query(start)[0]) if self.measured_tree is not None else math.inf
        hard=np.minimum(min(2.2,clearance),measured_start+.2*np.linalg.norm(offsets,axis=1))
        free=(distances>=threshold-.02)&(physical>=hard-.02)&(points[:,2]>=minimum_altitude)&(points[:,2]<=maximum_altitude)
        free[source]=True
        sources=[];destinations=[];weights=[]
        direction=goal-start;length=float(np.linalg.norm(direction));direction/=length
        for a,b,step,vertical_cost in neighbors:
            valid=free[a]&free[b];a=a[valid];b=b[valid]
            if not len(a):continue
            cost=step*(1.+.15/(np.maximum(0.,distances[b]-clearance)+.5))+vertical_cost
            sources.append(a);destinations.append(b);weights.append(cost)
        if not sources:raise ValueError("3D_NO_COLLISION_FREE_LOCAL_PATH")
        graph=coo_matrix((np.concatenate(weights),(np.concatenate(sources),np.concatenate(destinations))),
            shape=(len(points),len(points))).tocsr()
        costs,predecessors=dijkstra(graph,indices=source,return_predecessors=True)
        horizon=min(8.,length)
        progress=offsets@direction
        candidates=free&np.isfinite(costs)&(progress>=max(.5,horizon-2.))&(np.linalg.norm(offsets,axis=1)<=12.)
        if length<=8.:
            candidates&=np.linalg.norm(points-goal,axis=1)<=2.
        choices=np.flatnonzero(candidates)
        scores=costs[choices]+np.linalg.norm(points[choices]-goal,axis=1)
        ordered=list(choices[np.argsort(scores)[:60]])
        # If no route toward the goal is currently feasible, search measured
        # free space for a clearance-improving egress instead of a fixed manoeuvre.
        egress=np.flatnonzero(free&np.isfinite(costs)&(np.linalg.norm(offsets,axis=1)>=2.)
            &(np.linalg.norm(offsets,axis=1)<=8.)
            &(distances>=min(clearance+.25,initial+.75)))
        escape_scores=costs[egress]*.2+np.linalg.norm(points[egress]-goal,axis=1)*.1-2*np.minimum(distances[egress],clearance+1.)
        escapes=[int(i) for i in egress[np.argsort(escape_scores)[:60]]]
        ordered=(escapes+[i for i in ordered if i not in escapes]) if urgent else (
            ordered+[i for i in escapes if i not in ordered])
        forward=set(int(i) for i in choices)
        for endpoint in ordered:
            chain=[];index=int(endpoint)
            while index!=source and index>=0:
                chain.append(tuple(points[index]));index=int(predecessors[index])
            if index!=source:continue
            chain=[tuple(start),*reversed(chain)]
            reaches_goal=self.path_clear((*chain,tuple(goal)),clearance)
            if reaches_goal:
                chain.append(tuple(goal))
            elif endpoint in forward and length<=8. and not (urgent and endpoint in escapes):
                continue
            if not self.path_clear(chain,clearance):continue
            simplified=[chain[0]];i=0
            while i<len(chain)-1:
                j=len(chain)-1
                while j>i+1 and not self.path_clear((chain[i],chain[j]),clearance):j-=1
                simplified.append(chain[j]);i=j
            path=resample_polyline(simplified,.4)
            if self.path_clear(path,clearance):
                reason="voxel_egress_3d" if urgent and endpoint in escapes else ("voxel_dijkstra_3d" if reaches_goal else (
                    "voxel_local_horizon_3d" if endpoint in forward else "voxel_egress_3d")
                )
                return DynamicPathPlan(path,True,preferred_side,reason)
        raise ValueError("3D_NO_COLLISION_FREE_LOCAL_PATH")
