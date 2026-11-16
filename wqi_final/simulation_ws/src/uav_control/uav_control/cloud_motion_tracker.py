"""Short-lived map-frame lidar tracks; no simulator model-state subscription."""
from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


@dataclass
class _Track:
    center: np.ndarray
    points: np.ndarray
    stamp: float
    history: deque = field(default_factory=deque)
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))


class CloudMotionTracker:
    def __init__(self, speed_bound=1., horizon=4., memory=3.):
        if not all(math.isfinite(v) and v>0 for v in (speed_bound,horizon,memory)):
            raise ValueError("Motion tracking limits must be positive and finite")
        self.speed_bound,self.horizon,self.memory=speed_bound,horizon,memory
        self.tracks=[]
        self.stamp=-math.inf

    @staticmethod
    def clusters(points):
        points=np.asarray(points,dtype=float).reshape((-1,3))
        if not len(points):return []
        pairs=cKDTree(points).query_pairs(1.15,output_type='ndarray')
        if len(pairs):
            rows=np.concatenate((pairs[:,0],pairs[:,1]))
            cols=np.concatenate((pairs[:,1],pairs[:,0]))
            graph=coo_matrix((np.ones(len(rows)),(rows,cols)),shape=(len(points),len(points)))
            _,labels=connected_components(graph,directed=False)
        else:
            labels=np.arange(len(points))
        result=[]
        for label in np.unique(labels):
            cloud=points[labels==label]
            # Large connected walls stay in the measured cloud, not a moving track.
            if np.max(np.ptp(cloud,axis=0))<=3.:
                result.append(cloud[np.linspace(0,len(cloud)-1,min(16,len(cloud)),dtype=int)])
        return result

    def update(self,points,stamp):
        if not math.isfinite(stamp) or stamp<=self.stamp:
            return
        self.stamp=stamp
        self.tracks=[t for t in self.tracks if stamp-t.stamp<=self.memory]
        clouds=self.clusters(points)
        centers=[np.mean(c,axis=0) for c in clouds]
        matched=set()
        if self.tracks and centers:
            predicted=np.asarray([t.center+t.velocity*(stamp-t.stamp) for t in self.tracks])
            costs=np.linalg.norm(predicted[:,None,:]-np.asarray(centers)[None,:,:],axis=2)
            for i,j in zip(*linear_sum_assignment(costs)):
                track=self.tracks[i]
                if costs[i,j]>1.2+self.speed_bound*(stamp-track.stamp):continue
                matched.add(j)
                track.history.append((stamp,centers[j]))
                while len(track.history)>2 and stamp-track.history[0][0]>1.:
                    track.history.popleft()
                if len(track.history)>=3 and stamp-track.history[0][0]>=.3:
                    times=np.asarray([s-stamp for s,_ in track.history])
                    samples=np.asarray([p for _,p in track.history])
                    velocity=np.linalg.lstsq(np.column_stack((times,np.ones(len(times)))),samples,rcond=None)[0][0]
                    speed=float(np.linalg.norm(velocity))
                    if speed>self.speed_bound:velocity*=self.speed_bound/speed
                    track.velocity=.5*track.velocity+.5*velocity
                track.center,track.points,track.stamp=centers[j],clouds[j],stamp
        for j,cloud in enumerate(clouds):
            if j not in matched:
                track=_Track(centers[j],cloud,stamp)
                track.history.append((stamp,centers[j]))
                self.tracks.append(track)
        # Measured geometry is never removed; only temporal hypotheses are bounded.
        self.tracks=sorted(self.tracks,key=lambda t:t.stamp,reverse=True)[:32]

    def prediction_points(self):
        samples=[]
        axes=np.vstack((np.zeros(3),np.eye(3),-np.eye(3)))
        for track in self.tracks:
            age=self.stamp-track.stamp
            speed=float(np.linalg.norm(track.velocity))
            if speed<.15 and age<.2:continue
            for future in np.arange(0.,self.horizon+.01,.5):
                # Unobserved motion is bounded briefly; never erase on one missed ray.
                uncertainty=.25+.15*future+self.speed_bound*age
                cloud=track.points+track.velocity*(age+future)
                samples.append((cloud[:,None,:]+axes[None,:,:]*uncertainty).reshape((-1,3)))
        return np.vstack(samples) if samples else np.empty((0,3))
