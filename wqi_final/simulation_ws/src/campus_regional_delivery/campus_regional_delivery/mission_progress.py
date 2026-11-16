"""Bound repeated movement without improving the accepted route's remaining length."""
import math


class RouteProgressWatchdog:
    def __init__(self,start,timeout=60.,minimum_progress=.5):
        self.timeout,self.minimum_progress=float(timeout),float(minimum_progress)
        if not all(math.isfinite(v) for v in (start,self.timeout,self.minimum_progress)) or self.timeout<0 or self.minimum_progress<=0:
            raise ValueError("Progress clock/limits must be finite and nonnegative")
        self.sample=(None,float(start))

    def observe(self,distance,now,minimum_remaining=0.):
        # Nav2 can publish the previous blackboard path before the new plan
        # arrives. Such a remainder cannot satisfy this goal's geometric bound.
        if (not all(math.isfinite(v) for v in (distance,now,minimum_remaining))
                or distance<=0 or minimum_remaining<0 or distance+.5<minimum_remaining):
            return False
        best,last=self.sample
        if best is None or distance<=best-self.minimum_progress:
            self.sample=(distance,float(now))
        return True

    def expired(self,now):
        return self.timeout>0 and now-self.sample[1]>self.timeout
