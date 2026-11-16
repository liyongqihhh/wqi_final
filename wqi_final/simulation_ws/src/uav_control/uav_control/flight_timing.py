"""Simulation deadlines with an independent wall-clock loss-of-progress bound."""
import math


def recent_cloud_candidates(entries,after_ns,now_ns,now_wall):
    """Keep exact-stamped alternatives when the newest cloud precedes its TF."""
    return sorted((e for e in entries if e[0]>max(0,after_ns)
        and -.25<=(now_ns-e[0])/1e9<=.6 and 0<=now_wall-e[2]<=30.),
        key=lambda e:e[0],reverse=True)


def replan_clock(wall_now,simulation_now,dynamic):
    """Do not rerun simulation planners eight times faster at RTF 0.125."""
    return simulation_now if dynamic else wall_now


def flight_deadline_expired(wall_elapsed,sim_elapsed,timeout,dynamic,wall_watchdog):
    if not all(math.isfinite(v) for v in (wall_elapsed,sim_elapsed,timeout,wall_watchdog)):
        return True
    if timeout<=0 or wall_watchdog<=0 or wall_elapsed<0 or sim_elapsed<-.25:
        return True
    return (sim_elapsed>=timeout or wall_elapsed>=wall_watchdog) if dynamic else wall_elapsed>=timeout
