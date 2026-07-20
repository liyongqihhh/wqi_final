from dataclasses import dataclass
import math
from typing import Callable


MAX_EXACT_STOPS = 10
_COST_EPSILON = 1e-9


@dataclass(frozen=True)
class RoutePlan:
    order: tuple[int, ...]
    total_cost: float


def _finite_nonnegative(value: float, description: str) -> float:
    cost = float(value)
    if not math.isfinite(cost) or cost < 0.0:
        raise ValueError(f"{description} must be finite and non-negative")
    return cost


def _is_better(candidate, current) -> bool:
    if current is None:
        return True
    candidate_cost, candidate_order = candidate
    current_cost, current_order = current
    if candidate_cost < current_cost - _COST_EPSILON:
        return True
    return (
        abs(candidate_cost - current_cost) <= _COST_EPSILON
        and candidate_order < current_order
    )


def optimize_visit_order(
    stop_count: int,
    start_cost: Callable[[int], float],
    transition_cost: Callable[[int, int], float],
    return_cost: Callable[[int], float] | None = None,
) -> RoutePlan:
    """Find the exact shortest order for at most MAX_EXACT_STOPS stops."""
    count = int(stop_count)
    if count < 0:
        raise ValueError("stop_count cannot be negative")
    if count > MAX_EXACT_STOPS:
        raise ValueError(
            f"Exact route optimization supports at most {MAX_EXACT_STOPS} stops"
        )
    if count == 0:
        return RoutePlan((), 0.0)

    starts = [
        _finite_nonnegative(start_cost(index), f"start cost for stop {index}")
        for index in range(count)
    ]
    transitions = [
        [
            0.0
            if origin == destination
            else _finite_nonnegative(
                transition_cost(origin, destination),
                f"transition cost {origin}->{destination}",
            )
            for destination in range(count)
        ]
        for origin in range(count)
    ]
    returns = None
    if return_cost is not None:
        returns = [
            _finite_nonnegative(
                return_cost(index), f"return cost for stop {index}"
            )
            for index in range(count)
        ]

    # (visited bit mask, final stop) -> (cost, ordered stop indices)
    states = {
        (1 << index, index): (starts[index], (index,))
        for index in range(count)
    }
    for mask in range(1, 1 << count):
        for final_stop in range(count):
            state = states.get((mask, final_stop))
            if state is None:
                continue
            cost, order = state
            for next_stop in range(count):
                bit = 1 << next_stop
                if mask & bit:
                    continue
                candidate = (
                    cost + transitions[final_stop][next_stop],
                    order + (next_stop,),
                )
                key = (mask | bit, next_stop)
                if _is_better(candidate, states.get(key)):
                    states[key] = candidate

    full_mask = (1 << count) - 1
    best = None
    for final_stop in range(count):
        cost, order = states[(full_mask, final_stop)]
        if returns is not None:
            cost += returns[final_stop]
        candidate = (cost, order)
        if _is_better(candidate, best):
            best = candidate
    return RoutePlan(best[1], best[0])
