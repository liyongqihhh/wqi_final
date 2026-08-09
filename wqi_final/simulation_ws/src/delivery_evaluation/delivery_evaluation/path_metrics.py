import math


class PathAccumulator:
    def __init__(self, maximum_jump_m: float = 2.0) -> None:
        self.maximum_jump_m = float(maximum_jump_m)
        if not math.isfinite(self.maximum_jump_m) or self.maximum_jump_m <= 0.0:
            raise ValueError("maximum_jump_m must be positive")
        self.reset()

    def reset(self) -> None:
        self.length_m = 0.0
        self.last_stamp_ns = None
        self.last_position = None
        self.accepted_samples = 0
        self.rejected_samples = 0

    def add(self, stamp_ns: int, x: float, y: float, z: float = 0.0) -> bool:
        stamp = int(stamp_ns)
        position = (float(x), float(y), float(z))
        if stamp < 0 or not all(math.isfinite(value) for value in position):
            self.rejected_samples += 1
            return False
        if self.last_stamp_ns is not None and stamp <= self.last_stamp_ns:
            self.rejected_samples += 1
            return False
        if self.last_position is not None:
            distance = math.dist(self.last_position, position)
            if distance > self.maximum_jump_m:
                self.rejected_samples += 1
                self.last_stamp_ns = stamp
                self.last_position = position
                return False
            self.length_m += distance
        self.last_stamp_ns = stamp
        self.last_position = position
        self.accepted_samples += 1
        return True


def endpoint_error(position, target, dimensions: int = 3) -> float:
    count = int(dimensions)
    if count not in (2, 3):
        raise ValueError("dimensions must be 2 or 3")
    current = tuple(float(value) for value in position[:count])
    expected = tuple(float(value) for value in target[:count])
    if len(current) != count or len(expected) != count:
        raise ValueError("position and target have insufficient dimensions")
    if not all(math.isfinite(value) for value in (*current, *expected)):
        raise ValueError("position and target must be finite")
    return math.dist(current, expected)
