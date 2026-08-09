import pytest

from delivery_evaluation.path_metrics import PathAccumulator, endpoint_error


def test_path_integrates_ordered_samples_and_rejects_jumps():
    path = PathAccumulator(maximum_jump_m=2.0)
    assert path.add(1, 0.0, 0.0)
    assert path.add(2, 1.0, 0.0)
    assert not path.add(3, 10.0, 0.0)
    assert path.add(4, 11.0, 0.0)
    assert path.length_m == pytest.approx(2.0)
    assert path.rejected_samples == 1


def test_path_rejects_duplicate_timestamp():
    path = PathAccumulator()
    assert path.add(10, 0.0, 0.0)
    assert not path.add(10, 0.1, 0.0)


def test_endpoint_error_supports_2d_and_3d():
    assert endpoint_error((0, 0, 5), (3, 4, 9), 2) == pytest.approx(5.0)
    assert endpoint_error((0, 0, 0), (2, 3, 6), 3) == pytest.approx(7.0)
