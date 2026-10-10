import math

from guga_evaluate.metrics import (
    RisingEdgeCounter,
    SeriesStats,
    nearest_path_error,
    path_curvatures,
    path_length,
    percentile,
    wrap_angle,
)


def test_wrap_angle():
    assert math.isclose(wrap_angle(3.0 * math.pi), math.pi, abs_tol=1e-12)
    assert math.isclose(wrap_angle(-3.0 * math.pi), -math.pi, abs_tol=1e-12)


def test_percentile_and_summary():
    values = [1.0, 2.0, 3.0, 4.0]
    assert percentile(values, 50.0) == 2.5
    summary = SeriesStats(values=values).summary()
    assert summary["count"] == 4
    assert math.isclose(summary["mean"], 2.5)
    assert math.isclose(summary["rmse"], math.sqrt(7.5))


def test_path_geometry():
    straight = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert path_length(straight) == 2.0
    assert path_curvatures(straight) == [0.0]

    error = nearest_path_error(0.5, 1.0, 0.0, straight)
    assert error is not None
    cross_track, heading, progress = error
    assert math.isclose(cross_track, 1.0)
    assert math.isclose(heading, 0.0)
    assert math.isclose(progress, 0.25)


def test_corner_curvature():
    values = path_curvatures([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    assert len(values) == 1
    assert math.isclose(values[0], math.sqrt(2.0), rel_tol=1e-12)


def test_rising_edge_counter_debounces_events():
    counter = RisingEdgeCounter(debounce_sec=0.5)
    assert counter.summary(enabled=True)["count"] is None
    assert counter.observe(False, 1.0) is False
    assert counter.observe(True, 1.1) is True
    assert counter.observe(True, 1.2) is False
    assert counter.observe(False, 1.3) is False
    assert counter.observe(True, 1.4) is False
    assert counter.observe(False, 1.5) is False
    assert counter.observe(True, 1.7) is True
    summary = counter.summary(enabled=True)
    assert summary["count"] == 2
    assert summary["active"] is True
