import numpy as np
import pytest
from g1_dual_arm_teaching.waypoint import JointWaypoint, WaypointStore


def test_waypoint_json_roundtrip(tmp_path):
    original = JointWaypoint("p1", np.arange(14), 2.0, 0.5)
    path = tmp_path / "p1.json"
    WaypointStore.save(path, original)
    loaded = WaypointStore.load(path)
    assert loaded.name == original.name
    np.testing.assert_array_equal(loaded.q_arm, original.q_arm)
    assert loaded.duration == 2.0


@pytest.mark.parametrize("q", [[float("nan")], [float("inf")], []])
def test_invalid_waypoint_rejected(q):
    with pytest.raises(ValueError):
        JointWaypoint("bad", q)


def test_legacy_split_arm_format():
    point = JointWaypoint.from_dict({"name": "p", "left_arm": {"q": [1] * 7}, "right_arm": {"q": [2] * 7}})
    np.testing.assert_array_equal(point.q_arm, [1] * 7 + [2] * 7)


@pytest.mark.parametrize('suffix', ['.json', '.yaml', '.yml'])
def test_store_formats(tmp_path, suffix):
    point = JointWaypoint('a', np.arange(14) * .01, None, .1, [.1, .2, .3])
    path = tmp_path / ('waypoint' + suffix)
    WaypointStore.save(path, point)
    loaded = WaypointStore.load(path)
    np.testing.assert_array_equal(loaded.q_arm, point.q_arm)
    np.testing.assert_array_equal(loaded.waist_q_reference, point.waist_q_reference)


@pytest.mark.parametrize('data', [
    {'name': 'bad', 'q_arm': [0] * 14, 'waist': {'mode': 'MOVE'}},
    {'name': 'bad', 'q_arm': [0] * 14, 'pose': [0] * 7},
    {'name': 'bad', 'left_arm': [0] * 6, 'right_arm': [0] * 8},
    {'name': 'bad', 'q_arm': [0] * 14, 'motion': {'duration': 0}},
    {'name': 'bad', 'q_arm': [0] * 14, 'motion': {'hold_time': -1}},
    {'name': 'bad', 'q_arm': [0] * 14, 'motion': {'duration': float('nan')}},
])
def test_invalid_format(data):
    with pytest.raises(ValueError):
        JointWaypoint.from_dict(data)
