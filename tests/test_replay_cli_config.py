import sys
import types

import pytest

from g1_dual_arm_teaching.trajectory.replay_permission import ReplayPermission
from g1_dual_arm_teaching.utils.hardware import resolve_network_interface
from g1_dual_arm_teaching.trajectory.replay_c2 import RelativeReplaySpec


def test_internal_replay_config_loads_offline_bounds_without_hardware_gains():
    config = ReplayPermission('configs/replay.yaml')
    assert config.allowed is False
    assert config.gains is None
    assert config.limits.velocity == 2.0
    assert config.replay_mode == 'relative'
    assert config.manifest['candidate']['start_index'] == 1033
    assert config.manifest['candidate']['end_index'] == 1093
    assert config.manifest['candidate']['motion_scale'] == 1.0
    assert config.speed == 0.25


def test_relative_spec_accepts_explicit_half_open_segment_and_scale(tmp_path):
    import json
    import numpy as np
    from g1_dual_arm_teaching.trajectory.replay import ReplayLimits

    timestamps = np.arange(8, dtype=float) * 0.02
    q = np.zeros((8, 14))
    arrays = dict(timestamp=timestamps, cycle_timestamp=timestamps + .001,
                  q_measured=q, dq_measured=q.copy(), q_ref=q.copy(),
                  waist_q=np.zeros((8, 3)), torso_roll_pitch=np.zeros((8, 2)),
                  arm_motor_ids=np.arange(15, 29), waist_motor_ids=np.arange(12, 15))
    path = tmp_path / 'recording.npz'
    np.savez(path, **arrays)
    path.with_suffix('.json').write_text(json.dumps(dict(
        sample_count=8, duration_s=float(timestamps[-1]), data_integrity='PASS')))
    limits = ReplayLimits((-3.14,) * 14, (3.14,) * 14, 2., 25., 100., .1,
                          100., 100., 1., .1, 300., 100.)
    spec = RelativeReplaySpec.load(path, limits, .25, jerk_limit=1e6,
                                   jerk_jump_limit=1e6, motion_scale=1.,
                                   start_index=2, end_index=8)
    assert spec.start_index == 2 and spec.end_index == 8
    assert spec.motion_scale == 1.


def test_interface_resolution_prefers_explicit_then_fixed_config():
    assert resolve_network_interface({'robot': {'network_interface': 'eth0'}}, 'enp2s0') == 'enp2s0'
    assert resolve_network_interface({'robot': {'network_interface': 'eth0'}}) == 'eth0'


def test_interface_detection_requires_exactly_one_live_ipv4_candidate(monkeypatch):
    import socket

    class Address:
        def __init__(self, family, address):
            self.family, self.address = family, address

    module = types.ModuleType('psutil')
    module.net_if_addrs = lambda: {
        'lo': [Address(socket.AF_INET, '127.0.0.1')],
        'eth0': [Address(socket.AF_INET, '192.0.2.10')],
        'wlan0': [Address(socket.AF_INET, '192.0.2.11')],
    }
    module.net_if_stats = lambda: {
        name: types.SimpleNamespace(isup=True) for name in ('lo', 'eth0', 'wlan0')
    }
    monkeypatch.setitem(sys.modules, 'psutil', module)
    with pytest.raises(RuntimeError, match='cannot uniquely identify'):
        resolve_network_interface({'robot': {'network_interface': None}})
