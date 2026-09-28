from pathlib import Path
import pytest
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from g1_dual_arm_teaching.utils.config import load_config


@pytest.fixture
def config():
    value = load_config(Path(__file__).resolve().parents[1] / 'configs')
    value['control']['frequency_hz'] = 100
    value['control']['acquire_ramp_s'] = 0.02
    value['control']['release_ramp_s'] = 0.02
    value['waist'].update(configuration='ACTIVE_3DOF', configuration_verified=True, active_joints=[12, 13, 14], locked_joints=[])
    value['control'].update(stable_state_s=0.02, release_observe_s=0.03)
    return value
