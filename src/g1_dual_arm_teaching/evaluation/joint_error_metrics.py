"""Joint encoder-space errors; these do not measure external Cartesian accuracy."""
import numpy as np


def _samples(value):
    data = np.asarray(value, dtype=float)
    if data.ndim != 2 or data.shape[1] != 14 or data.shape[0] == 0 or not np.all(np.isfinite(data)):
        raise ValueError('expected non-empty finite N x 14 samples')
    return data


def joint_error_metrics(actual, desired):
    actual, desired = _samples(actual), _samples(desired)
    if actual.shape != desired.shape:
        raise ValueError('samples must have matching dimensions')
    error = actual - desired
    return {'bias_rad': np.mean(error, axis=0),
            'mae_rad': np.mean(np.abs(error), axis=0),
            'rmse_rad': np.sqrt(np.mean(error ** 2, axis=0)),
            'max_abs_rad': np.max(np.abs(error), axis=0)}


def repeatability_metrics(settled_trial_positions):
    values = _samples(settled_trial_positions)
    if len(values) < 2:
        raise ValueError('at least two settled trials required')
    return {'std_rad': np.std(values, axis=0, ddof=1),
            'peak_to_peak_rad': np.ptp(values, axis=0)}
