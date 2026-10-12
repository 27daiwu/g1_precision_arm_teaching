"""Offline metadata compatibility reader; never imports transport or commands."""
import json
from pathlib import Path
import uuid

import numpy as np

from ..utils.naming import read_log_timestamp


def load_trajectory(path):
    """Read schema 1/2 pairs without renaming or rewriting legacy artifacts."""
    path = Path(path)
    metadata_path = path.with_suffix('.json')
    with metadata_path.open(encoding='utf-8') as stream:
        metadata = json.load(stream)
    if metadata.get('schema_version', 1) not in (1, 2):
        raise ValueError('unsupported trajectory schema')
    with np.load(path.with_suffix('.npz'), allow_pickle=False) as data:
        arrays = {key: data[key].copy() for key in data.files}
    q = arrays['q_measured']
    if q.ndim != 2 or q.shape[1] != 14 or not np.isfinite(q).all():
        raise ValueError('invalid measured trajectory shape/data')
    metadata.setdefault('trajectory_id', uuid.uuid5(uuid.NAMESPACE_URL,
                        f'g1-legacy-trajectory:{metadata_path.name}').hex)
    metadata.setdefault('session_id', metadata.get('session'))
    metadata.setdefault('segment_index', metadata.get('index'))
    metadata.setdefault('previous_segment_id', None)
    metadata.setdefault('start_q', q[0].tolist() if len(q) else None)
    metadata.setdefault('end_q', q[-1].tolist() if len(q) else None)
    metadata.setdefault('connection_delta_q', None)
    metadata.setdefault('connection_max_abs_delta_q', None)
    instant = read_log_timestamp(metadata_path)
    metadata.setdefault('created_at', instant.isoformat() if instant else None)
    metadata.setdefault('recording_started_at', None)  # Not recoverable from old monotonic times.
    metadata.setdefault('display_name', metadata_path.stem)
    return metadata, arrays
