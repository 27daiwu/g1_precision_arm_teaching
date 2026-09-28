"""JSONL response records, including experimental waist observations."""
import json
import time
from pathlib import Path


class ResponseRecorder:
    def __init__(self, path, client):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.stream = Path(path).open('x', encoding='utf-8')
        self.client = client

    def record(self, state, q_des, phase):
        row = dict(timestamp=time.monotonic(), state_timestamp=state.timestamp,
                   phase=phase, backend='hardware' if self.client.real else 'simulation',
                   q_des=q_des.tolist(), q_actual=state.q.tolist(),
                   WAIST_COMMAND_REQUIRED='UNKNOWN',
                   WAIST_COMMAND_SENT=self.client.config['waist']['send_commands'],
                   WAIST_STATE_RESPONSE=state.waist_q.tolist(),
                   WAIST_HOLD_BEHAVIOR='UNKNOWN',
                   acquire_waist_q=self.client.acquire_waist_q.tolist())
        self.stream.write(json.dumps(row, allow_nan=False) + '\n')
        self.stream.flush()

    def close(self):
        self.stream.close()
