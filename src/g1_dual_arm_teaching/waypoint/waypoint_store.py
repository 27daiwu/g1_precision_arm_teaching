"""Human-readable JSON/YAML waypoint persistence; never pickle."""
import json
from pathlib import Path
import yaml
from .waypoint import JointWaypoint


class WaypointStore:
    @staticmethod
    def save(path, waypoint):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        data = waypoint.to_dict()
        text = yaml.safe_dump(data, sort_keys=False) if path.suffix.lower() in ('.yaml', '.yml') else json.dumps(data, indent=2) + '\n'
        path.write_text(text, encoding='utf-8')

    @staticmethod
    def load(path):
        path = Path(path)
        with path.open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream) if path.suffix.lower() in ('.yaml', '.yml') else json.load(stream)
        return JointWaypoint.from_dict(data)
