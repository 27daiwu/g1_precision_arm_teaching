"""Windows-safe zoned timestamp filenames and legacy timestamp parsing."""
from datetime import datetime
from pathlib import Path
import re
from zoneinfo import ZoneInfo


def wall_now():
    return datetime.now(ZoneInfo('Asia/Shanghai'))


def filename_timestamp(instant):
    return instant.strftime('%Y-%m-%dT%H-%M-%S.%f%z')


def timestamped_path(directory, prefix, suffix, now=None):
    directory = Path(directory)
    instant = now or wall_now()
    stamp = filename_timestamp(instant)
    stem = f'{prefix}_{stamp}'
    candidate = directory / f'{stem}{suffix}'
    serial = 1
    while candidate.exists():
        candidate = directory / f'{stem}_{serial}{suffix}'
        serial += 1
    return candidate


def read_log_timestamp(path):
    name = Path(path).stem
    match = re.search(r'\d{4}-\d{2}-\d{2}T\d{2}-\d{2}-\d{2}\.\d{6}[+-]\d{4}', name)
    if match:
        try:
            return datetime.strptime(match.group(), '%Y-%m-%dT%H-%M-%S.%f%z')
        except ValueError:
            return None
    legacy = re.search(r'(?:^|_)(\d{16,20})(?:_|$)', name)
    if legacy is None:
        return None
    value = legacy.group(1)
    try:
        if value.isdigit():
            return datetime.fromtimestamp(int(value) / 1e9, ZoneInfo('Asia/Shanghai'))
        return datetime.strptime(value, '%Y-%m-%dT%H-%M-%S.%f%z')
    except (ValueError, OSError, OverflowError):
        return None
