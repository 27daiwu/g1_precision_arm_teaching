#!/usr/bin/env python3
"""Explicit --real is required for any hardware connection."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from g1_dual_arm_teaching.cli import main

if __name__ == '__main__':
    raise SystemExit(main('demo'))
