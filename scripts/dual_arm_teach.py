#!/usr/bin/env python3
"""Arm SDK dual arm kinesthetic teaching entry."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from g1_dual_arm_teaching.dual_arm_teach import main

if __name__ == '__main__':
    raise SystemExit(main())
