#!/usr/bin/env python3
"""Compatibility entrypoint for the RMUC test publisher."""

from pathlib import Path
import runpy


if __name__ == "__main__":
    target = Path(__file__).resolve().parent / "used" / "rmuc_test_publisher.py"
    runpy.run_path(str(target), run_name="__main__")