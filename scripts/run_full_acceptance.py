#!/usr/bin/env python3
"""Stable entry point for the resumable fresh-session acceptance runner."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_acceptance import main


if __name__ == '__main__':
    raise SystemExit(main())
