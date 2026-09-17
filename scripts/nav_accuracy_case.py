#!/usr/bin/env python3
"""One fresh-session Nav2 accuracy case for run_acceptance.py."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import accuracy_benchmark


if __name__ == '__main__':
    sys.argv = [sys.argv[0], '--repeats', '1', '--timeout', '120',
                '--outdir', os.environ['ACCEPTANCE_CASE_DIR']]
    raise SystemExit(accuracy_benchmark.main())
