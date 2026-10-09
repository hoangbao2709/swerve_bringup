#!/usr/bin/env python3
"""Reject private or mutable development data from an installed web package."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys


FORBIDDEN_NAMES = {'.env', '.git', '.venv', 'node_modules', 'secrets.json', 'db.sqlite3'}
FORBIDDEN_SUFFIXES = ('.sqlite', '.sqlite3', '.sqlite3-wal', '.sqlite3-shm',
                      '.pgm', '.posegraph', '.posegraph.tar.gz')


def violations(share: Path) -> list[Path]:
    root = share.resolve(strict=True)
    if not root.is_dir():
        raise ValueError(f'package share path is not a directory: {root}')
    found = []
    for item in root.rglob('*'):
        relative = item.relative_to(root)
        if any(part in FORBIDDEN_NAMES for part in relative.parts):
            found.append(relative)
            continue
        if item.name.endswith(FORBIDDEN_SUFFIXES):
            found.append(relative)
            continue
        if item.is_symlink():
            try:
                item.resolve(strict=True).relative_to(root)
            except (OSError, ValueError):
                found.append(relative)
    return found


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('share', type=Path, help='installed share/waretwin_web directory')
    args = parser.parse_args()
    try:
        invalid = violations(args.share)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    if invalid:
        for path in invalid:
            print(f'FORBIDDEN release artifact: {path}', file=sys.stderr)
        return 1
    print(f'PASS: installed package contains no database, secret, node_modules, venv, or local map artifacts ({args.share.resolve()})')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
