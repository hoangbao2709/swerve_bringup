#!/usr/bin/env python3
"""Consistent SQLite backup/restore helpers for WareTwin runtime data."""
from __future__ import annotations

import argparse
from contextlib import closing
import fcntl
import os
from pathlib import Path
import sqlite3
import tempfile
from datetime import datetime, timezone


def _connect_readonly(path: Path):
    return sqlite3.connect(f'{path.resolve().as_uri()}?mode=ro', uri=True, timeout=30)


def _integrity_check(connection: sqlite3.Connection) -> None:
    result = connection.execute('PRAGMA integrity_check').fetchone()
    if not result or result[0] != 'ok':
        raise RuntimeError('SQLite integrity_check failed; database was not replaced')


def backup_database(database: str | Path, backup_dir: str | Path) -> Path | None:
    """Create an immutable consistent snapshot; never overwrite a prior backup."""
    source = Path(database).expanduser().resolve()
    if not source.exists():
        return None
    if not source.is_file():
        raise RuntimeError(f'database path is not a regular file: {source}')
    target_dir = Path(backup_dir).expanduser().resolve()
    target_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    target_info = target_dir.stat()
    if target_info.st_uid != os.geteuid() or target_info.st_mode & 0o077:
        raise RuntimeError('backup directory must be owned by the current user and private (mode 0700)')
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    final_path = target_dir / f'waretwin-db-{timestamp}.sqlite3'
    fd, temp_name = tempfile.mkstemp(prefix='.waretwin-db-', suffix='.tmp', dir=target_dir)
    os.close(fd)
    temp_path = Path(temp_name)
    try:
        with closing(_connect_readonly(source)) as src:
            dst = sqlite3.connect(temp_path, timeout=30)
            try:
                src.backup(dst)
                _integrity_check(dst)
                dst.commit()
            finally:
                dst.close()
        os.chmod(temp_path, 0o600)
        with temp_path.open('rb') as stream:
            os.fsync(stream.fileno())
        # Timestamp collision is extraordinarily unlikely, but never replace.
        if final_path.exists():
            raise FileExistsError(f'refusing to overwrite existing backup: {final_path}')
        os.link(temp_path, final_path)
        temp_path.unlink()
        directory_fd = os.open(target_dir, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        return final_path
    finally:
        temp_path.unlink(missing_ok=True)


def restore_database(backup: str | Path, database: str | Path) -> Path:
    """Validate backup fully, then atomically replace the stopped runtime DB."""
    source = Path(backup).expanduser().resolve(strict=True)
    destination = Path(database).expanduser().resolve()
    if source == destination:
        raise ValueError('backup and destination database must be different files')
    if not source.is_file():
        raise RuntimeError(f'backup is not a regular file: {source}')
    destination.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    fd, temp_name = tempfile.mkstemp(prefix=f'.{destination.name}.restore-', dir=destination.parent)
    os.close(fd)
    temp_path = Path(temp_name)
    try:
        with closing(_connect_readonly(source)) as src:
            dst = sqlite3.connect(temp_path, timeout=30)
            try:
                _integrity_check(src)
                src.backup(dst)
                _integrity_check(dst)
                dst.commit()
            finally:
                dst.close()
        os.chmod(temp_path, 0o600)
        with temp_path.open('rb') as stream:
            os.fsync(stream.fileno())
        os.replace(temp_path, destination)
        directory_fd = os.open(destination.parent, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        return destination
    finally:
        temp_path.unlink(missing_ok=True)


def _exclusive_runtime_lock() -> object:
    state_home = Path(os.environ.get('XDG_STATE_HOME', Path.home() / '.local/state')).expanduser()
    if not state_home.is_absolute():
        state_home = Path.home() / '.local/state'
    lock_dir = state_home / 'waretwin'
    lock_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    if lock_dir.is_symlink() or lock_dir.stat().st_uid != os.geteuid():
        raise RuntimeError('runtime state directory must be a real directory owned by the current user')
    os.chmod(lock_dir, 0o700)
    lock = (lock_dir / 'stack.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock.close()
        raise RuntimeError('Stop WareTwin before restoring or creating an offline backup') from exc
    return lock


def main() -> int:
    state_home = Path(os.environ.get('XDG_STATE_HOME', Path.home() / '.local/state')).expanduser()
    if not state_home.is_absolute():
        state_home = Path.home() / '.local/state'
    runtime_dir = Path(os.environ.get('WARETWIN_RUNTIME_DIR') or state_home / 'waretwin').expanduser()
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    backup = commands.add_parser('backup', help='create a consistent SQLite snapshot')
    backup.add_argument('--database', default=os.environ.get('WARETWIN_DATABASE_PATH', runtime_dir / 'db.sqlite3'))
    backup.add_argument('--backup-dir', default=runtime_dir / 'backups')
    restore = commands.add_parser('restore', help='restore a snapshot with a pre-restore backup')
    restore.add_argument('--database', default=os.environ.get('WARETWIN_DATABASE_PATH', runtime_dir / 'db.sqlite3'))
    restore.add_argument('--backup', required=True)
    restore.add_argument('--backup-dir', default=runtime_dir / 'backups')
    args = parser.parse_args()
    try:
        lock = _exclusive_runtime_lock()
        try:
            if args.command == 'backup':
                path = backup_database(args.database, args.backup_dir)
                if path is None:
                    raise RuntimeError(f'database does not exist: {args.database}')
                print(path)
            else:
                prior = backup_database(args.database, args.backup_dir)
                path = restore_database(args.backup, args.database)
                print(f'restored {path}; pre-restore snapshot: {prior or "database did not previously exist"}')
        finally:
            lock.close()
        return 0
    except (OSError, ValueError, RuntimeError, sqlite3.Error) as exc:
        parser.exit(1, f'database operation failed: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
