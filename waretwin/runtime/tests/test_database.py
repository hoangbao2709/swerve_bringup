import os
from pathlib import Path
import sqlite3
import tempfile
import unittest

from waretwin.runtime.database import backup_database, restore_database


class RuntimeDatabaseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='waretwin-db-test-')
        self.root = Path(self.temp.name)
        self.database = self.root / 'runtime.sqlite3'
        with sqlite3.connect(self.database) as connection:
            connection.execute('CREATE TABLE records (id INTEGER PRIMARY KEY, value TEXT NOT NULL)')
            connection.execute('INSERT INTO records(value) VALUES (?)', ('preserve-me',))

    def tearDown(self):
        self.temp.cleanup()

    def rows(self):
        with sqlite3.connect(self.database) as connection:
            return connection.execute('SELECT value FROM records ORDER BY id').fetchall()

    def test_snapshot_and_rollback_preserve_pre_migration_records(self):
        backup = backup_database(self.database, self.root / 'backups')
        self.assertIsNotNone(backup)
        self.assertEqual(self.rows(), [('preserve-me',)])
        self.assertEqual(os.stat(backup).st_mode & 0o777, 0o600)

        # Simulate a partially applied migration that corrupts a user's row.
        with sqlite3.connect(self.database) as connection:
            connection.execute('UPDATE records SET value=?', ('partial-migration',))
            connection.execute('CREATE TABLE half_migrated (id INTEGER)')
        restore_database(backup, self.database)
        self.assertEqual(self.rows(), [('preserve-me',)])
        with sqlite3.connect(self.database) as connection:
            self.assertIsNone(connection.execute(
                "SELECT name FROM sqlite_master WHERE name='half_migrated'").fetchone())

    def test_invalid_backup_never_replaces_existing_database(self):
        invalid = self.root / 'not-a-database.sqlite3'
        invalid.write_bytes(b'not sqlite')
        before = self.database.read_bytes()
        with self.assertRaises(sqlite3.DatabaseError):
            restore_database(invalid, self.database)
        self.assertEqual(self.database.read_bytes(), before)
        self.assertEqual(self.rows(), [('preserve-me',)])

    def test_missing_database_does_not_create_an_empty_backup(self):
        self.assertIsNone(backup_database(self.root / 'absent.sqlite3', self.root / 'backups'))
        self.assertFalse((self.root / 'backups').exists())

    def test_backup_refuses_a_group_or_world_accessible_directory(self):
        shared = self.root / 'shared-backups'
        shared.mkdir(mode=0o755)
        shared.chmod(0o755)
        with self.assertRaisesRegex(RuntimeError, 'backup directory must be owned'):
            backup_database(self.database, shared)
        self.assertEqual(shared.stat().st_mode & 0o777, 0o755)


if __name__ == '__main__':
    unittest.main()
