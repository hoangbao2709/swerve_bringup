#!/usr/bin/env python3
"""End-to-end, headless regression for the canonical map pipeline."""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "waretwin" / "backend"
import sys
sys.path.insert(0, str(BACKEND))
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "config.settings")
import django
django.setup()

from django.conf import settings
from twin.canonical_map import validate_canonical_layout
from twin.map_artifacts import build_revision_artifacts


FIXTURE = ROOT / "tools" / "fixtures" / "warehouse_pipeline_fixture.json"


class WarehousePipelineE2E(unittest.TestCase):
    def setUp(self):
        self.layout = json.loads(FIXTURE.read_text(encoding="utf-8"))

    def test_fixture_validation_and_artifacts(self):
        self.assertEqual(validate_canonical_layout(self.layout), [])
        with tempfile.TemporaryDirectory() as tmp:
            old_root = getattr(settings, "WARETWIN_ARTIFACT_ROOT", None)
            settings.WARETWIN_ARTIFACT_ROOT = Path(tmp) / "maps"
            try:
                staging, final, manifest = build_revision_artifacts(
                    self.layout, warehouse_id="E2E-WH", revision=7, published_version=1,
                )
                os.replace(staging, final)
                self.assertEqual(manifest["revision"], 7)
                for relative in ("canonical_map.json", "datamatrix_map.yaml", "tag_graph.yaml", "manifest.json", "gazebo/warehouse.world"):
                    self.assertTrue((final / relative).is_file(), relative)
                exported = json.loads((final / "canonical_map.json").read_text())
                self.assertEqual(exported["floors"][0]["id"], "F1")
                matrix = yaml.safe_load((final / "datamatrix_map.yaml").read_text())
                self.assertEqual([tag["tag_id"] for tag in matrix["markers"]], [1001, 1002, 1003, 1004, 1005, 1006])
                graph = yaml.safe_load((final / "tag_graph.yaml").read_text())
                self.assertEqual(graph["tags"]["1002"]["neighbors"], [1001, 1003, 1006])
                self.assertEqual(graph["edges"][2]["direction"], "forward")
                self.assertEqual(json.loads((final / "manifest.json").read_text())["revision"], 7)
                gz = shutil.which("gz")
                if gz is None:
                    self.skipTest("Gazebo gz executable is not installed")
                result = subprocess.run([gz, "sdf", "-k", str(final / "gazebo" / "warehouse.world")], capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stderr or result.stdout)
            finally:
                if old_root is None:
                    delattr(settings, "WARETWIN_ARTIFACT_ROOT")
                else:
                    settings.WARETWIN_ARTIFACT_ROOT = old_root

    def test_invalid_fixture_never_creates_artifacts(self):
        self.layout["navigation_tags"][0]["x"] = 99
        with tempfile.TemporaryDirectory() as tmp:
            old_root = getattr(settings, "WARETWIN_ARTIFACT_ROOT", None)
            settings.WARETWIN_ARTIFACT_ROOT = Path(tmp) / "maps"
            try:
                with self.assertRaises(ValueError):
                    # The exporter invoked by build_revision_artifacts performs
                    # the complete canonical validation before committing output.
                    build_revision_artifacts(self.layout, warehouse_id="E2E-WH", revision=8, published_version=2)
                self.assertFalse((Path(tmp) / "maps" / "E2E-WH" / "8").exists())
            finally:
                if old_root is None:
                    delattr(settings, "WARETWIN_ARTIFACT_ROOT")
                else:
                    settings.WARETWIN_ARTIFACT_ROOT = old_root


if __name__ == "__main__":
    unittest.main()
