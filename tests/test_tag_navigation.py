"""Simulation-level state tests; no ROS/Gazebo process is needed."""
import os
import sys
import unittest

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'swerve_controller'))
from tag_navigation_core import RouteExecutor, RouteState, TagGraph


class TagNavigationSimulationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = os.path.join(os.path.dirname(__file__), '..', 'config', 'tag_graph.yaml')
        with open(path, encoding='utf-8') as stream:
            cls.graph = TagGraph(yaml.safe_load(stream)['tags'])

    def make_executor(self):
        executor = RouteExecutor(self.graph); executor.anchor(305); return executor

    def test_case_1_intermediate_tag_route_and_sequence(self):
        executor = self.make_executor()
        self.assertEqual(executor.start(407), [305, 405, 406, 407])
        self.assertEqual(executor.next_tag, 405)
        self.assertEqual(executor.expected_seen(405), 'advanced')
        self.assertEqual(executor.next_tag, 406)
        self.assertEqual(executor.expected_seen(406), 'advanced')
        self.assertEqual(executor.next_tag, 407)

    def test_case_2_near_coordinate_without_tag_is_approach_not_success(self):
        executor = self.make_executor(); executor.start(407)
        executor.state = RouteState.APPROACH_TAG  # Nav2 acquisition radius reached.
        self.assertNotEqual(executor.state, RouteState.ARRIVED)
        self.assertEqual(executor.next_tag, 405)

    def test_case_3_target_tag_detection_is_the_only_arrival_gate(self):
        executor = self.make_executor(); executor.start(407)
        for tag in (405, 406): self.assertEqual(executor.expected_seen(tag, offset_ok=True), 'advanced')
        self.assertEqual(executor.expected_seen(407, offset_ok=False), 'offset_out_of_tolerance')
        self.assertNotEqual(executor.state, RouteState.ARRIVED)
        self.assertEqual(executor.expected_seen(407, offset_ok=True), 'advanced')
        self.assertEqual(executor.state, RouteState.ARRIVED)

    def test_case_4_wrong_tag_is_explicit_and_can_replan(self):
        executor = self.make_executor(); executor.start(407)
        self.assertEqual(executor.expected_seen(407), 'wrong_tag')
        self.assertEqual(executor.state, RouteState.WRONG_TAG)
        self.assertEqual(executor.replan_from(407), [407])
        self.assertEqual(executor.state, RouteState.ARRIVED)

    def test_unanchored_rejects_global_mission(self):
        with self.assertRaisesRegex(RuntimeError, 'UNANCHORED'):
            RouteExecutor(self.graph).start(407)


if __name__ == '__main__':
    unittest.main()
