"""ROS-independent tag graph planning and executor state machine."""
from __future__ import annotations

import heapq
import math


class TagGraph:
    def __init__(self, tags):
        self.tags = {int(k): v for k, v in tags.items()}
        for tag_id, tag in self.tags.items():
            for neighbor in tag['neighbors']:
                if int(neighbor) not in self.tags:
                    raise ValueError(f'tag {tag_id} refers to unknown neighbor {neighbor}')

    def route(self, start, target):
        start, target = int(start), int(target)
        if start not in self.tags or target not in self.tags:
            raise ValueError('start or target tag is not in the tag graph')
        frontier, cost, parent = [(0.0, start)], {start: 0.0}, {start: None}
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == target:
                out = []
                while current is not None:
                    out.append(current); current = parent[current]
                return list(reversed(out))
            here = self.tags[current]
            for nxt in here['neighbors']:
                nxt = int(nxt); there = self.tags[nxt]
                new_cost = cost[current] + math.hypot(here['x'] - there['x'], here['y'] - there['y'])
                if new_cost < cost.get(nxt, float('inf')):
                    cost[nxt], parent[nxt] = new_cost, current
                    heuristic = math.hypot(there['x'] - self.tags[target]['x'], there['y'] - self.tags[target]['y'])
                    heapq.heappush(frontier, (new_cost + heuristic, nxt))
        raise ValueError(f'no allowed corridor from {start} to {target}')


class RouteState:
    UNANCHORED = 'UNANCHORED'
    DEAD_RECKONING = 'DEAD_RECKONING'
    NAVIGATING_SEGMENT = 'NAVIGATING_SEGMENT'
    APPROACH_TAG = 'APPROACH_TAG'
    ARRIVED = 'ARRIVED'
    TAG_ACQUIRE_FAILED = 'TAG_ACQUIRE_FAILED'
    WRONG_TAG = 'WRONG_TAG'


class RouteExecutor:
    """Tag confirmation gate.  A coordinate/Nav2 result can never advance it."""
    def __init__(self, graph):
        self.graph = graph; self.current_tag = None; self.target_tag = None
        self.route = []; self.index = 0; self.state = RouteState.UNANCHORED

    @property
    def next_tag(self):
        return self.route[self.index + 1] if self.index + 1 < len(self.route) else None

    def anchor(self, tag_id):
        if int(tag_id) not in self.graph.tags: raise ValueError('detected tag is not in graph')
        self.current_tag = int(tag_id)
        if self.state == RouteState.UNANCHORED: self.state = RouteState.DEAD_RECKONING

    def start(self, target_tag):
        if self.current_tag is None: raise RuntimeError('UNANCHORED: a valid tag anchor is required')
        self.target_tag = int(target_tag); self.route = self.graph.route(self.current_tag, self.target_tag); self.index = 0
        self.state = RouteState.ARRIVED if self.current_tag == self.target_tag else RouteState.NAVIGATING_SEGMENT
        return self.route

    def expected_seen(self, detected_tag, offset_ok=True):
        detected_tag = int(detected_tag)
        if not offset_ok: return 'offset_out_of_tolerance'
        if self.next_tag is None:
            return 'not_expecting_tag'
        if detected_tag != self.next_tag:
            self.state = RouteState.WRONG_TAG
            return 'wrong_tag'
        self.current_tag = detected_tag; self.index += 1
        self.state = RouteState.ARRIVED if detected_tag == self.target_tag else RouteState.NAVIGATING_SEGMENT
        return 'advanced'

    def replan_from(self, detected_tag):
        self.anchor(detected_tag)
        return self.start(self.target_tag)
