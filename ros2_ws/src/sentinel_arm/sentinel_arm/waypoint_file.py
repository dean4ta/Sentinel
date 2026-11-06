# Waypoint trajectory file I/O for arm teach-and-repeat.
#
# Pure Python: no rclpy import, so it tests on the Mac without ROS.
# Files are YAML (PyYAML), one trajectory per file:
#
#   name: pick_place_demo
#   created: 2026-11-06T14:32:00+00:00
#   joint_names: [base, shoulder, elbow, clamp]
#   waypoints:
#     - name: home
#       position: [0.00, 0.50, -0.30, 0.00]
#       delay_after: 0.5
#
# position order is fixed to JOINT_NAMES; load validates it.

import re
from datetime import datetime, timezone

import yaml

JOINT_NAMES = ['base', 'shoulder', 'elbow', 'clamp']
NUM_JOINTS = len(JOINT_NAMES)

# Waypoint names: non-empty, [A-Za-z0-9_.-]+. Same rule as the srv docs.
_NAME_RE = re.compile(r'^[A-Za-z0-9_.-]+$')


class WaypointError(Exception):
    """Invalid waypoint data or edit command."""


class Waypoint:
    """One taught pose: joint angles (rad) + optional hold after moving."""

    def __init__(self, name, position, delay_after=0.0):
        if not isinstance(name, str) or not _NAME_RE.match(name):
            raise WaypointError(f'invalid waypoint name {name!r}')
        if len(position) != NUM_JOINTS:
            raise WaypointError(
                f'{name!r}: expected {NUM_JOINTS} joint values, '
                f'got {len(position)}')
        delay_after = float(delay_after)
        if delay_after < 0:
            raise WaypointError(f'{name!r}: negative delay_after')
        self.name = name
        self.position = [float(x) for x in position]
        self.delay_after = delay_after

    def to_dict(self):
        d = {'name': self.name, 'position': self.position}
        if self.delay_after:
            d['delay_after'] = self.delay_after
        return d

    @classmethod
    def from_dict(cls, d):
        if not isinstance(d, dict):
            raise WaypointError(f'waypoint entry not a mapping: {d!r}')
        try:
            name = d['name']
            position = d['position']
        except KeyError as e:
            raise WaypointError(f'waypoint missing {e}')
        return cls(name, position, d.get('delay_after', 0.0))


class Trajectory:
    """In-memory waypoint sequence + YAML file load/save."""

    def __init__(self, name='unnamed'):
        self.name = name
        self.created = datetime.now(timezone.utc).isoformat(timespec='seconds')
        self.waypoints = []

    # --- edits (all raise WaypointError on bad input) ---

    def _index(self, name):
        for i, wp in enumerate(self.waypoints):
            if wp.name == name:
                return i
        raise WaypointError(f'no waypoint named {name!r}')

    def add(self, waypoint):
        if any(wp.name == waypoint.name for wp in self.waypoints):
            raise WaypointError(f'duplicate waypoint name {waypoint.name!r}')
        self.waypoints.append(waypoint)

    def remove(self, name):
        del self.waypoints[self._index(name)]

    def rename(self, old, new):
        self._index(old)
        Waypoint(new, [0.0] * NUM_JOINTS)  # name check only
        if any(wp.name == new for wp in self.waypoints):
            raise WaypointError(f'duplicate waypoint name {new!r}')
        self.waypoints[self._index(old)].name = new

    def reorder(self, name, index):
        index = int(index)
        if not 0 <= index < len(self.waypoints):
            raise WaypointError(f'reorder index {index} out of range')
        wp = self.waypoints.pop(self._index(name))
        self.waypoints.insert(index, wp)

    def clear(self):
        self.waypoints = []

    def names(self):
        return [wp.name for wp in self.waypoints]

    # --- file I/O ---

    def save_file(self, path):
        doc = {
            'name': self.name,
            'created': self.created,
            'joint_names': JOINT_NAMES,
            'waypoints': [wp.to_dict() for wp in self.waypoints],
        }
        # ponytail: plain write, no atomic-rename dance; a crash mid-write
        # can corrupt the file, add tempfile+rename if that ever bites.
        with open(path, 'w') as f:
            yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None)

    @classmethod
    def load_file(cls, path):
        with open(path) as f:
            doc = yaml.safe_load(f)
        if not isinstance(doc, dict):
            raise WaypointError(f'{path}: not a mapping at top level')
        header = doc.get('joint_names')
        if header != JOINT_NAMES:
            raise WaypointError(
                f'{path}: joint_names {header!r}, expected {JOINT_NAMES!r}')
        traj = cls(doc.get('name', 'unnamed'))
        traj.created = doc.get('created', traj.created)
        for entry in doc.get('waypoints', []):
            traj.add(Waypoint.from_dict(entry))
        return traj
