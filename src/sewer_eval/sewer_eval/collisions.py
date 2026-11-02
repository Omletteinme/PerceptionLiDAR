"""Collision counting from Gazebo contact messages.

CUSTOM code (ours), pure Python, no ROS imports. The robot carries contact sensors (SIMULATED by Gazebo); every
message lists pairs of colliding collision shapes with scoped names such as 'sewer_robot::base_link::base_link_collision'
and 'walls::link::c3'. The floor is not an obstacle, so contacts with it are ignored. Contacts arrive many times a second
while the robot scrapes along a wall, so a COLLISION is counted per obstacle model when it touches after `gap` seconds
without touching it (a long scrape is one event, plus its duration in `contact_time`).
"""
from dataclasses import dataclass, field

ROBOT = 'sewer_robot'
IGNORED = ('floor', 'ground')            # the floor of the sewer world and of the test arena


def partner_model(name1, name2, robot=ROBOT):
    """The obstacle model in a contact pair involving the robot, or None (floor, robot-only, unrelated pair)."""
    m1, m2 = name1.split('::')[0], name2.split('::')[0]
    if m1 == robot and m2 != robot:
        other = m2
    elif m2 == robot and m1 != robot:
        other = m1
    else:
        return None
    return None if other in IGNORED else other


def category(model):
    """'wall' for the walls, 'debris' for debris_*, otherwise 'other' (for example the test arena's box)."""
    return 'wall' if model.startswith('wall') else 'debris' if model.startswith('debris') else 'other'


@dataclass
class CollisionCounter:
    gap: float = 1.0                                  # seconds without contact before a touch counts as a new collision
    events: list = field(default_factory=list)        # (t, model) of every counted collision
    contact_time: float = 0.0                         # seconds in contact with any obstacle (approximate: message-rate based)
    _last: dict = field(default_factory=dict)         # model -> time of its latest contact
    _t_any: float = None

    def update(self, t, models):
        """Feed the obstacle models in contact at time t (empty = no contact message). Returns the new events."""
        new = []
        for m in sorted(set(models)):
            if m not in self._last or t - self._last[m] > self.gap:
                self.events.append((t, m))
                new.append((t, m))
            self._last[m] = t
        if models:
            if self._t_any is not None and t - self._t_any <= self.gap:
                self.contact_time += t - self._t_any
            self._t_any = t
        return new

    def summary(self):
        counts = {'wall': 0, 'debris': 0, 'other': 0}
        for _, m in self.events:
            counts[category(m)] += 1
        return dict(total=len(self.events), **counts, contact_time=round(self.contact_time, 2),
                    events=[[round(t, 2), m] for t, m in self.events])
