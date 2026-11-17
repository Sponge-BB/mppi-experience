"""Post-delivery refresh request; no execution permission or predicted prefix."""
import math


class AvoidanceEpisode:
    def __init__(self):
        self.goal = None
        self.episode = 0
        self.active = False
        self.clear_count = 0

    def observe(self, goal, delivered, intervention, source_stamp=None):
        if goal != self.goal:
            self.goal, self.episode = goal, 0
            self.active, self.clear_count = False, 0
        if (not delivered or intervention is None or source_stamp is None or
                not math.isfinite(source_stamp) or source_stamp <= 0):
            self.clear_count = 0
            return None
        if not intervention:
            self.clear_count += 1
            if self.clear_count >= 4:
                self.active = False
            return None
        if not self.active:
            self.episode = float(source_stamp)
        self.active, self.clear_count = True, 0
        return self.episode


def encode_event(checked_at, goal, reference_start, odom, joint, map_stamp,
                 dynamic_stamp, dynamic_required, episode):
    # No command/target fields: the receiver consumes only an early refresh request.
    # The caller must invoke this only after genuine navigation command delivery.
    values = [1., checked_at, goal, reference_start, odom, joint, map_stamp,
              dynamic_stamp, float(dynamic_required), float(episode)]
    values = [float(value) for value in values]
    if len(values) != 10 or not all(math.isfinite(value) for value in values):
        raise ValueError('invalid_refresh_event')
    if (not all(value > 0 for value in values[1:7]) or values[8] not in (0., 1.)
            or (values[8] and values[7] <= 0) or values[9] <= 0
            or values[9] > checked_at+.005):
        raise ValueError('invalid_refresh_event_identity')
    return values
