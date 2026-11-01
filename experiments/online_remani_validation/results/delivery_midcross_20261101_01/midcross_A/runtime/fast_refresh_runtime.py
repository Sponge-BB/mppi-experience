"""Post-command avoidance feedback, not control authorization (isolated SIL)."""
import math
from pathlib import Path
import time

from native_collision_runtime import replace_once


class AvoidanceEpisode:
    """Reuse the kernel's intervention hysteresis; do not latch per reference."""
    def __init__(self):
        self.generation = None
        self.episode = 0
        self.active = False
        self.clear_count = 0

    def observe(self, generation, validated, active):
        if generation != self.generation:
            self.generation = generation
            self.episode = 0
            self.active = False
            self.clear_count = 0
        # Failed/expired controls are not evidence that avoidance has cleared.
        if not validated or active is None:
            self.clear_count = 0
            return self.episode, False
        if active:
            if not self.active:
                self.episode += 1
            self.active = True
            self.clear_count = 0
        else:
            # referenceChanged() resets the kernel's intervention flag. Keep
            # this episode across that reset until four explicitly safe normal
            # tracking cycles, reusing the kernel's .25m/four-cycle clear rule.
            self.clear_count += 1
            if self.clear_count >= 4:
                self.active = False
        return self.episode, bool(active)


def phase_state(phase):
    """None means no known dynamic/normal tracking state, not 'clear'."""
    if not phase:
        return None, 0.
    decision = phase.get('decision', [])
    strategy = phase.get('strategy', [])
    if len(decision) < 4 or len(strategy) < 2:
        return None, 0.
    gap = float(decision[3])
    if not math.isfinite(gap):
        return None, 0.
    if strategy[0] == 1 and strategy[1] == 1 and decision[0] == 2:
        return True, gap
    # Neither reference-reset inactive nor rejoining mode 3 proves clearing.
    if decision[0] == 0 and gap > .25:
        return False, gap
    return None, gap


def publish_feedback(node, phase, validated, stamp, generation, revision, goal_key):
    import rospy
    from std_msgs.msg import Float64MultiArray
    from collab_seed_export import goal_token

    if not hasattr(node, 'fast_refresh_episode'):
        node.fast_refresh_episode = AvoidanceEpisode()
        node.fast_refresh_pub = rospy.Publisher(
            '/planning/local_refresh_request', Float64MultiArray, queue_size=1)
    active, gap = phase_state(phase)
    episode, request = node.fast_refresh_episode.observe(generation, validated, active)
    values = [1., stamp, float(generation), float(revision),
              float(goal_token(goal_key or '')), float(episode),
              float(request), float(validated), gap]
    packet = Float64MultiArray()
    packet.data = values
    publication_monotonic = time.monotonic()
    publication_ros_time = rospy.get_time()
    node.fast_refresh_pub.publish(packet)
    return dict(stamp=stamp, publication_ros_time=publication_ros_time,
                publication_monotonic=publication_monotonic,
                generation=generation, revision=revision, episode=episode,
                active=request, validated=bool(validated), nominal_gap=gap,
                goal_token=values[4], scope='request slow refresh only')


def patch_runtime(trial):
    trial = Path(trial)
    path = trial/'runtime/local_execution.py'
    text = path.read_text()
    text = replace_once(text,
        '            session=node.execution;generation=session.generation;guide=session.guide',
        '            session=node.execution;generation=session.generation;guide=session.guide\n'
        '            refresh_goal_key=session.goal_key')
    anchor = '        publish_seed(node,checked,validated_count,published,generation,revision,odom_stamp,cloud_stamp,stamp)'
    text = replace_once(text, anchor, anchor+'\n'
        '        from fast_refresh_runtime import publish_feedback\n'
        '        fast_refresh_feedback=publish_feedback(node,phase_debug,\n'
        "            reason=='ok' and validated_count>0 and validation_code==0,\n"
        '            published,generation,revision,refresh_goal_key)')
    text = replace_once(text,
        "            'local_route':checked[:validated_count:10,1:3].tolist(),'local_phase':phase_debug,",
        "            'fast_refresh_feedback':fast_refresh_feedback,\n"
        "            'local_route':checked[:validated_count:10,1:3].tolist(),'local_phase':phase_debug,")
    compile(text, str(path), 'exec')
    path.write_text(text)

