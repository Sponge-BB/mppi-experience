#!/usr/bin/env python3
"""Controlled offline call of deployed cycle; all publishers are mocks."""
import hashlib
import json
from pathlib import Path
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent
WS = ROOT.parents[3]
sys.path.insert(0, str(WS/'src/mm_remani/test'))
from test_mppi_execution_gate import ExecutionDeliveryTests, message


rows = []
for elapsed, feasible in ((.05, False), (.12, False), (.12, True)):
    fixture = ExecutionDeliveryTests()
    fixture.setUp()
    node = fixture.node
    node.deadline = .095
    fixture.feed(message(100.))
    node.reference.active = True
    node.reference.goal_stamp = 100.
    node.reference.navigation = node.kernel
    node.sent_command = True
    node.sent_navigation_goal = 100.
    clock = [0.]

    def solve(*_):
        clock[0] = elapsed
        if feasible:
            return fixture.solve()
        return None, {'feasible': 0}

    node.reference.plan.side_effect = solve
    with patch('whole_body_mppi.time.monotonic', side_effect=lambda: clock[0]), \
            patch('whole_body_mppi.rospy.logwarn'):
        fixture.cycle()
    status = json.loads(node.info_pub.publish.call_args.args[0].data)
    rows.append(dict(controlled_elapsed_ms=elapsed*1000, feasible_return=feasible,
        rejection=status['state'], fault_true_calls=sum(bool(c.args[0].data)
            for c in node.fault_pub.publish.call_args_list),
        arm_publications=node.arm_pub.publish.call_count,
        base_commands=[[c.args[0].linear.x, c.args[0].angular.z]
            for c in node.base_pub.publish.call_args_list]))

assert rows[0]['rejection'] == rows[1]['rejection'] == 'no_feasible_candidate'
assert rows[0]['fault_true_calls'] == rows[1]['fault_true_calls'] == 1
assert rows[2]['rejection'] == 'planning_deadline_exceeded'
assert rows[2]['fault_true_calls'] == 0
code = WS/'src/mm_remani/scripts/whole_body_mppi.py'
report = dict(method='Actual cycle, mocked publishers, fresh permission/state/map, controlled solver clock. No ROS node, sockets, CAN, sleep or runtime edits.',
    code_sha256=hashlib.sha256(code.read_bytes()).hexdigest(), deadline_ms=95, cases=rows,
    conclusion='At identical 120ms simulated solver latency, no-result raises a navigation fault while a feasible result is treated as local deadline retry.',
    boundary='This proves the code branch defect. Real bag cycle_ms includes rejection work; the exact real solve elapsed was not recorded.')
(ROOT/'deadline_branch_reproduction.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
