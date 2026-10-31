"""Post-command export of a checked prediction, NEVER an execution promise."""
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray


def goal_token(text):
    result=1469598103934665603
    for value in text.encode('utf8'):
        result=((result ^ value)*1099511628211)&((1<<64)-1)
    return result & 0xffffffff


def publish_seed(node, checked, count, stamp, generation, revision, odom, cloud, dynamic):
    if not hasattr(node,'factorial_seed_pub'):
        node.factorial_seed_pub=rospy.Publisher('/sim/mppi_seed_states',Float64MultiArray,queue_size=1)
    packet=Float64MultiArray()
    if count>=3:
        rows=np.asarray(checked[:count:10],dtype=float)
        if len(rows)>=2 and np.all(np.isfinite(rows)):
            packet.data=[1.,stamp,float(generation),float(revision),float(goal_token(node.execution.goal_key or '')),
                         odom,cloud,dynamic,8.,float(len(rows))]+rows.ravel().tolist()
    node.factorial_seed_pub.publish(packet)
