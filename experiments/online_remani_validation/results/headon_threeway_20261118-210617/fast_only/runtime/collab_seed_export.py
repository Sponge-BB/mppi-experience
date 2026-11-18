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
    # Export the SAME local field and full projected body used by execution.
    # Both native and projected checks must accept a rolling target. This is a
    # target eligibility test only, not a replacement collision cost backend.
    if not hasattr(node,'factorial_field_pub'):
        node.factorial_field_pub=rospy.Publisher('/sim/mppi_target_field',Float64MultiArray,queue_size=1)
        node.factorial_field_stamp=-1e9
    now=rospy.get_time()
    if now-node.factorial_field_stamp<.19:return
    node.factorial_field_stamp=now
    message=Float64MultiArray()
    with node.lock:
        field=node.field;obstacles=node.obs.copy();cloud=node.cloud_stamp;dynamic=node.obs_stamp
        healthy=node.healthy
    if healthy and field is not None:
        body=node.expected_body
        header=[20261022.,now,cloud,dynamic,node.gx,node.gy,node.res,node.nx,node.ny,node.limits_margin,len(body),len(obstacles)]
        message.data=np.concatenate((header,body.ravel(),obstacles.ravel(),field.ravel())).tolist()
    node.factorial_field_pub.publish(message)
