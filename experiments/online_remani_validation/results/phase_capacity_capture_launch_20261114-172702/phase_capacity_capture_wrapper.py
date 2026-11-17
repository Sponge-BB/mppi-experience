import sys
sys.path.insert(0, '/home/l/remani_ws/experiments/online_remani_validation')
import record_latency_session as capture
capture.TOPICS += ('/mm_controller_node/base_tracking_error', '/mm_controller_node/arm_tracking_error', '/mppi/prediction_rollouts', '/joint_ctrl_single')
capture.main()
