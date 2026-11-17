# Driver execution gate, published only in command-timeout mode.
# Stamp is the actual last ArmStatus CAN source time, never publication time.
# Zero means no valid source timestamp has been received.
std_msgs/Header header
# Driver enable and all six fresh motors; native startup additionally requires
# the original standby target to be reached and stable before this becomes true.
bool enabled
# True only after an external joint command succeeds and its lease is valid.
# Native standby readiness alone never asserts this acknowledgement.
bool command_active
# Once set, this driver will reject commands until an explicit process restart.
bool stop_latched
# Normal ArmStatus and complete joint/motor feedback pass source-age checks.
bool feedback_valid
# Earliest real status/joint/motor source-feedback expiry in wall/ROS time.
# A previous command lease affects command_active, not healthy command readiness.
# Repeated publication cannot extend it; zero denies execution.
time valid_until
