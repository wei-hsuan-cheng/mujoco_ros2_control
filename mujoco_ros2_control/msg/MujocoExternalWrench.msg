####################################################################################################
# External wrench command for MuJoCo.
#
# body_name selects the MuJoCo body receiving the wrench.
# wrench_frame selects the frame in which the wrench values are represented.
# Supported wrench_frame values:
# - "world" or "global": MuJoCo world frame
# - "body", "local", or empty: target body frame
# - any MuJoCo body name: that body's frame
#
# header.stamp is optional for command timing.
# header.frame_id is used as wrench_frame only when wrench_frame is empty.
####################################################################################################
std_msgs/Header header
string body_name
string wrench_frame
geometry_msgs/Wrench wrench
