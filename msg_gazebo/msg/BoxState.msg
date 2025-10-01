# BoxState.msg
# Contains the full 6DOF state of the box in the world frame, plus its velocity.
std_msgs/Header header
geometry_msgs/PoseStamped pose_world          # The predicted pose of the box in the 'world' frame
geometry_msgs/TwistStamped velocity_world