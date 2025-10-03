# Dynamic-Pick-from-Moving-Conveyor-With-AMR


ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"


ros2 service call /conveyor3/CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 10.0}"

ros2 service call /start_dynamic_pick std_srvs/srv/Trigger "{}"
