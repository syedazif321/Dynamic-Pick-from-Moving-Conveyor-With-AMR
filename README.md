# Dynamic-Pick-from-Moving-Conveyor-With-AMR


ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"


ros2 service call /conveyor3/CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 10.0}"

ros2 service call /start_dynamic_pick std_srvs/srv/Trigger "{}"


azif@azif:~/projetcs/Dynamic-Pick-from-Moving-Conveyor-With-AMR$ ros2 topic echo /joint_states --once
header:
  stamp:
    sec: 10721
    nanosec: 974000000
  frame_id: base_link
name:
- joint2
- joint3
- joint5
- joint6
- joint1
- joint4
- joint7
position:
- -0.6186871940940399
- 0.4245574549836082
- 0.514352882474463
- 1.2415513769947104
- -0.7847751610991667
- 1.281974774731177
- -0.11330507387479738
velocity:
- -0.06349233155863987
- 0.21221750883568827
- 0.13142241789035228
- -0.006726234453278614
- -0.26094091079962234
- -0.0062921704872860845
- 0.007102658774644693
effort:
- .nan
- .nan
- .nan
- .nan
- .nan
- .nan
- .nan



0.1 17


ros2 topic pub --once /rm_group_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{header: {stamp: now}, joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7], points: [{positions: [1.0, -1.0, -3.1, -0.66, 3.07, -2.12, -3.66], time_from_start: {sec: 5, nanosec: 0}}]}"


20  - 0.1