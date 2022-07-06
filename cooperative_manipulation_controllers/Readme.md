If moveit cannot connect to action server, launch the controller via rusrun

[ERROR] [1656598640.061294714]: libfranka: Move command rejected: command not possible in the current mode!
solution: rostopic pub -1 /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}"

[ERROR] [1656598987.875167694]: libfranka: Move command aborted: motion aborted by reflex! ["cartesian_motion_generator_joint_acceleration_discontinuity"]

press and release emergency stop
