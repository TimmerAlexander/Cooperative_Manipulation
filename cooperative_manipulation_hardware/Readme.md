# Launch the cooperative manipulation with hardware

## Start roscore
1. roscore 

## Launch the ur16e
Check on dashboard if robot is running

1. rosrun ur_launch_hardware startup.py
-----------------------------------------
1. rosrun ur_launch_hardware enably.py
2. rosrun ur_launch_hardware grip_service_interface.py
-----------------------------------------

3. roslaunch ur_launch_hardware ur.launch

## Launch the panda
1. On Franka Desk click on "Activate FCI"
2. roslaunch cooperative_manipulation_hardware franka_hardware.launch
3. roslaunch panda_moveit.config move_group.launch

## Launch rviz
roslaunch coopaerative_manipulation_hardware coopaerative_manipulation_hardware_rviz.launch

## Move to start position
1. roslaunch ur_launch_hardware ur16e_move_to_start.launch
1. roslaunch ur_launch_hardware franka_move_to_start.launch

## Launch the controllers
1. roslaunch cooperative_manipulation_controllers ur16e_control_hardware.launch
2. roslaunch cooperative_manipulation_controllers franka_control_hardware.launch


## Set velocity commands
rostopic pub -r 10 /cooperative_manipulation/cartesian_velocity_command
