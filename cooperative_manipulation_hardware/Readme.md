# Launch the cooperative manipulation with hardware

## Start roscore
1. roscore 

## Launch the ur16e
Check on dashboard if ROS program is running

If recommended extract calibration information:
 roslaunch ur_calibration calibration_correction.launch robot_ip:=ur target_filename:="/home/rosmatch/catkin_ws_timmer/src/match_mobile_robotics/ur/ur_launch_hardware/config/my_calibration.yaml"
and  pass that calibration to the launch file

1. roslaunch ur_launch_hardware ur.launch

* if nessecary: rosrun ur_launch_hardware enably.py and rosrun ur_launch_hardware grip_service_interface.py
 

## Launch the panda
1. On Franka Desk click on "Activate FCI"
2. roslaunch cooperative_manipulation_hardware franka_hardware.launch

## Launch rviz
roslaunch coopaerative_manipulation_hardware coopaerative_manipulation_hardware_rviz.launch

## Move to start position
1. roslaunch cooperative_manipulation_hardware ur16e_move_to_start.launch
1. roslaunch cooperative_manipulation_hardware franka_move_to_start.launch

## Launch the controllers
1. roslaunch cooperative_manipulation_controllers ur16e_control_hardware.launch
2. roslaunch cooperative_manipulation_controllers franka_control_hardware.launch


## Set velocity commands
Publish velocity commands under: rostopic pub -r 10 /cooperative_manipulation/cartesian_velocity_command
