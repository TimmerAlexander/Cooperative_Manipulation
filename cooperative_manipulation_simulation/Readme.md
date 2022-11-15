# Launch the cooperative manipulation in the simulation

## Launch gazebo and the robots and the object
1. roslaunch cooperative_manipulation_simualtion cooperative_manipulation_gazebo.launch
2. roslaunch cooperative_manipulation_simualtion cooperative_manipulation_ur16e.launch
3. roslaunch cooperative_manipulation_moveit custom_ur16e_moveit.launch
4. roslaunch cooperative_manipulation_simualtion cooperative_manipulation_franka.launch
4. roslaunch cooperative_manipulation_simualtion cooperative_manipulation_object.launch

## Grasp the object
1. roslaunch cooperative_manipulation_simualtion cooperative_manipulation_gripper_grasp.launch

## Launch the controllers
1. roslaunch cooperative_manipulation_controllers ur16e_control_simulation.launch
2. roslaunch cooperative_manipulation_controllers franka_control_simulation.launch

## Set velocity commands
rostopic pub -r 10 /cooperative_manipulation/cartesian_velocity_command
