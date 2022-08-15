# Franka Emika
## Clean build with libfranka: 
catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/rosmatch/catkin_ws_timmer/src/libfranka/build
## Recovery from error:
rostopic pub -1 /panda/franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}" 

# UR16e:
## Zero F/T-Sensor
rosservice call /ur/ur_hardware_interface/zero_ftsensor 

# Rosbag:

## Navigate to rosbag folder
    cd ~/catkin_ws_timmer/src/Cooperative_Manipulation/cooperative_manipulation/rosbag
## Record measurements:
    rosbag record /panda/franka_state_controller/F_ext /panda/measurement/delta_pos /panda/measurement/delta_ori /ur/wrench /ur/measurement/delta_pos /ur/measurement/delta_ori /ur/measurement/sigma
## Play rosbag.file 
    rosbag play <rosbag file name>

## Record measurements:
### Franka Emika:
        - wrench: /panda/franka_state_controller/F_ext
        - Pos diff: /panda/measurement/delta_pos
        - Ori diff: /panda/measurement/delta_ori
### UR16e:
        - wrench: /ur/wrench
        - Pos diff: /ur/measurement/delta_pos
        - Ori diff: /ur/measurement/delta_ori



