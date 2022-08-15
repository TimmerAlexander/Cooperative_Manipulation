clean build with libfranka: catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/rosmatch/catkin_ws_timmer/src/libfranka/build

rostopic pub -1 /panda/franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}" 


rosservice call /ur/ur_hardware_interface/zero_ftsensor 



Record measurements:
    Franka Emika:
        - wrench: /panda/franka_state_controller/F_ext
        - Pos diff: /panda/measurement/delta_pos
        - Ori diff: /panda/measurement/delta_ori





rosbag record /panda/franka_state_controller/F_ext /panda/measurement/delta_pos /panda/measurement/delta_ori


rosbag play <rosbag file name>