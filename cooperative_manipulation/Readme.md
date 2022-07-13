clean build with libfranka: catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/rosmatch/catkin_ws_timmer/src/libfranka/build

rostopic pub -1 /panda/franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}" 
