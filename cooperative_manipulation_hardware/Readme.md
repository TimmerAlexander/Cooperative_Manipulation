# Launch cooperative manipulation hardware

## 1. Start roscore
- In a new Terminal: 

        roscore

## 2. Launch the ur16e
- In a new Terminal: 

        roslaunch cooperative_manipulation_hardware ur16e_hardware.launch

- Move the ur16e to the start position if nessecary
    - In a new Terminal: 

            roslaunch ur_launch_hardware ur16e_move_to_start.launch


## 3. Launch the panda
- On Franka Desk click on "Activate FCI" in the menu!
- In a new Terminal:

        roslaunch cooperative_manipulation_hardware franka_hardware.launch

## 4. Launch the controllers
- Franka impedance controller
    - In a new Terminal:

            roslaunch cooperative_manipulation_controllers ur16e_control_hardware.launch

- UR16e admittance controller
    - In a new Terminal:

            roslaunch cooperative_manipulation_controllers franka_control_hardware.launch

## 5. Launch cooperative manipulation node
- In a new Terminal: 

        rosrun cooperative_manipulation cooperative_manipulation_node.py

## 6. Launch cooperative manipulation rviz
- In a new Terminal: 

        rosrun cooperative_manipulation_hardware cooperative_manipulation_rviz.rviz


# Launch the cooperative manipulation movements
## Launch the cooperative movement
- In a new Terminal: 

        rosrun cooperative_manipulation cooperative_movment.py

## Launch the elbow singularity movement
- In a new Terminal: 

        rosrun cooperative_manipulation elbow_singularity.py


# Troubleshooting
## Franka Emika:
### Clean build with libfranka: 
    catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/rosmatch/< your workspace >/src/libfranka/build
### Recovery from error:
    rostopic pub -1 /panda/franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}" 

## UR16e:
### Zero F/T-Sensor
    rosservice call /ur/ur_hardware_interface/zero_ftsensor 

# Measurements:

### Franka Emika:
        - Wrench: /panda/franka_state_controller/F_ext
        - Position difference: /panda/measurement/delta_pos
        - Orientation difference: /panda/measurement/delta_ori
### UR16e:
        - Wrench: /ur/wrench
        - Position difference: /ur/measurement/delta_pos
        - Orientation difference: /ur/measurement/delta_ori
        - Sigma: /ur/measurement/sigma

## Rosbag:

- Navigate to your rosbag folder

        cd ~/catkin_ws_timmer/src/Cooperative_Manipulation/cooperative_manipulation/rosbag
- Record measurements:

        rosbag record /panda/franka_state_controller/F_ext /panda/measurement/delta_pos /panda/measurement/delta_ori /ur/wrench /ur/measurement/delta_pos /ur/measurement/delta_ori /ur/measurement/sigma
- Play rosbag file 

        rosbag play <rosbag file name>




