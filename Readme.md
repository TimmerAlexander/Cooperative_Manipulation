
# 1. Installation
Start by changing directory to your catkin workspace!

## 1. Clone Package

        git clone https://github.com/match-ROS/match_mobile_robotics.git
The dependencies can be installed as following link: https://github.com/match-ROS/match_mobile_robotics.git

## 2. Clone Package

        git clone https://github.com/Grossbier/simulation_multirobots.git

## 3. Clone Package

        git clone https://github.com/TimmerAlexander/Cooperative_Manipulation

## 4. Clone Package

        git clone https://github.com/utecrobotics/robotiq

In the "robotiq" package modify following files:

In "mimic_joint_plugin.cpp" change:

    event::Events::DisconnectWorldUpdateBegin(this->updateConnection);

to 

    this->updateConnection.reset();

and 

    void MimicJointPlugin::UpdateChild()
    {
        static ros::Duration period(world_->GetPhysicsEngine()->GetMaxStepSize());

        // Set mimic joint's angle based on joint's angle
        double angle = joint_->GetAngle(0).Radian()*multiplier_+offset_;
        
        if(abs(angle-mimic_joint_->GetAngle(0).Radian())>=sensitiveness_)
        {
            if(has_pid_)
            {
            double a = mimic_joint_->GetAngle(0).Radian();
            if(a!=a)
                a = angle;
            double error = angle-a;
            double effort = gazebo::math::clamp(pid_.computeCommand(error, period), -max_effort_, max_effort_);
            }
            else
            {
            #if GAZEBO_MAJOR_VERSION >= 4
                mimic_joint_->SetPosition(0, angle);
            #else
                mimic_joint_->SetAngle(0, angle);
            #endif
            }
        }
    }

to 

    void MimicJointPlugin::UpdateChild()
    {
        static ros::Duration period(world_->Physics()->GetMaxStepSize());

        // Set mimic joint's angle based on joint's angle
        double angle = joint_->Position ( 0 )*multiplier_+offset_;
        
        if(abs(angle-mimic_joint_->Position ( 0 ))>=sensitiveness_)
        {
            if(has_pid_)
            {
            double a = mimic_joint_->Position ( 0 );
            if(a!=a)
                a = angle;
            double error = angle-a;
            double effort = ignition::math::clamp(pid_.computeCommand(error, period), -max_effort_, max_effort_);
            }
            else
            {
            #if GAZEBO_MAJOR_VERSION >= 4
                mimic_joint_->SetPosition(0, angle);
            #else
                mimic_joint_->SetAngle(0, angle);
            #endif
            }
        }
    }



In "robotiq_85_gripper.tansmission.xacro" change:

    <hardwareInterface>PositionJointInterface</hardwareInterface>

to 

    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>

In "robotiq_85_gripper.urdf.xacro" change:

    <mu1>1.0</mu1>
    <mu2>1.0</mu2>

to

    <mu1>1000000.0</mu1>
    <mu2>1000000.0</mu2>

## 5. Install libfranka: 
Libfranka can be installed as described in the following link: https://frankaemika.github.io/docs/installation_linux.html#installing-from-the-ros-repositories

## 6. Build packages
Use your standard build tools to build the downloaded packages e.g. :

        catkin build 

Clean build with libfranka: 

        catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/<Path to your workspace>/src/libfranka/build

# 2. Usage
## **Launch cooperative manipulation with hardware.**

## 1. Start rosmaster
In a new Terminal: 

        roscore

## 2. Launch the UR16e
Move the UR16e to the start position if nessecary. You can change the start pose of the UR16e by changing the joint angles in "ur16e_move_to_start.launch".

In a new Terminal: 

        roslaunch cooperative_manipulation_hardware ur16e_move_to_start.launch

In a new Terminal: 

        roslaunch cooperative_manipulation_hardware ur16e_hardware.launch




## 3. Launch the panda
On Franka Desk click on "Activate FCI" in the menu!

In a new Terminal:

        roslaunch cooperative_manipulation_hardware franka_hardware.launch

## 4. Launch the controllers
Launch the UR16e admittance controller.

In a new Terminal:

        roslaunch cooperative_manipulation_controllers franka_control_hardware.launch

Launch the Franka impedance controller.

In a new Terminal:

        roslaunch cooperative_manipulation_controllers ur16e_control_hardware.launch


## 5. Launch the cooperative manipulation node
This script enables communication between the robots.

In a new Terminal: 

        rosrun cooperative_manipulation cooperative_manipulation_node.py

## 6. Launch the scene in Rviz
In a new Terminal: 

        rosrun cooperative_manipulation_hardware cooperative_manipulation_hardware_rviz.rviz


## 7. Launch the cooperative manipulation movements
### **A movement consisting of translatory and rotatory movement**
Plan a trajectory of translational and rotational mmovements in "measurements_movement.py" and launch the movement as follow:

In a new Terminal: 

        rosrun cooperative_manipulation measurements_movement.py

### **A movement consisting of a circular motion**
A movement that triggers an elbow singularity at UR16e (from the given start pose).
You can set the radius and speed in "elbow_singularity.py" and start the movement as follows:

In a new Terminal: 

        rosrun cooperative_manipulation elbow_singularity.py

### **Set a velocity commands**
Set a velocity command from the terminal under the topic:

        rostopic pub -r 10 /cooperative_manipulation/cartesian_velocity_command

## **Troubleshooting**
### **Franka Emika:**

**ERROR:**

"libfranka: Move command rejected: command not possible in the current mode!"

**Solution:** 

In a new Terminal:

        rostopic pub -1 /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}"

**ERROR:**

"libfranka: Move command aborted: motion aborted by reflex! ["cartesian_motion_generator_joint_acceleration_discontinuity"]"

**Solution:**

In a new Terminal:

        press and release emergency stop

### **UR16e:**
Zero F/T-Sensor:

    rosservice call /ur/ur_hardware_interface/zero_ftsensor 






## **Launch the cooperative manipulation in the simulation**

## 1. Launch gazebo, Rviz, the robots and the object
In a new Terminal: 

        roslaunch cooperative_manipulation_simulation cooperative_manipulation_gazebo.launch

In a new Terminal: 

        roslaunch cooperative_manipulation_simulation cooperative_manipulation_ur16e.launch
In a new Terminal:  

        roslaunch cooperative_manipulation_moveit custom_ur16e_moveit.launch
In a new Terminal: 

        roslaunch cooperative_manipulation_simulation cooperative_manipulation_franka.launch
In a new Terminal: 

        roslaunch cooperative_manipulation_simulation cooperative_manipulation_object.launch

## 2. Grasp the object
Change the directory in "cooperative_manipulation_simulation/model/item_1m/model.sdf" if you haven't already done so:

    <uri>/<Path to your workspace>/src/Cooperative_Manipulation/cooperative_manipulation_simulation/model/visual/ItemProfile_1m.dae</uri>
In a new Terminal: 

        roslaunch cooperative_manipulation_simulation cooperative_manipulation_gripper_grasp.launch

## 3. Launch the controllers
In a new Terminal: 

        roslaunch cooperative_manipulation_controllers ur16e_control_simulation.launch

In a new Terminal: 

        roslaunch cooperative_manipulation_controllers franka_control_simulation.launch

## 4. Launch the singularity avoidance node
In a new Terminal: 

        rosrun cooperative_manipulation singularity_avoidance_node.py

## 5. Launch the cooperative manipulation movements
### **A movement consisting of translatory and rotatory movement**
Plan a trajectory of translational and rotational mmovements in "cooperative_movment.py" and launch the movement as follow:

In a new Terminal: 

        rosrun cooperative_manipulation cooperative_movment.py

### **A movement consisting of a circular motion**
A movement that triggers an elbow singularity at UR16e (from the given start pose).
You can set the radius and speed in "circular_movement.py" and start the movement as follows:

In a new Terminal:

        rosrun cooperative_manipulation circular_movement.py

### **Set a velocity commands**
Set a velocity command from the terminal under the topic:

        rostopic pub -r 10 /cooperative_manipulation/cartesian_velocity_command


