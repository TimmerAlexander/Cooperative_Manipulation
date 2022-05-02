#!/usr/bin/env python3

# /***************************************************************************

# 
# @package: panda_siimulator_examples
# @metapackage: panda_simulator
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
# 

# **************************************************************************/

# /***************************************************************************
# Copyright (c) 2019-2021, Saif Sidhik
 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# **************************************************************************/

"""
    This is a demo showing task-space control on the 
    simulator robot using the ROS topics and messages directly 
    from panda_simulator. The task-space force for the desired
    pose is computed using a simple PD law, and the corresponding
    joint torques are computed and sent to the robot. 
    
    By using this file you can set a equilibrium pose by using interactive marker. You can also set the target 
    By publishing the topic "panda_simulator/equili_pose" .

"""

import copy
import rospy


import tf, numpy, math
from geometry_msgs.msg import Twist, Vector3Stamped
from franka_core_msgs.msg import EndPointState, JointCommand, RobotState
from franka_interface import ArmInterface



class franka_impedance_controller():
    
    def config(self):
        # --------- Modify as required ------------
        # Task-space controller parameters
        # stiffness gains
        self.P_trans_x = 10.
        self.P_trans_y = 10.
        self.P_trans_z = 10.
        self.P_rot_x = 10.
        self.P_rot_y = 10.
        self.P_rot_z = 10.
        # Damping gains
        self.D_trans_x = 10.
        self.D_trans_y = 10.
        self.D_trans_z = 10.
        self.D_rot_x = 10.
        self.D_rot_y = 10.
        self.D_rot_z = 10.
        # -----------------------------------------
        
        self.F_target = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Min and max limits for the cartesian velocity (trans/rot) (unit: [m/s])
        self.cartesian_velocity_trans_min_limit = 0.001
        self.cartesian_velocity_trans_max_limit = 0.1
        self.cartesian_velocity_rot_min_limit = 0.001
        self.cartesian_velocity_rot_max_limit = 0.01
        self.publish_rate = 100 # [Hz]

        self.JACOBIAN = None
        self.CARTESIAN_POSE = None
        self.CARTESIAN_VEL = None
        
        
        self.goal_pos = numpy.array([None])
        self.goal_euler = numpy.array([None])
        self.start_pos = numpy.array([0.0,0.0,0.0])
        self.start_euler = numpy.array([0.0,0.0,0.0])   
        self.start_offset = 0.01
        
        self.delta_pos = numpy.array([0.0,0.0,0.0])
        self.delta_euler = numpy.array([0.0,0.0,0.0])
        self.curr_euler  = numpy.array([0.0,0.0,0.0])
        
        self.desired_velocity_transformed = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        
        self.base_cartesian_velocity_trans = Vector3Stamped()
        self.base_cartesian_cartesian_velocity_rot = Vector3Stamped()
        self.panda_link8_cartesian_velocity_trans = Vector3Stamped()
        self.padna_link8_cartesian_velocity_rot = Vector3Stamped()
        
        
        

    def __init__(self):
        # * Load config parameters
        self.config()
         # * Get namespace for topics from launch file
        
        rospy.init_node("ts_control_sim_only")
        
        # * Initialize tf TransformListener
        try:
            self.listener = tf.TransformListener()
            self.listener.waitForTransform("panda/base","world", rospy.Time(), rospy.Duration(4.0))
        except:
            print("self.listener.waitForTransform in init")
        # if not using franka_ros_interface, you have to subscribe to the right topics
        # to obtain the current end-effector state and robot jacobian for computing 
        # commands
        self.cartesian_state_sub = rospy.Subscriber(
            'panda_simulator/custom_franka_state_controller/tip_state',
            EndPointState,
            self._on_endpoint_state,
            queue_size=1,
            tcp_nodelay=True)

        self.robot_state_sub = rospy.Subscriber(
            'panda_simulator/custom_franka_state_controller/robot_state',
            RobotState,
            self._on_robot_state,
            queue_size=1,
            tcp_nodelay=True)
        
        self.cartesian_msg_sub = rospy.Subscriber(
            '/cooperative_manipulation/cartesian_velocity_command', Twist, self.cartesian_msg_callback,queue_size=1)
        
        # create joint command message and fix its type to joint torque mode
        self.command_msg = JointCommand()
        self.command_msg.names = ['panda_joint1','panda_joint2','panda_joint3',\
            'panda_joint4','panda_joint5','panda_joint6','panda_joint7']
        self.command_msg.mode = JointCommand.TORQUE_MODE
        
        # Also create a publisher to publish joint commands
        self.joint_command_publisher = rospy.Publisher(
                'panda_simulator/motion_controller/arm/joint_commands',
                JointCommand,
                tcp_nodelay=True,
                queue_size=1)


        
        # wait for messages to be populated before proceeding
        rospy.loginfo("Subscribing to robot state topics...")
        while (True):
            if not (self.JACOBIAN is None or self.CARTESIAN_POSE is None):
                print(self.JACOBIAN,self.CARTESIAN_POSE)
                break
        rospy.loginfo("Recieved messages; Launch Franka Impedance Control.")

        pose = copy.deepcopy(self.CARTESIAN_POSE)
        
        start_pos, start_ori = pose['position'],pose['euler']
        self.goal_pos, self.goal_euler = start_pos, start_ori # set goal pose a starting pose in the beginning
        

            
        print("start pos and euler: ")
        print(self.goal_pos)
        print(self.goal_euler)
        

        rospy.on_shutdown(self._on_shutdown)
        
        # start controller thread
        self.control_thread()
        
        
        rospy.spin()    

    
    def cartesian_msg_callback(self,desired_velocity):
            """
            Get the cartesian velocity command and transform it from the 'world' frame to the 'panda/panda_link8' (EE-frame)frame and from the 'panda/panda_link8' frame to the 'panda/base' (0-frame)frame.
            
            rostopic pub -r 10 cooperative_manipulation/cartesian_velocity_command geometry_msgs/Twist "linear:
            x: 0.0
            y: 0.0
            z: 0.0
            angular:
            x: 0.0
            y: 0.0
            z: 0.0" 
            """
            # Get current time stamp
            now = rospy.Time()

            world_cartesian_velocity_trans  = Vector3Stamped()
            world_cartesian_velocity_rot  = Vector3Stamped()
            # Converse cartesian_velocity translation to vector3
            world_cartesian_velocity_trans.header.frame_id = 'world'
            world_cartesian_velocity_trans.header.stamp = now
            world_cartesian_velocity_trans.vector.x = desired_velocity.linear.x
            world_cartesian_velocity_trans.vector.y = desired_velocity.linear.y
            world_cartesian_velocity_trans.vector.z = desired_velocity.linear.z
            
            # Transform cartesian_velocity translation from 'world' frame to 'panda/base' frame and from 'panda/base' frame to 'panda/panda_link8'
            panda_link8_cartesian_velocity_trans = self.listener.transformVector3('panda/panda_link8',world_cartesian_velocity_trans)


            base_cartesian_velocity_trans = self.listener.transformVector3('panda/base',panda_link8_cartesian_velocity_trans)
 
            
            # Converse cartesian_velocity rotation to vector3
            world_cartesian_velocity_rot.header.frame_id = 'world'
            world_cartesian_velocity_rot.header.stamp = now
            world_cartesian_velocity_rot.vector.x = desired_velocity.angular.x
            world_cartesian_velocity_rot.vector.y = desired_velocity.angular.y
            world_cartesian_velocity_rot.vector.z = desired_velocity.angular.z
            
            # Transform cartesian_velocity rotation from 'world' frame to 'panda/base' frame and from 'panda/base' frame to 'panda/panda_link8'
            panda_link8_cartesian_velocity_rot = self.listener.transformVector3('panda/panda_link8',world_cartesian_velocity_rot)

            base_cartesian_velocity_rot = self.listener.transformVector3('panda/base',panda_link8_cartesian_velocity_rot)

            
            # Converse cartesian_velocity from vector3 to numpy.array
            self.desired_velocity_transformed = [
                base_cartesian_velocity_trans.vector.x,
                base_cartesian_velocity_trans.vector.y,
                base_cartesian_velocity_trans.vector.z,
                base_cartesian_velocity_rot.vector.x,
                base_cartesian_velocity_rot.vector.y,
                base_cartesian_velocity_rot.vector.z,
                ]
            
            
            #print("panda_link8_cartesian_velocity_trans")
            #print(panda_link8_cartesian_velocity_trans)
            #print("base_cartesian_velocity_trans")
            #print(base_cartesian_velocity_trans)
        
    def _on_robot_state(self,msg):
        """
            Callback function for updating jacobian and EE velocity from robot state
        """
        self.JACOBIAN = numpy.asarray(msg.O_Jac_EE).reshape(6,7,order = 'F')
        self.CARTESIAN_VEL = {
                    'linear': numpy.asarray([msg.O_dP_EE[0], msg.O_dP_EE[1], msg.O_dP_EE[2]]),
                    'angular': numpy.asarray([msg.O_dP_EE[3], msg.O_dP_EE[4], msg.O_dP_EE[5]]) }

    def _on_endpoint_state(self,msg):
        """
            Callback function to get current end-point state
        """
        # pose message received is a vectorised column major transformation matrix
        cart_pose_trans_mat = numpy.asarray(msg.O_T_EE).reshape(4,4,order='F')

        self.CARTESIAN_POSE = {
            'position': cart_pose_trans_mat[:3,3],
            'euler': numpy.asarray(tf.transformations.euler_from_matrix(cart_pose_trans_mat[:3,:3])) }
    
    
    def control_thread(self):
        """
            Actual control loop. Uses goal pose from the feedback thread
            and current robot states from the subscribed messages to compute
            task-space force, and then the corresponding joint torques.
        """
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            
                curr_pose = copy.deepcopy(self.CARTESIAN_POSE)
                curr_pos, curr_euler  = curr_pose['position'], curr_pose['euler']
                
                # * Check self.target_cartesian_velocity for the min/max velocity limits
                # Calculate the norm of target_cartesian_velocity (trans and rot)
                target_cartesian_trans_velocity_norm = numpy.linalg.norm(self.desired_velocity_transformed[0:3])
                target_cartesian_rot_velocity_norm = numpy.linalg.norm(self.desired_velocity_transformed[3:6])
                
                #  Check for cartesian velocity max limit and set to max limit, if max limit is exceeded
                if target_cartesian_trans_velocity_norm > self.cartesian_velocity_trans_max_limit:
                    for i in range(3):
                        self.desired_velocity_transformed[i] = (self.desired_velocity_transformed[i]/target_cartesian_trans_velocity_norm) * self.cartesian_velocity_trans_max_limit
                        
                if target_cartesian_rot_velocity_norm > self.cartesian_velocity_rot_max_limit:
                    for i in range(3,6):
                        self.desired_velocity_transformed[i] = (self.desired_velocity_transformed[i]/target_cartesian_rot_velocity_norm) * self.cartesian_velocity_rot_max_limit
                
                # Check for cartesian velocity min limit and set to null, if min limit is understeps
                if target_cartesian_trans_velocity_norm < self.cartesian_velocity_trans_min_limit:
                    for i in range(3):
                        self.desired_velocity_transformed[i] = 0.0
                
                if target_cartesian_rot_velocity_norm < self.cartesian_velocity_rot_min_limit:
                    for i in range(3,6):
                        self.desired_velocity_transformed[i] = 0.0
                    
                print("self.goal_pos,self.goal_euler")
                print(self.goal_pos,self.goal_euler)
                print("curr_pos, curr_euler")
                print(curr_pos, curr_euler)
                
                for i in range(3):
                    self.goal_pos[i] = self.goal_pos[i] + (self.desired_velocity_transformed[i]/self.publish_rate)
                    self.delta_pos[i] = (self.goal_pos[i] - curr_pos[i])
                        
                for i in range(3):
                    self.goal_euler[i] = self.goal_euler[i] + (self.desired_velocity_transformed[i+3]/self.publish_rate)
                    self.delta_euler[i] = (self.goal_euler[i] - curr_euler[i])

                print("self.delta_pos")
                print(self.delta_pos)

            
                #print(self.delta_pos)
                #print(self.delta_euler)
                        
                # print("self.desired_velocity_transformed: ")
                # print(self.desired_velocity_transformed)
                    
                # Todo: 2. Calculate pos and ori differences from velocity (goal - curr) 3. Try world -> panda/panda_link8 -> panda/panda_base, should be equal as 0_dP_EE and tf homogeniuos transformation(0=base and EE=panda_link8)4. Calculate velocity differences (new_vel - old vel) (old_vel from CARTESIAN_VEL or last cmd_vel?)
                    
                self.F_target[0] = self.D_trans_x * self.desired_velocity_transformed[0] + self.P_trans_x * self.delta_pos[0]
                self.F_target[1] = self.D_trans_y * self.desired_velocity_transformed[1] + self.P_trans_y * self.delta_pos[2]
                self.F_target[2] = self.D_trans_z * self.desired_velocity_transformed[2] + self.P_trans_z * self.delta_pos[2]
                self.F_target[3] =self.D_rot_x * self.desired_velocity_transformed[3] + self.P_rot_x * self.delta_euler[0]
                self.F_target[4] = self.D_rot_y * self.desired_velocity_transformed[4] + self.P_rot_y * self.delta_euler[1]
                self.F_target[5] =  self.D_rot_z * self.desired_velocity_transformed[5] + self.P_rot_z * self.delta_euler[2]

                
                print("self.F_target")
                print(self.F_target)
                    
                J = copy.deepcopy(self.JACOBIAN)

                # joint torques to be commanded
                tau = numpy.dot(J.T,self.F_target)
                # publish joint commands
                self.command_msg.effort = tau.flatten()
                self.joint_command_publisher.publish(self.command_msg)
                rate.sleep()

    def _on_shutdown(self):
        """
            Clean shutdown controller thread when rosnode dies.
        """
        print("Shutdown impedance controller:")

        print("Shutdown publisher joint velocity!")
        self.joint_command_publisher.publish(self.joint_command_publisher)
        
        print("Unregister from robot_state_sub!")
        self.robot_state_sub.unregister()
        
        print("Unregister from cartesian_state_sub!")
        self.cartesian_state_sub.unregister()
        
    
if __name__ == "__main__":
    franka_impedance_controller()