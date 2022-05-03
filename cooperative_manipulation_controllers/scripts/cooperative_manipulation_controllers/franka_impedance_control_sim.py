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

import copy, numpy, quaternion
import rospy
import tf
import threading
from geometry_msgs.msg import Twist, Vector3Stamped
from franka_core_msgs.msg import EndPointState, JointCommand, RobotState


import quaternion

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
        self.P_pos = 50.
        self.P_ori = 25.
        # Damping gains
        self.D_trans_x = 10.
        self.D_trans_y = 10.
        self.D_trans_z = 10.
        self.D_rot_x = 10.
        self.D_rot_y = 10.
        self.D_rot_z = 10.
        self.D_pos = 10.
        self.D_ori = 1.
        # -----------------------------------------
        # create joint command message and fix its type to joint torque mode
        self.command_msg = JointCommand()
        self.command_msg.names = ['panda_joint1','panda_joint2','panda_joint3',\
            'panda_joint4','panda_joint5','panda_joint6','panda_joint7']
        self.command_msg.mode = JointCommand.TORQUE_MODE
        
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
        
        
        self.delta_pos = numpy.array([0.0,0.0,0.0]).reshape([3,1])
        self.delta_ori = numpy.array([0.0,0.0,0.0]).reshape([3,1])
        
        self.desired_velocity_trans_transformed  = numpy.array([0.0,0.0,0.0])
        self.desired_velocity_rot_transformed  = numpy.array([0.0,0.0,0.0])
        
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
            '/cooperative_manipulation/cartesian_velocity_command', 
            Twist, 
            self.cartesian_msg_callback,
            queue_size=1,
            tcp_nodelay=True)
        
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
        
        rospy.on_shutdown(self._on_shutdown)
        
        curr_pose = copy.deepcopy(self.CARTESIAN_POSE)
        curr_pos, curr_ori = curr_pose['position'],curr_pose['orientation']
        self.goal_pos = numpy.asarray(curr_pos.reshape([1,3]))
        self.goal_ori = curr_ori

        # start controller thread
        rate = rospy.Rate(self.publish_rate)
        ctrl_thread = threading.Thread(target=self.control_thread, args = [rate])
        ctrl_thread.start()
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
            self.desired_velocity_trans_transformed = [
                base_cartesian_velocity_trans.vector.x,
                base_cartesian_velocity_trans.vector.y,
                base_cartesian_velocity_trans.vector.z,
                ]
            
            self.desired_velocity_rot_transformed = [
                base_cartesian_velocity_rot.vector.x,
                base_cartesian_velocity_rot.vector.y,
                base_cartesian_velocity_rot.vector.z,
                ]
            
            
    def control_thread(self,rate):
        """
            Actual control loop. Uses goal pose from the feedback thread
            and current robot states from the subscribed messages to compute
            task-space force, and then the corresponding joint torques.
        """
        movement_trans = numpy.array([None])
        movement_ori = numpy.array([None])
        while not rospy.is_shutdown():
            
            # Get current position and ori angle
            curr_pose = copy.deepcopy(self.CARTESIAN_POSE)
            curr_pos, curr_ori = curr_pose['position'],curr_pose['orientation']
            
            # # Get current linear and angular velocity
            # vel_trans = (self.CARTESIAN_VEL['linear']).reshape([3,1])
            # vel_rot = self.CARTESIAN_VEL['angular'].reshape([3,1])
            
            # print("vel_trans, vel_rot")
            # print(vel_trans, vel_rot)
            
            
            # * Check self.target_cartesian_velocity for the min/max velocity limits
            # Calculate the norm of target_cartesian_velocity (trans and rot)
            target_cartesian_trans_velocity_norm = numpy.linalg.norm(self.desired_velocity_trans_transformed)
            target_cartesian_rot_velocity_norm = numpy.linalg.norm(self.desired_velocity_rot_transformed)
                
            #  Check for cartesian velocity max limit and set to max limit, if max limit is exceeded
            if target_cartesian_trans_velocity_norm > self.cartesian_velocity_trans_max_limit:
                for i in range(3):
                    self.desired_velocity_trans_transformed[i] = (self.desired_velocity_trans_transformed[i]/target_cartesian_trans_velocity_norm) * self.cartesian_velocity_trans_max_limit
                        
            if target_cartesian_rot_velocity_norm > self.cartesian_velocity_rot_max_limit:
                for i in range(3):
                    self.desired_velocity_rot_transformed[i] = (self.desired_velocity_rot_transformed[i]/target_cartesian_rot_velocity_norm) * self.cartesian_velocity_rot_max_limit
                    
                
                
            # Check for cartesian velocity min limit and set to null, if min limit is understeps
            if target_cartesian_trans_velocity_norm < self.cartesian_velocity_trans_min_limit:
                for i in range(3):
                    self.desired_velocity_trans_transformed[i] = 0.0
                
            if target_cartesian_rot_velocity_norm < self.cartesian_velocity_rot_min_limit:
                for i in range(3):
                    self.desired_velocity_rot_transformed[i] = 0.0


            # Calculate the current movement/orientation
            movement_trans = numpy.asarray([x / self.publish_rate for x in self.desired_velocity_trans_transformed]).reshape([1,3])
            
            movement_ori = quaternion.from_euler_angles(numpy.asarray([x / self.publish_rate for x in self.desired_velocity_rot_transformed]))  
            
            print("movement_ori %0.4f",movement_ori)
            
            # Add 
            self.goal_pos = (self.goal_pos + movement_trans)
            self.goal_ori = self.quat_add(self.goal_ori,movement_ori)
            # Calculate position and ori difference
            self.delta_pos = (self.goal_pos - curr_pos).reshape([3,1])
            self.delta_ori = self.quatdiff_in_euler(curr_ori, self.goal_ori).reshape([3,1])


            # Calculate linear and angular velocity difference
            #self.delta_linear = (self.desired_velocity_trans_transformed.reshape([3,1]) - vel_trans)
            #self.delta_angular = (self.desired_velocity_rot_transformed.reshape([3,1]) - vel_rot)
            
            #print("self.delta_linear,self.delta_angular")
            #print(self.delta_linear,self.delta_angular)
            
            # Desired task-space force using PD law
            F = numpy.vstack([self.P_pos*(self.delta_pos), self.P_ori*(self.delta_ori)])

            J = copy.deepcopy(self.JACOBIAN)

            # joint torques to be commanded
            tau = numpy.dot(J.T,F)

            # publish joint commands
            self.command_msg.effort = tau.flatten()
            self.joint_command_publisher.publish(self.command_msg)
            rate.sleep()
                        
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
            'orientation': quaternion.from_rotation_matrix(cart_pose_trans_mat[:3,:3]) }
    
    def quatdiff_in_euler(self,quat_curr, quat_des):
        """
        Compute difference between quaternions and return 
        Euler angles as difference
    """
        curr_mat = quaternion.as_rotation_matrix(quat_curr)
        des_mat = quaternion.as_rotation_matrix(quat_des)
        rel_mat = des_mat.T.dot(curr_mat)
        rel_quat = quaternion.from_rotation_matrix(rel_mat)
        vec = quaternion.as_float_array(rel_quat)[1:]
        if rel_quat.w < 0.0:
            vec = -vec
        
        return -des_mat.dot(vec)
    
    def quat_add(self,init_quat, add_quat):
        """
        Compute difference between quaternions and return 
        Euler angles as difference
    """
        # Extract the values from Q0
        w0 = init_quat.w
        x0 = init_quat.x
        y0 = init_quat.y
        z0 = init_quat.z
     
        # Extract the values from Q1
        w1 = add_quat.w
        x1 = add_quat.x
        y1 = add_quat.y
        z1 = add_quat.z
     
        # Computer the product of the two quaternions, term by term
        Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
        Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
        Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
        Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

        sum_quat = numpy.quaternion(Q0Q1_w ,Q0Q1_x,Q0Q1_y,Q0Q1_z)
        return sum_quat 

       
 
    
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