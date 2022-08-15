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
    Description

    Impedance controller

    Input:
    * Desired cartesian velocity of the EE: desired_velocity (In 'world' frame)

    Output:
    * Joint effort: self.command_msg.effort (Float64MultiArray)
"""

import numpy
import rospy
import tf
from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import Float64MultiArray
from cooperative_manipulation_controllers.msg import SingularityAvoidance, WorkspaceViolation

# For measurements
from franka_msgs.msg import FrankaState
import moveit_commander
import quaternion, sys, copy
# For measurements

class franka_hardware_node():

    def config(self):
        # Min and max limits for the cartesian velocity (trans/rot) (unit: [m/s],[rad/s])
        self.cartesian_velocity_trans_max_limit = 0.1
        self.cartesian_velocity_rot_max_limit = 0.1
        # Control thread publish rate
        self.publish_rate = 100 # [Hz]
        # Create joint command message 
        self.command_msg = Float64MultiArray()
        # Initialize desired translational and rotation velocity
        self.desired_velocity_trans_transformed  = numpy.array([0.0,0.0,0.0])
        self.desired_velocity_rot_transformed  = numpy.array([0.0,0.0,0.0])
        # Initialize trajectory velocity for object rotation
        self.world_trajectory_velocity = numpy.array([0.0,0.0,0.0])
        # Singularity avoidance
        self.singularity_avoidance_stop = False
        self.singularity_velocity_trans_transformed  = numpy.array([0.0,0.0,0.0])
        self.singularity_velocity_rot_transformed  = numpy.array([0.0,0.0,0.0])
        # Franka workspace violation
        self.workspace_violation_limit = 0.68
        self.workspace_violation = False
        self.workspace_violation_msg = WorkspaceViolation()
        
        self.CARTESIAN_POSE = None


    def __init__(self):
        # * Load config parameters
        self.config()
        
        # * Initialize node
        rospy.init_node("franka_hardware_node")
        
        # * Get namespace for topics from launch file
        self.namespace = rospy.get_param("~panda_ns")

        # * Initialize tf TransformListener
        self.tf_listener = tf.TransformListener()

        # Wait for transformations in tf tree
        rospy.loginfo("Wait for transformation '/panda_link8' to 'world'.")
        self.tf_listener.waitForTransform("/panda_link8","/world", rospy.Time(), rospy.Duration(5.0))
        rospy.loginfo("Wait for transformation '/panda_link0' to '/panda_link8'.")
        self.tf_listener.waitForTransform("/panda_link0","/panda_link8", rospy.Time(), rospy.Duration(5.0))
        rospy.loginfo("Wait for transformation 'world' to '/panda_link0'.")
        self.tf_listener.waitForTransform("world","/panda_link0", rospy.Time(), rospy.Duration(5.0))

        # Wait for transformations from 'world' to 'panda_gripper' and 'world' to 'ur16e_gripper'
        rospy.loginfo("Wait for transformation 'world' to '/panda_EE'.")
        self.tf_listener.waitForTransform("world","/panda_EE", rospy.Time(), rospy.Duration(10.0))
        # rospy.loginfo("Wait for transformation 'world' to 'ur16e_gripper'.")
        # self.tf_listener.waitForTransform("world","ur16e_gripper", rospy.Time(), rospy.Duration(10.0))

        # * Initialize subscriber:
        self.cartesian_msg_sub = rospy.Subscriber(
            '/cooperative_manipulation/cartesian_velocity_command',
            Twist,
            self.cartesian_msg_callback,
            queue_size=1,
            tcp_nodelay=True)

        self.cartesian_msg_sub = rospy.Subscriber(
            '/cooperative_manipulation/singularity_velocity', 
            SingularityAvoidance, 
            self.singularity_velocity_callback,
            queue_size=1,
            tcp_nodelay=True)

        # * Initialize publisher:
        self.velocity_command_publisher = rospy.Publisher(
                '/' + self.namespace + '/franka_cartesian_impedance_controller/desired_velocity',
                Float64MultiArray,
                tcp_nodelay=True,
                queue_size=1)

        # Publish singularity velocity
        self.workspace_violation_pub = rospy.Publisher(
            "/cooperative_manipulation/franka/workspace",
            WorkspaceViolation,
            queue_size=1)

        # * Initialize on_shutdown clean up
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("Launch Franka Hardware Node.")
        
        # Moveit, Publisher and Subscriber for measurements-------------------------------------------------------------
        # moveit_commander.roscpp_initialize(sys.argv)

        # try:
        #     group_name = 'panda_arm'
        #     print("Initialize movit_commander. Group name: ",group_name)
        #     self.group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers=5.0)
        # except Exception as e:
        #     print(e)
        
        # * Initialize subscriber:
        self.cartesian_state_sub = rospy.Subscriber(
            '/panda/franka_state_controller/franka_states',
            FrankaState,
            self._get_franka_state,
            queue_size=1,
            tcp_nodelay=True)
        
        
        self.delta_pos_msg = Float64MultiArray()
        self.delta_ori_msg = Float64MultiArray()
        
        self.delta_pos_publisher = rospy.Publisher(
                '/' + self.namespace + '/measurement/delta_pos',
                Float64MultiArray,
                tcp_nodelay=True,
                queue_size=1)
        
        self.delta_ori_publisher = rospy.Publisher(
                '/' + self.namespace + '/measurement/delta_ori',
                Float64MultiArray,
                tcp_nodelay=True,
                queue_size=1)
        
        # Wait for messages to be populated before proceeding
        rospy.loginfo("Subscribing to robot state topics...")
        while (True):
            if not ( self.CARTESIAN_POSE is None):
                print(self.CARTESIAN_POSE)
                break
        rospy.loginfo("Recieved messages; Launch Franka Impedance Control.")
        
        
        
        
        # * Get start position and orientation
        start_pose = copy.deepcopy(self.CARTESIAN_POSE)
        start_pos, start_ori = start_pose['position'],start_pose['orientation']

        # * Initialize self.goal_pos and self.goal_ori
        self.goal_pos = numpy.asarray(start_pos.reshape([1,3]))
        self.goal_ori = start_ori
        
        
        
        # Moveit, Publisher and Subscriber for measurements-------------------------------------------------------------
        
        # # * Run controller thread
        self.control_thread()
        rospy.spin()

    # Callback for measurements-----------------------------------------------------------------------------------------
    
    def _get_franka_state(self,msg):
        """
            Callback function to get current end-point state.

        Args:
            msg (franka_core_msgs.msg.EndPointState): Current tip-state state
        """
        # pose message received is a vectorised column major transformation matrix
        cart_pose_trans_mat = numpy.asarray(msg.O_T_EE).reshape(4,4,order='F')

        # print("cart_pose_trans_mat")
        # print(cart_pose_trans_mat)
        
        self.CARTESIAN_POSE = {
            'position': cart_pose_trans_mat[:3,3],
            'orientation': quaternion.from_rotation_matrix(cart_pose_trans_mat[:3,:3]) }
        

    # Callback for measurements-----------------------------------------------------------------------------------------


    def control_thread(self):
        """
            Actual control loop. Uses goal pose from the feedback thread
            and current robot states from the subscribed messages to compute
            task-space force, and then the corresponding joint torques.
        """
        
        # For measurements----------------------------------------------------------------------------------------------
        # Calculate the translational and rotation movement
        # Declare movement_trans and movement_ori
        movement_trans = numpy.array([None])
        movement_ori = numpy.array([None])
        
        time_old = rospy.Time.now()
        time_old = time_old.to_sec() - 0.01

        
        # For measurements----------------------------------------------------------------------------------------------
        
        
        # Set rospy.rate
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            # * Check self.target_cartesian_trans_velocity and self.target_cartesian_trot_velocity for the min/max velocity limits
            # Calculate the norm of target_cartesian_velocity (trans and rot)
            target_cartesian_trans_velocity_norm = numpy.linalg.norm(self.desired_velocity_trans_transformed)
            target_cartesian_rot_velocity_norm = numpy.linalg.norm(self.desired_velocity_rot_transformed)

            # Check whether the trans/rot velocity  limit has been exceeded. If the trans/rot velocity max limit has been exceeded, then normalize the velocity to the length of the velocity upper limit
            if target_cartesian_trans_velocity_norm > self.cartesian_velocity_trans_max_limit:
                for i in range(3):
                    self.desired_velocity_trans_transformed[i] = (self.desired_velocity_trans_transformed[i]/target_cartesian_trans_velocity_norm) * self.cartesian_velocity_trans_max_limit

            if target_cartesian_rot_velocity_norm > self.cartesian_velocity_rot_max_limit:
                for i in range(3):
                    self.desired_velocity_rot_transformed[i] = (self.desired_velocity_rot_transformed[i]/target_cartesian_rot_velocity_norm) * self.cartesian_velocity_rot_max_limit


        # For measurements----------------------------------------------------------------------------------------------
            time_now = rospy.Time.now()
            time_now = time_now.to_sec()
            time_diff = numpy.round(time_now - time_old,3)
            time_old = time_now
            # Get current position and orientation
            curr_pose = copy.deepcopy(self.CARTESIAN_POSE)
            curr_pos, curr_ori = curr_pose['position'],curr_pose['orientation']
 

            movement_trans = numpy.asarray([x / self.publish_rate for x in self.desired_velocity_trans_transformed]).reshape([1,3])
            movement_ori = self.euler_to_quaternion(numpy.asarray([x / self.publish_rate for x in self.desired_velocity_rot_transformed]))
            
            # Add the movement to current pose and orientation
            self.goal_pos = (self.goal_pos + movement_trans)
            self.goal_ori = self.add_quaternion(self.goal_ori,movement_ori)
         
            # Calculate position and orientation difference
            self.delta_pos = (self.goal_pos - curr_pos).reshape([3,1])
            self.delta_ori = self.quatdiff_in_euler(curr_ori, self.goal_ori).reshape([3,1])
            
        
            
            print("self.delta_pos")
            print(self.delta_pos)
            
            print("self.delta_ori")
            print(self.delta_ori)
            
            self.delta_pos_msg.data = self.delta_pos
            self.delta_ori_msg.data = self.delta_ori
            
            self.delta_pos_publisher.publish(self.delta_pos_msg)
            self.delta_ori_publisher.publish(self.delta_ori_msg)
       
        # For measurements----------------------------------------------------------------------------------------------





            #* Check for workspace violation
            self.check_workspace_violation(self.workspace_violation_limit)

            #* Add singular_velocity to self.desired_velocity_trans_transformed and self.desired_velocity_rot_transformed 
            if self.singularity_avoidance_stop == False and self.workspace_violation == False:
                self.desired_velocity_trans_transformed = numpy.subtract(self.desired_velocity_trans_transformed,self.singularity_velocity_trans_transformed)
                self.desired_velocity_rot_transformed = numpy.subtract(self.desired_velocity_rot_transformed,self.singularity_velocity_rot_transformed)
            else:
                self.desired_velocity_trans_transformed = numpy.array([0.0,0.0,0.0])
                self.desired_velocity_rot_transformed = numpy.array([0.0,0.0,0.0])
                
            #* Publish the velocity command to franka impedance controller   
            self.command_msg.data = numpy.append(self.desired_velocity_trans_transformed,self.desired_velocity_rot_transformed) 

            self.velocity_command_publisher.publish(self.command_msg)
            rate.sleep()

    def transform_vector(self,source_frame: str,target_frame: str,input_vector: numpy.array):
        """ 
            Transforms a vector from source frame to target frame.

        Args:
            source_frame (str): The frame to transform from 
            target_frame (str): The frame to transform to
            input_array (numpy.array): The vector in source frame as array

        Returns:
            numpy.array: The vector in target frame as array
        """
        source_frame_cartesian_velocity_trans = Vector3Stamped()
        source_frame_cartesian_velocity_rot = Vector3Stamped()
        
        # Get current time stamp
        now = rospy.Time()
 
        # Converse input_vector translation from numpy.array to vector3
        source_frame_cartesian_velocity_trans.header.frame_id = source_frame
        source_frame_cartesian_velocity_trans.header.stamp = now
        source_frame_cartesian_velocity_trans.vector.x = input_vector[0]
        source_frame_cartesian_velocity_trans.vector.y = input_vector[1]
        source_frame_cartesian_velocity_trans.vector.z = input_vector[2]
        
        # Transform input_vector translation from 'wrist_3_link' frame to 'base_link' frame
        target_frame_cartesian_velocity_trans = self.tf_listener.transformVector3(target_frame,source_frame_cartesian_velocity_trans)
        
        # Converse input_vector rotation from numpy.array to vector3
        source_frame_cartesian_velocity_rot.header.frame_id = source_frame
        source_frame_cartesian_velocity_rot.header.stamp = now
        source_frame_cartesian_velocity_rot.vector.x = input_vector[3]
        source_frame_cartesian_velocity_rot.vector.y = input_vector[4]
        source_frame_cartesian_velocity_rot.vector.z = input_vector[5]
        
        # Transform input_vector rotation from 'wrist_3_link' frame to 'base_link' frame
        target_frame_cartesian_velocity_transrot = self.tf_listener.transformVector3(target_frame,source_frame_cartesian_velocity_rot)
        
        # Converse input_vector from vector3 to numpy.array
        output_vector = numpy.array([
            target_frame_cartesian_velocity_trans.vector.x,
            target_frame_cartesian_velocity_trans.vector.y,
            target_frame_cartesian_velocity_trans.vector.z,
            target_frame_cartesian_velocity_transrot.vector.x,
            target_frame_cartesian_velocity_transrot.vector.y,
            target_frame_cartesian_velocity_transrot.vector.z
            ])
        
        return output_vector
    
    def cartesian_msg_callback(self,desired_velocity):
        """
            Get the cartesian velocity command and transform it from the 'world' frame to the 'panda_link8' (EE-frame)frame and from the 'panda_link8' frame to the 'panda_link0' (0-frame)frame.

            rostopic pub -r 10 /cooperative_manipulation/cartesian_velocity_command geometry_msgs/Twist "linear:
            x: 0.0
            y: 0.0
            z: 0.0
            angular:
            x: 0.0
            y: 0.0
            z: 0.0"

        Args:
            desired_velocity (geometry_msgs.msg.Twist): Desired cartesian velocity
        """
        # Get current time stamp
        now = rospy.Time()
        # Calculate the trajectory velocity of the manipulator for a rotation of the object-----------------------------
        # Calculate the trajectory velocity of the manipulator for a rotation of the object
        #Get self.panda_current_position, self.panda_current_quaternion of the '/panda_link8' frame in the 'world' frame
        # panda_tf_time = self.tf_listener.getLatestCommonTime("/world", "/panda_link8")
        # panda_current_position, panda_current_quaternion = self.tf_listener.lookupTransform("/world", "/panda_link8", panda_tf_time)


        # # Get self.panda_current_position, self.panda_current_quaternion of the '/panda_EE' frame in the 'world' frame
        # panda_tf_time = self.tf_listener.getLatestCommonTime("/world", "/panda_EE")
        # panda_EE_position, panda_EE_quaternion = self.tf_listener.lookupTransform("/world", "/panda_EE", panda_tf_time)

        # # Get ur16e_current_position, ur16e_current_quaternion of the 'wrist_3_link' in frame in the 'world' frame
        # ur16e_tf_time = self.tf_listener.getLatestCommonTime("/world", "/wrist_3_link")
        # ur16e_gripper_position, ur16e_gripper_quaternion = self.tf_listener.lookupTransform("/world", "/ur16e_gripper", ur16e_tf_time)

        # # Object rotation around x axis
        # if desired_velocity.angular.x != 0.0:
        #     panda_current_position_x = numpy.array([
        #         0.0,
        #         panda_current_position[1],
        #         panda_current_position[2]
        #         ])

        #     self.robot_distance_x = numpy.array([
        #             0.0,
        #             ur16e_gripper_position[1] - panda_EE_position[1],
        #             ur16e_gripper_position[2] - panda_EE_position[2],
        #         ])

        #     center_x = (numpy.linalg.norm(self.robot_distance_x)/2) * (1/numpy.linalg.norm(self.robot_distance_x)) * self.robot_distance_x + panda_EE_position
        #     world_desired_rotation_x = numpy.array([desired_velocity.angular.x,0.0,0.0])
        #     world_radius_x = panda_current_position_x - center_x
        #     self.world_trajectory_velocity_x = numpy.cross(world_desired_rotation_x,world_radius_x)
        #     self.world_trajectory_velocity = self.world_trajectory_velocity + self.world_trajectory_velocity_x

        # # Object rotation around y axis
        # if desired_velocity.angular.y != 0.0:
        #     panda_current_position_y = numpy.array([
        #         panda_current_position[0],
        #         0.0,
        #         panda_current_position[2]
        #         ])

        #     self.robot_distance_y = numpy.array([
        #         ur16e_gripper_position[0] - panda_EE_position[0],
        #         0.0,
        #         ur16e_gripper_position[2] - panda_EE_position[2],
        #         ])

        #     center_y = (numpy.linalg.norm(self.robot_distance_y)/2) * (1/numpy.linalg.norm(self.robot_distance_y)) * self.robot_distance_y + panda_EE_position
        #     world_desired_rotation_y = numpy.array([0.0,desired_velocity.angular.y,0.0])
        #     world_radius_y = panda_current_position_y - center_y
        #     self.world_trajectory_velocity_y = numpy.cross(world_desired_rotation_y,world_radius_y)
        #     self.world_trajectory_velocity = self.world_trajectory_velocity + self.world_trajectory_velocity_y

        # # Object rotation around z axis
        # if desired_velocity.angular.z != 0.0:
        #     panda_current_position_z = numpy.array([
        #         panda_current_position[0],
        #         panda_current_position[1],
        #         0.0,
        #         ])

        #     self.robot_distance_z = numpy.array([
        #         ur16e_gripper_position[0] - panda_EE_position[0],
        #         ur16e_gripper_position[1] - panda_EE_position[1],
        #         0.0,
        #         ])

        #     center_z = (numpy.linalg.norm(self.robot_distance_z)/2) * (1/numpy.linalg.norm(self.robot_distance_z)) * self.robot_distance_z + panda_EE_position
        #     world_desired_rotation_z = numpy.array([0.0,0.0,desired_velocity.angular.z])
        #     world_radius_z = panda_current_position_z - center_z
        #     self.world_trajectory_velocity_z = numpy.cross(world_desired_rotation_z,world_radius_z)
        #     self.world_trajectory_velocity = self.world_trajectory_velocity + self.world_trajectory_velocity_z

        # Transform the velocity from 'world' frame to 'panda_link0' frame----------------------------------------------

        world_cartesian_velocity_trans  = Vector3Stamped()
        world_cartesian_velocity_rot  = Vector3Stamped()
        # Converse cartesian_velocity translation to vector3
        world_cartesian_velocity_trans.header.frame_id = 'world'
        world_cartesian_velocity_trans.header.stamp = now
        world_cartesian_velocity_trans.vector.x = desired_velocity.linear.x + self.world_trajectory_velocity[0]
        world_cartesian_velocity_trans.vector.y = desired_velocity.linear.y + self.world_trajectory_velocity[1]
        world_cartesian_velocity_trans.vector.z = desired_velocity.linear.z + self.world_trajectory_velocity[2]

        #Transform cartesian_velocity translation from 'world' frame to 'panda_link0' frame 
        panda_link0_cartesian_velocity_trans = self.tf_listener.transformVector3('panda_link0',world_cartesian_velocity_trans)

        # Converse cartesian_velocity rotation to vector3
        world_cartesian_velocity_rot.header.frame_id = 'world'
        world_cartesian_velocity_rot.header.stamp = now
        world_cartesian_velocity_rot.vector.x = desired_velocity.angular.x
        world_cartesian_velocity_rot.vector.y = desired_velocity.angular.y
        world_cartesian_velocity_rot.vector.z = desired_velocity.angular.z

        #Transform cartesian_velocity rotation from 'world' frame to 'panda_link0' frame 
        panda_link0_cartesian_velocity_rot = self.tf_listener.transformVector3('panda_link0',world_cartesian_velocity_rot)


        # Converse cartesian_velocity from vector3 to numpy.array
        self.desired_velocity_trans_transformed = [
            panda_link0_cartesian_velocity_trans.vector.x,
            panda_link0_cartesian_velocity_trans.vector.y,
            panda_link0_cartesian_velocity_trans.vector.z,
            ]

        self.desired_velocity_rot_transformed = [
            panda_link0_cartesian_velocity_rot.vector.x,
            panda_link0_cartesian_velocity_rot.vector.y,
            panda_link0_cartesian_velocity_rot.vector.z,
            ]
        
        # Set the trajectory velocity for an object rotation to zero
        self.world_trajectory_velocity = [0.0,0.0,0.0]

    def singularity_velocity_callback(self,singularity_velocity):
        """
            Get the singularity velocity command and transform it into the 'panda_link0' frame.
            
        Args:
            singularity_velocity (Float64MultiArray): Singularity avoidance velocity.
        """
        self.singularity_avoidance_stop = singularity_velocity.singularity_stop
        singularity_velocity_transformed = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        singularity_velocity_transformed = self.transform_vector('world','panda_link0',singularity_velocity.singularity_velocity)
        
        self.singularity_velocity_trans_transformed = [
            singularity_velocity_transformed[0],
            singularity_velocity_transformed[1],
            singularity_velocity_transformed[2],
            ]
            
        self.singularity_velocity_rot_transformed = [
            singularity_velocity_transformed[3],
            singularity_velocity_transformed[4],
            singularity_velocity_transformed[5],
            ] 
        

    def check_workspace_violation(self,workspace_limit: float):
        """_summary_

        Args:
            workspace_limit (float): _description_
        """
        panda_tf_time_1 = self.tf_listener.getLatestCommonTime("/world", "/panda_link1")
        panda_current_position_1, panda_current_quaternion = self.tf_listener.lookupTransform("/world", "/panda_link1", panda_tf_time_1)
        
        
        panda_tf_time_2 = self.tf_listener.getLatestCommonTime("/world", "/panda_link6")
        panda_current_position_2, panda_current_quaternion = self.tf_listener.lookupTransform("/world", "/panda_link6", panda_tf_time_2)

        max_workspace = numpy.array([panda_current_position_2[0] - panda_current_position_1[0],
                                          panda_current_position_2[1] - panda_current_position_1[1],
                                          panda_current_position_2[2] - panda_current_position_1[2]
                                          ])
  
        if numpy.linalg.norm(max_workspace) >= workspace_limit:
            self.workspace_violation = True
            rospy.loginfo("Workspace violation!")
            # print("self.workspace_violation ")
            # print(self.workspace_violation )
            # print("workspace_vector")
            # print(numpy.linalg.norm(max_workspace))
            
        self.workspace_violation_msg.workspace_violation = self.workspace_violation
        self.workspace_violation_pub.publish(self.workspace_violation_msg)
        
    def add_quaternion(self,quat_0: numpy.quaternion,quat_1: numpy.quaternion):
        """
            Add two quaternions and return the sum.
            
        Args:
            quat_0 (numpy.quaternion): First quaternion
            quat_1 (numpy.quaternion): Second quaternion
            
        Returns:
            numpy.quaternion: Sum of both quaternions
        """
        # Extract the values from Q0
        w_0 = quat_0.w
        x_0 = quat_0.x
        y_0 = quat_0.y
        z_0 = quat_0.z
        # Extract the values from Q1
        w_1 = quat_1.w
        x_1 = quat_1.x
        y_1 = quat_1.y
        z_1 = quat_1.z
        # Compute the product of the two quaternions, term by term
        sum_w = w_0 * w_1 - x_0 * x_1 - y_0 * y_1 - z_0 * z_1
        sum_x = w_0 * x_1 + x_0 * w_1 + y_0 * z_1 - z_0 * y_1
        sum_y = w_0 * y_1 - x_0 * z_1 + y_0 * w_1 + z_0 * x_1
        sum_z = w_0 * z_1 + x_0 * y_1 - y_0 * x_1 + z_0 * w_1
        
        sum_quat = numpy.quaternion(sum_w ,sum_x,sum_y,sum_z)
        
        return sum_quat 
    
    def euler_to_quaternion(self,euler_array: numpy.array):
        """
            Convert Euler angles to a quaternion.
            
            Args:
                :param alpha: Rotation around x-axis angle in radians.
                :param beta: Rotation around y-axis  angle in radians.
                :param gamma: Rotation around z-axis angle in radians.
            
            Returns:
                :return quaternion_from_euler: The orientation in quaternion 
        """
        alpha, beta, gamma = euler_array

        q_x = numpy.sin(alpha/2) * numpy.cos(beta/2) * numpy.cos(gamma/2) - numpy.cos(alpha/2) * numpy.sin(beta/2) * numpy.sin(gamma/2)
        q_y = numpy.cos(alpha/2) * numpy.sin(beta/2) * numpy.cos(gamma/2) + numpy.sin(alpha/2) * numpy.cos(beta/2) * numpy.sin(gamma/2)
        q_z = numpy.cos(alpha/2) * numpy.cos(beta/2) * numpy.sin(gamma/2) - numpy.sin(alpha/2) * numpy.sin(beta/2) * numpy.cos(gamma/2)
        q_w = numpy.cos(alpha/2) * numpy.cos(beta/2) * numpy.cos(gamma/2) + numpy.sin(alpha/2) * numpy.sin(beta/2) * numpy.sin(gamma/2)
        
        quaternion_from_euler = numpy.quaternion(q_w,q_x,q_y,q_z)
        
        return quaternion_from_euler
    

    def quatdiff_in_euler(self,quat_curr: numpy.quaternion, quat_des: numpy.quaternion):
        """            
            Compute difference between quaternions and return 
            Euler angles as difference.

        Args:
            quat_curr (numpy.quaternion): Current orientation 
            quat_des (numpy.quaternion): Desired orientation

        Returns:
            numpy.array: Difference between quaternions
        """
        curr_mat = quaternion.as_rotation_matrix(quat_curr)
        des_mat = quaternion.as_rotation_matrix(quat_des)
        rel_mat = des_mat.T.dot(curr_mat)
        rel_quat = quaternion.from_rotation_matrix(rel_mat)
        vec = quaternion.as_float_array(rel_quat)[1:]
        if rel_quat.w < 0.0:
            vec = -vec
        
        return -des_mat.dot(vec)

    def _on_shutdown(self):
        """
            Shutdown publisher and subscriber when rosnode dies.
        """
        print("Shutdown impedance controller:")
        print("Shutdown publisher joint velocity!")
        self.velocity_command_publisher.publish(self.velocity_command_publisher)
        print("Unregister from cartesian_state_sub!")
        self.cartesian_state_sub.unregister()


if __name__ == "__main__":
    franka_hardware_node()