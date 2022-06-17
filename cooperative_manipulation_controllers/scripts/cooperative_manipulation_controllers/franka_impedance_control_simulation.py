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
    Description...
    
    Impedance controller
    
    Input: 
    * Desired cartesian velocity of the EE: desired_velocity (In 'world' frame)
    * External wrench from the f/t sensor: wrench_ext (In 'panda/panda_link7' frame)
    
    Output: 
    * Joint effort: self.command_msg.effort (Franka joints)
"""
import copy, numpy, quaternion
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Twist, Vector3Stamped, WrenchStamped, TransformStamped
from franka_core_msgs.msg import EndPointState, JointCommand, RobotState

class franka_impedance_controller():
    
    def config(self):
        # Stiffness gains
        self.P_trans_x = 50.
        self.P_trans_y = 50.
        self.P_trans_z = 50.
        self.P_rot_x = 50.
        self.P_rot_y = 50.
        self.P_rot_z = 50.
        # Damping gains
        self.D_trans_x = 10.
        self.D_trans_y = 10.
        self.D_trans_z = 10.
        self.D_rot_x = 1.
        self.D_rot_y = 1.
        self.D_rot_z = 1.
        # Wrench compliance gains
        self.wrench_force_x = 0.2
        self.wrench_force_y = 0.2
        self.wrench_force_z = 0.2
        self.wrench_torque_x = 0.2
        self.wrench_torque_y = 0.2
        self.wrench_torque_z = 0.2
        # Min and max limits for the cartesian velocity (trans/rot) (unit: [m/s],[rad/s])
        self.cartesian_velocity_trans_min_limit = 0.0009
        self.cartesian_velocity_trans_max_limit = 0.1
        self.cartesian_velocity_rot_min_limit = 0.0009
        self.cartesian_velocity_rot_max_limit = 0.1
        # Control thread publish rate
        self.publish_rate = 100 # [Hz]
        # Wrench filter force treshold (1.5 - 2.7)
        self.wrench_filter_force = 1.55
        # Wrench filter torque treshold (0.14 - 0.2)
        self.wrench_filter_torque = 0.14
        # Declare librarys for jacobian matrix, cartesian pose and velocity
        self.JACOBIAN = None
        self.CARTESIAN_POSE = None
        self.CARTESIAN_VEL = None
        # Create joint command message and fix its type to joint torque mode
        self.command_msg = JointCommand()
        self.command_msg.names = ['panda_joint1','panda_joint2','panda_joint3',\
            'panda_joint4','panda_joint5','panda_joint6','panda_joint7']
        self.command_msg.mode = JointCommand.TORQUE_MODE
        # Initialize position and orientation difference
        self.delta_pos = numpy.array([0.0,0.0,0.0]).reshape([3,1])
        self.delta_ori = numpy.array([0.0,0.0,0.0]).reshape([3,1])
        # Initialize linear and angular velocity difference
        self.delta_linear = numpy.array([0.0,0.0,0.0]).reshape([3,1])
        self.delta_angular = numpy.array([0.0,0.0,0.0]).reshape([3,1])
        # Initialize desired translational and rotation velocity
        self.desired_velocity_trans_transformed  = numpy.array([0.0,0.0,0.0])
        self.desired_velocity_rot_transformed  = numpy.array([0.0,0.0,0.0])
        # Initialize external wrench        
        self.wrench_force_velocity_transformed = numpy.array([0.0,0.0,0.0,])
        self.wrench_torque_velocity_transformed = numpy.array([0.0,0.0,0.0,])
        # Initialize trajectory velocity for object rotation
        self.world_trajectory_velocity = numpy.array([0.0,0.0,0.0])
        # The offset of the object to the 'panda/panda_link8' frame        
        self.panda_gripper_offset = 0.10655

        
    def __init__(self):
        # * Load config parameters
        self.config()

        # * Initialize node
        rospy.init_node("ts_control_sim_only")
        
        # * Initialize tf TransformBroadcaster
        self.brodacaster = tf2_ros.StaticTransformBroadcaster()
        # * Initialize tf TransformListener
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform("panda/panda_link8","world", rospy.Time(), rospy.Duration(5.0))
        rospy.loginfo("Waited for transformation 'panda/panda_link8' to 'world'.")
        self.tf_listener.waitForTransform("panda/base","panda/panda_link8", rospy.Time(), rospy.Duration(5.0))
        rospy.loginfo("Waited for transformation 'panda/base' to 'panda/panda_link8'.")
        self.tf_listener.waitForTransform("world","base_link", rospy.Time(), rospy.Duration(5.0))
        rospy.loginfo("Waited for transformation 'world' to 'base_link'.")

        self.set_gripper_offset()

        self.tf_listener.waitForTransform("world","ur16e_gripper", rospy.Time(), rospy.Duration(10.0))
        rospy.loginfo("Waited for transformation 'world' to 'ur16e_gripper'.")



        # ! If not using franka_ros_interface, you have to subscribe to the right topics to obtain the current end-effector state and robot jacobian for computing commands
        # * Initialize subscriber:
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
        
        self.wrench_msg_sub = rospy.Subscriber(
            '/gazebo/robot/wrist/ft', 
            WrenchStamped, 
            self.wrench_msg_callback,
            queue_size=1,
            tcp_nodelay=True)
        
        # * Initialize publisher:
        # Also create a publisher to publish joint commands
        self.joint_command_publisher = rospy.Publisher(
                'panda_simulator/motion_controller/arm/joint_commands',
                JointCommand,
                tcp_nodelay=True,
                queue_size=1)
        

        # Wait for messages to be populated before proceeding
        rospy.loginfo("Subscribing to robot state topics...")
        while (True):
            if not (self.JACOBIAN is None or self.CARTESIAN_POSE is None):
                print(self.JACOBIAN,self.CARTESIAN_POSE)
                break
        rospy.loginfo("Recieved messages; Launch Franka Impedance Control.")
        
        # * Initialize on_shutdown clean up
        rospy.on_shutdown(self._on_shutdown)
        
        # * Get start position and orientation
        start_pose = copy.deepcopy(self.CARTESIAN_POSE)
        start_pos, start_ori = start_pose['position'],start_pose['orientation']
        
        # * Initialize self.goal_pos and self.goal_ori
        self.goal_pos = numpy.asarray(start_pos.reshape([1,3]))
        self.goal_ori = start_ori
        
        # * Initialize P_trans/P_rot and D_trans/D_rot numpy.array
        self.P_trans = numpy.array([self.P_trans_x,self.P_trans_y,self.P_trans_z]).reshape([3,1])
        self.P_rot = numpy.array([self.P_rot_x,self.P_rot_y,self.P_rot_z]).reshape([3,1])
        self.D_trans = numpy.array([self.D_trans_x,self.D_trans_y,self.D_trans_z]).reshape([3,1])
        self.D_rot = numpy.array([self.D_rot_x,self.D_rot_y,self.D_rot_z]).reshape([3,1])

        
        # * Run controller thread
        self.control_thread()
        
        rospy.spin()    
    
    def set_gripper_offset(self):
        """
            Set the gripper offset from 'panda/panda_link8' frame.
        """
        static_gripper_offset = TransformStamped()
        static_gripper_offset.header.stamp = rospy.Time.now()
        static_gripper_offset.header.frame_id = "/panda/panda_link8"
        static_gripper_offset.child_frame_id = "panda/panda_gripper"
        static_gripper_offset.transform.translation.x = 0.0
        static_gripper_offset.transform.translation.y = 0.0
        static_gripper_offset.transform.translation.z = self.panda_gripper_offset
        static_gripper_offset.transform.rotation.x = 0.0
        static_gripper_offset.transform.rotation.y = 0.0
        static_gripper_offset.transform.rotation.z =  -0.924
        static_gripper_offset.transform.rotation.w =  0.383

        self.brodacaster.sendTransform(static_gripper_offset)

    def control_thread(self):
        """
            Actual control loop. Uses goal pose from the feedback thread
            and current robot states from the subscribed messages to compute
            task-space force, and then the corresponding joint torques.
        """
        # Set rospy.rate
        rate = rospy.Rate(self.publish_rate)
        # Declare movement_trans and movement_ori
        movement_trans = numpy.array([None])
        movement_ori = numpy.array([None])
        
        while not rospy.is_shutdown():
            self.set_gripper_offset()
            # Get current position and orientation 
            curr_pose = copy.deepcopy(self.CARTESIAN_POSE)
            curr_pos, curr_ori = curr_pose['position'],curr_pose['orientation']
            # Get current linear and angular velocity
            current_vel_trans = (self.CARTESIAN_VEL['linear']).reshape([3,1])
            current_vel_rot = (self.CARTESIAN_VEL['angular']).reshape([3,1])
            
            # print("current_vel_trans")
            # print(numpy.linalg.norm(current_vel_trans))

            
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
                    
                
            # Check whether the velocity limit has been undershot. If the velocity  has fallen below the min velocity limit, then set the velocity  to zero
            if target_cartesian_trans_velocity_norm < self.cartesian_velocity_trans_min_limit:
                for i in range(3):
                    self.desired_velocity_trans_transformed[i] = 0.0
                
            if target_cartesian_rot_velocity_norm < self.cartesian_velocity_rot_min_limit:
                for i in range(3):
                    self.desired_velocity_rot_transformed[i] = 0.0
                    
                    
            # Calculate the translational and rotation movement
            movement_trans = numpy.asarray([x / self.publish_rate for x in self.desired_velocity_trans_transformed]).reshape([1,3])
            movement_ori = self.euler_to_quaternion(numpy.asarray([x / self.publish_rate for x in self.desired_velocity_rot_transformed]))
           
            
            
            # Add the movement to current pose and orientation
            self.goal_pos = (self.goal_pos + movement_trans)
            self.goal_ori = self.add_quaternion(self.goal_ori,movement_ori)
            # Calculate position and orientation difference
            self.delta_pos = (self.goal_pos - curr_pos).reshape([3,1])
            self.delta_ori = self.quatdiff_in_euler(curr_ori, self.goal_ori).reshape([3,1])
            
            
            
            # Calculate linear and angular velocity difference
            self.delta_linear = numpy.array(self.desired_velocity_trans_transformed).reshape([3,1]) - current_vel_trans + numpy.array(self.wrench_force_velocity_transformed).reshape([3,1])
            self.delta_angular = numpy.array(self.desired_velocity_rot_transformed).reshape([3,1]) - current_vel_rot + numpy.array(self.wrench_torque_velocity_transformed).reshape([3,1])
            
            # print("self.delta_angular")
            # print(self.delta_angular)
            
            # Desired task-space force using PD law
            F = numpy.vstack([numpy.multiply(self.P_trans,self.delta_pos), numpy.multiply(self.P_rot,self.delta_ori)]) + numpy.vstack([numpy.multiply(self.D_trans,self.delta_linear), numpy.multiply(self.D_rot,self.delta_angular)])

            J = copy.deepcopy(self.JACOBIAN)
            
            # joint torques to be commanded
            tau = numpy.dot(J.T,F)
            
            # publish joint commands
            self.command_msg.effort = tau.flatten()
            self.joint_command_publisher.publish(self.command_msg)
            rate.sleep()
                        
    def _on_robot_state(self,msg):
        """
            Callback function for updating jacobian and EE velocity from robot state.
            
        Args:
            msg (franka_core_msgs.msg.RobotState): Current robot state
        """
        self.JACOBIAN = numpy.asarray(msg.O_Jac_EE).reshape(6,7,order = 'F')
        self.CARTESIAN_VEL = {
                    'linear': numpy.asarray([msg.O_dP_EE[0], msg.O_dP_EE[1], msg.O_dP_EE[2]]),
                    'angular': numpy.asarray([msg.O_dP_EE[3], msg.O_dP_EE[4], msg.O_dP_EE[5]]) }

    def _on_endpoint_state(self,msg):
        """
            Callback function to get current end-point state.
            
        Args:
            msg (franka_core_msgs.msg.EndPointState): Current tip-state state
        """
        # pose message received is a vectorised column major transformation matrix
        cart_pose_trans_mat = numpy.asarray(msg.O_T_EE).reshape(4,4,order='F')
        
        self.CARTESIAN_POSE = {
            'position': cart_pose_trans_mat[:3,3],
            'orientation': quaternion.from_rotation_matrix(cart_pose_trans_mat[:3,:3]) }
    
        
    def cartesian_msg_callback(self,desired_velocity):
        """
            Get the cartesian velocity command and transform it from the 'world' frame to the 'panda/panda_link8' (EE-frame)frame and from the 'panda/panda_link8' frame to the 'panda/base' (0-frame)frame.
            
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
        
        # print("desired_velocity")
        # print(desired_velocity)
        
        # Calculate the trajectory velocity of the manipulator for a rotation of the object
        # Get self.panda_current_position, self.panda_current_quaternion of the '/panda/panda_link8' frame in the 'world' frame 
        panda_tf_time = self.tf_listener.getLatestCommonTime("/world", "/panda/panda_link8")
        panda_current_position, panda_current_quaternion = self.tf_listener.lookupTransform("/world", "/panda/panda_link8", panda_tf_time)


        # Get self.panda_current_position, self.panda_current_quaternion of the '/panda/panda_gripper' frame in the 'world' frame 
        panda_tf_time = self.tf_listener.getLatestCommonTime("/world", "/panda/panda_gripper")
        panda_gripper_position, panda_gripper_quaternion = self.tf_listener.lookupTransform("/world", "/panda/panda_gripper", panda_tf_time)

        # Get ur16e_current_position, ur16e_current_quaternion of the 'wrist_3_link' in frame in the 'world' frame 
        ur16e_tf_time = self.tf_listener.getLatestCommonTime("/world", "/wrist_3_link")
        ur16e_gripper_position, ur16e_gripper_quaternion = self.tf_listener.lookupTransform("/world", "/ur16e_gripper", ur16e_tf_time)
    
        # print("self.ur16e_current_position, self.ur16e_current_quaternion")
        # print(self.ur16e_current_position, self.ur16e_current_quaternion)
        # print("self.panda_current_position, self.panda_current_quaternion")
        # print(self.panda_position, self.panda_current_quaternion)

        # Object rotation around x axis 
        if desired_velocity.angular.x != 0.0:
            panda_current_position_x = numpy.array([
                0.0,
                panda_current_position[1],
                panda_current_position[2]
                ])

            # print("panda_current_position_x")
            # print(panda_current_position_x)

            self.robot_distance_x = numpy.array([
                0.0,
                ur16e_gripper_position[1] - panda_gripper_position[1],
                ur16e_gripper_position[2] - panda_gripper_position[2],
            ])
            
            # print(" self.robot_distance_x: y,z")
            # print( self.robot_distance_x)
        
            center_x = (numpy.linalg.norm(self.robot_distance_x)/2) * (1/numpy.linalg.norm(self.robot_distance_x)) * self.robot_distance_x + panda_gripper_position
            world_desired_rotation_x = numpy.array([desired_velocity.angular.x,0.0,0.0])
            
            # print("world_desired_rotation_x")
            # print(world_desired_rotation_x)
            
            world_radius_x = panda_current_position_x - center_x
            
            # print("world_radius_x")
            # print(world_radius_x)
            
            self.world_trajectory_velocity_x = numpy.cross(world_desired_rotation_x,world_radius_x)
            self.world_trajectory_velocity = self.world_trajectory_velocity + self.world_trajectory_velocity_x
            
            # print("self.world_trajectory_velocity")
            # print(self.world_trajectory_velocity)
            
            
        # Object rotation around y axis 
        if desired_velocity.angular.y != 0.0: 
            panda_current_position_y = numpy.array([
                panda_current_position[0],
                0.0,
                panda_current_position[2]
                ]) 

            self.robot_distance_y = numpy.array([
                ur16e_gripper_position[0] - panda_gripper_position[0],
                0.0,
                ur16e_gripper_position[2] - panda_gripper_position[2],
                ])
            
            center_y = (numpy.linalg.norm(self.robot_distance_y)/2) * (1/numpy.linalg.norm(self.robot_distance_y)) * self.robot_distance_y + panda_gripper_position
            world_desired_rotation_y = numpy.array([0.0,desired_velocity.angular.y,0.0])
            world_radius_y = panda_current_position_y - center_y
            self.world_trajectory_velocity_y = numpy.cross(world_desired_rotation_y,world_radius_y)
            self.world_trajectory_velocity = self.world_trajectory_velocity + self.world_trajectory_velocity_y
            

            
        # Object rotation around z axis 
        if desired_velocity.angular.z != 0.0:
            panda_current_position_z = numpy.array([
                panda_current_position[0],
                panda_current_position[1],
                0.0,
                ]) 

            self.robot_distance_z = numpy.array([
                ur16e_gripper_position[0] - panda_gripper_position[0],
                ur16e_gripper_position[1] - panda_gripper_position[1],
                0.0,
                ])
            
            center_z = (numpy.linalg.norm(self.robot_distance_z)/2) * (1/numpy.linalg.norm(self.robot_distance_z)) * self.robot_distance_z + panda_gripper_position
            world_desired_rotation_z = numpy.array([0.0,0.0,desired_velocity.angular.z])
            world_radius_z = panda_current_position_z - center_z
            self.world_trajectory_velocity_z = numpy.cross(world_desired_rotation_z,world_radius_z)
            self.world_trajectory_velocity = self.world_trajectory_velocity + self.world_trajectory_velocity_z



        # Transform the velcoities from 'world' frame to 'panda/panda_link8' frame
        # Get current time stamp
        now = rospy.Time()
        world_cartesian_velocity_trans  = Vector3Stamped()
        world_cartesian_velocity_rot  = Vector3Stamped()
        # Converse cartesian_velocity translation to vector3
        world_cartesian_velocity_trans.header.frame_id = 'world'
        world_cartesian_velocity_trans.header.stamp = now
        world_cartesian_velocity_trans.vector.x = desired_velocity.linear.x + self.world_trajectory_velocity[0]
        world_cartesian_velocity_trans.vector.y = desired_velocity.linear.y + self.world_trajectory_velocity[1]
        world_cartesian_velocity_trans.vector.z = desired_velocity.linear.z + self.world_trajectory_velocity[2]
            
        # Transform cartesian_velocity translation from 'world' frame to 'panda/base' frame and from 'panda/base' frame to 'panda/panda_link8
        base_cartesian_velocity_trans = self.tf_listener.transformVector3('panda/base',world_cartesian_velocity_trans)
            
        # Converse cartesian_velocity rotation to vector3
        world_cartesian_velocity_rot.header.frame_id = 'world'
        world_cartesian_velocity_rot.header.stamp = now
        world_cartesian_velocity_rot.vector.x = desired_velocity.angular.x
        world_cartesian_velocity_rot.vector.y = desired_velocity.angular.y
        world_cartesian_velocity_rot.vector.z = desired_velocity.angular.z
            
        # Transform cartesian_velocity rotation from 'world' frame to 'panda/base' frame and from 'panda/base' frame to 'panda/panda_link8'
        base_cartesian_velocity_rot = self.tf_listener.transformVector3('panda/base',world_cartesian_velocity_rot)
        
        # print("world_cartesian_velocity_rot")
        # print(world_cartesian_velocity_rot)

        # print("ase_cartesian_velocity_rot")
        # print(base_cartesian_velocity_rot )
        # Converse cartesian_velocity from vector3 to numpy.array
        self.desired_velocity_trans_transformed = [
            base_cartesian_velocity_trans.vector.x,
            base_cartesian_velocity_trans.vector.y,
            base_cartesian_velocity_trans.vector.z,
            ]
            
        self.desired_velocity_rot_transformed = [
            base_cartesian_velocity_rot.vector.x,
            -1 * base_cartesian_velocity_rot.vector.y,
            -1 * base_cartesian_velocity_rot.vector.z,
            ] 

        print("self.desired_velocity_rot_transformed")
        print(self.desired_velocity_rot_transformed)
        # Set the trajectory velocity for an object rotation to zero
        self.world_trajectory_velocity = [0.0,0.0,0.0]
        
    def wrench_msg_callback(self,wrench_ext):
        """ 
            Get external wrench in panda_link7.
        """
        # print("wrench_ext")
        # print(wrench_ext)
        
        # Get current time stamp
        now = rospy.Time()
            
        panda_link7_wrench_force  = Vector3Stamped()
        panda_link7_wrench_torque  = Vector3Stamped()
        # Converse cartesian_velocity translation to vector3
        panda_link7_wrench_force.header.frame_id = 'panda/panda_link7'
        panda_link7_wrench_force.header.stamp = now
        panda_link7_wrench_force.vector.x = wrench_ext.wrench.force.x
        panda_link7_wrench_force.vector.y = wrench_ext.wrench.force.y
        panda_link7_wrench_force.vector.z = wrench_ext.wrench.force.z
            
        # Transform cartesian_velocity translation from 'panda/panda_link7' frame to 'panda/base'
        base_wrench_force = self.tf_listener.transformVector3('panda/base',panda_link7_wrench_force)
            
        # Converse cartesian_velocity rotation to vector3
        panda_link7_wrench_torque.header.frame_id = 'panda/panda_link7'
        panda_link7_wrench_torque.header.stamp = now
        panda_link7_wrench_torque.vector.x = wrench_ext.wrench.torque.x
        panda_link7_wrench_torque.vector.y = wrench_ext.wrench.torque.y
        panda_link7_wrench_torque.vector.z = wrench_ext.wrench.torque.z
            
        # Transform cartesian_velocity rotation from 'panda/panda_link7' frame to 'panda/base'
        base_wrench_torque = self.tf_listener.transformVector3('panda/base',panda_link7_wrench_torque)
            
        # * Band-passfilter
        if numpy.abs(base_wrench_force.vector.x) < self.wrench_filter_force:
            base_wrench_force.vector.x = 0.0
        else: 
            base_wrench_force.vector.x = base_wrench_force.vector.x - numpy.sign(base_wrench_force.vector.x) * self.wrench_filter_force
            
        if(numpy.abs(base_wrench_force.vector.y) < self.wrench_filter_force):
            base_wrench_force.vector.y = 0.0
        else: 
            base_wrench_force.vector.y = base_wrench_force.vector.y - numpy.sign(base_wrench_force.vector.y) * self.wrench_filter_force

        if(numpy.abs(base_wrench_force.vector.z) < self.wrench_filter_force):
            base_wrench_force.vector.z = 0.0
        else: 
           base_wrench_force.vector.z = base_wrench_force.vector.z - numpy.sign(base_wrench_force.vector.z) * self.wrench_filter_force

        if(numpy.abs(base_wrench_torque.vector.x) < self.wrench_filter_torque):
            base_wrench_torque.vector.x = 0.0
        else: 
            base_wrench_torque.vector.x = base_wrench_torque.vector.x - numpy.sign(base_wrench_torque.vector.x) * self.wrench_filter_torque
        if(numpy.abs(base_wrench_torque.vector.y) < self.wrench_filter_torque):
            base_wrench_torque.vector.y = 0.0
        else: 
            base_wrench_torque.vector.y = base_wrench_torque.vector.y - numpy.sign(base_wrench_torque.vector.y) * self.wrench_filter_torque
        if(numpy.abs(base_wrench_torque.vector.z) < self.wrench_filter_torque):
            base_wrench_torque.vector.z = 0.0
        else: 
            base_wrench_torque.vector.z = base_wrench_torque.vector.z - numpy.sign(base_wrench_torque.vector.z) * self.wrench_filter_torque 
            
        
        # Converse cartesian_velocity from vector3 to numpy.array and multipy the compliance gains            
        self.wrench_force_velocity_transformed = [
            base_wrench_force.vector.x * self.wrench_force_x,
            base_wrench_force.vector.y * self.wrench_force_y,
            base_wrench_force.vector.z * self.wrench_force_z,
            ]
        
        self.wrench_torque_velocity_transformed = [
            base_wrench_torque.vector.x * self.wrench_torque_x,
            base_wrench_torque.vector.y * self.wrench_torque_y,
            base_wrench_torque.vector.z * self.wrench_torque_z,
            ]

        # print("self.wrench_force_velocity_transformed")
        # print(self.wrench_force_velocity_transformed)
        
    def euler_to_quaternion(self,euler_array: numpy.array):
        """
            Convert Euler angles to a quaternion.
            
            Inuput
                :param alpha: Rotation around x-axis) angle in radians.
                :param beta: The beta (rotation around y-axis) angle in radians.
                :param gamma: The gamma (rotation around z-axis) angle in radians.
            
            Output
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
    
    def add_quaternion(self,quat_0: numpy.quaternion,quat_1: numpy.quaternion):
        """
            Add two quaternions and return the sum.
            
        Args:
            quat_0 (numpy.quaternion): Firt quaternion
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
        # Computer the product of the two quaternions, term by term
        sum_w = w_0 * w_1 - x_0 * x_1 - y_0 * y_1 - z_0 * z_1
        sum_x = w_0 * x_1 + x_0 * w_1 + y_0 * z_1 - z_0 * y_1
        sum_y = w_0 * y_1 - x_0 * z_1 + y_0 * w_1 + z_0 * x_1
        sum_z = w_0 * z_1 + x_0 * y_1 - y_0 * x_1 + z_0 * w_1
        
        sum_quat = numpy.quaternion(sum_w ,sum_x,sum_y,sum_z)
        
        return sum_quat 
    
    def _on_shutdown(self):
        """
            Shutdown publisher and subscriber when rosnode dies.
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