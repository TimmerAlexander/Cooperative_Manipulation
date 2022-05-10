#!/usr/bin/env python3

# /***************************************************************************

# **************************************************************************/

"""
    Description...
    
    Admittance controller
    
    Input: 
    * Desired cartesian velocity of the EE: desired_velocity (In 'world' frame)
    * External wrench from the f/t sensor: wrench_ext (In 'wrist_3_link' frame)
    
    Output: 
    * Target joint velocity: self.target_joint_velocity (In 'base_link' frame)
"""

import rospy
import tf
import moveit_commander
from geometry_msgs.msg import WrenchStamped, Vector3Stamped
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy, math


class ur_admittance_controller():
    
    def config(self):
        # Stiffness gains
        #self.C_x = 1.0
        #self.C_y = 1.0
        #self.C_z = 1.0
        # Damping gains
        self.D_trans_x = 10.
        self.D_trans_y = 10.
        self.D_trans_z = 10.
        self.D_rot_x = 10.
        self.D_rot_y = 10.
        self.D_rot_z = 10.
        # Min and max limits for the cartesian velocity (trans/rot) (unit: [m/s])
        self.cartesian_velocity_trans_min_limit = 0.001
        self.cartesian_velocity_trans_max_limit = 0.1
        self.cartesian_velocity_rot_min_limit = 0.001
        self.cartesian_velocity_rot_max_limit = 0.01
        # Min and max limits for extern wrench
        self.wrench_ext_min_filter = 0.001
        self.wrench_ext_max_filter = 0.1
        # Control thread publish rate
        self.publish_rate = 100 # (unit: [Hz])
        # Initialize wrench_ext_filtered
        self.wrench_force_filtered_x = 0.0
        self.wrench_force_filtered_y = 0.0
        self.wrench_force_filtered_z = 0.0
        # Wrench filter parameter
        self.wrench_filter = 0.03
        
        # ? Brauche ich das? --------------------------------------------------------
        self.velocity_threshhold = 0.1
        self.cmd_vel_filter = 0.05
        # ? --------------------------------------------------------------------------
        
    def __init__(self):
        # * Load config parameters
        self.config()
        
        # * Initialize the needed velocity data types:
        # Initialize desired velocity transformed form 'wrorld' frame to 'base_link' frame (xdot_desired_baselink)
        self.desired_velocity_transformed = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.world_cartesian_velocity_trans = Vector3Stamped()
        self.world_cartesian_velocity_rot = Vector3Stamped()
        # Initialize velocity from admittance (xdot_a_wrist_3_link)
        self.admittance_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Initialize velocity from admittance 'wrist_3_link' frame to 'base_link' frame  (xdot_a_baselink)
        self.admittance_velocity_transformed  = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Initialize target cartesian velocity array (xdot_target_baselink)
        self.target_cartesian_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Declare target joint velocity msgs (qdot_target_baselink) (unit: [radian/s])
        self.target_joint_velocity = Float64MultiArray()
        # Initialize velocity (xdot_desired_wrist_3_link)
        self.velocity_transformed = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.wrist_3_link_cartesian_velocity_trans = Vector3Stamped()
        self.wrist_3_link_cartesian_velocity_rot = Vector3Stamped()
        # Initialize shutdown joint velocity, called on shutdown 
        self.shutdown_joint_velocity = Float64MultiArray()
        self.shutdown_joint_velocity.data = [0.0,0.0,0.0,0.0,0.0,0.0]
        
        self.wrench_ext_filtered = WrenchStamped()
        
        # * Initialize node
        rospy.init_node('admittance_controller_node', anonymous=True)
        
        # * Initialize on_shutdown clean up
        rospy.on_shutdown(self.shutdown)

        # * Get namespace for topics from launch file
        self.namespace = rospy.get_param("~ur_ns")
        
        # * Initialize move_it
        group_name = 'manipulator'
        print("Initialize movit_commander. Group name: ",group_name)
        self.group = moveit_commander.MoveGroupCommander(group_name)
        
        # * Initialize publisher:
        # Publish final joint velocity to "/ur/ur_admittance_controller/command"
        self.joint_velocity_pub = rospy.Publisher(
            "/" + self.namespace + "/ur_admittance_controller/command",
            Float64MultiArray,
            queue_size=1)
        
        # * Initialize subscriber:
        # Subscriber to "/ur/wrench"
        self.wrench_ext_sub = rospy.Subscriber(
            "/" + self.namespace + "/ft_sensor/raw",
            WrenchStamped,
            self.wrench_callback,queue_size=1)


        # Subscriber to "/ur/cooperative_manipulation/cartesian_velocity_command"
        self.cartesian_velocity_command_sub = rospy.Subscriber(
            "/cooperative_manipulation/cartesian_velocity_command",
            Twist,
            self.cartesian_velocity_command_callback,queue_size=1)


        
        # * Initialize tf TransformListener
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("wrist_3_link","base_link", rospy.Time(), rospy.Duration(4.0))
        self.listener.waitForTransform("world","base_link", rospy.Time(), rospy.Duration(4.0))
        
        # Wait for messages to be populated before proceeding
        rospy.loginfo("Subscribing to robot state topics...")
        while (True):
            if not (self.wrench_ext_filtered is None):
                print("wrench init: ")
                print(self.wrench_ext_filtered)
                break
        rospy.loginfo("Recieved messages; Launch ur16e Admittance control.")
        
        
        # * Run control_thread
        self.control_thread()
        
        rospy.spin()
    
    def cartesian_velocity_command_callback(self,desired_velocity):
        """
            Get the cartesian velocity command and transform it from from the 'world' frame to the 'base_link' frame.
            
            Send example velocity:
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

        # Converse cartesian_velocity translation to vector3
        self.world_cartesian_velocity_trans.header.frame_id = 'world'
        self.world_cartesian_velocity_trans.header.stamp = now
        self.world_cartesian_velocity_trans.vector.x = desired_velocity.linear.x
        self.world_cartesian_velocity_trans.vector.y = desired_velocity.linear.y
        self.world_cartesian_velocity_trans.vector.z = desired_velocity.linear.z
        
        # Transform cartesian_velocity translation from 'world' frame to 'base_link' frame
        self.base_link_cartesian_desired_velocity_trans = self.listener.transformVector3('base_link',self.world_cartesian_velocity_trans)
        
        # Converse cartesian_velocity rotation to vector3
        self.world_cartesian_velocity_rot.header.frame_id = 'world'
        self.world_cartesian_velocity_rot.header.stamp = now
        self.world_cartesian_velocity_rot.vector.x = desired_velocity.angular.x
        self.world_cartesian_velocity_rot.vector.y = desired_velocity.angular.y
        self.world_cartesian_velocity_rot.vector.z = desired_velocity.angular.z
        
        # Transform cartesian_velocity rotation from 'world' frame to 'base_link' frame
        self.base_link_cartesian_desired_velocity_rot = self.listener.transformVector3('base_link',self.    world_cartesian_velocity_rot)
        
        # Converse cartesian_velocity from vector3 to numpy.array
        self.desired_velocity_transformed = [
            self.base_link_cartesian_desired_velocity_trans.vector.x,
            self.base_link_cartesian_desired_velocity_trans.vector.y,
            self.base_link_cartesian_desired_velocity_trans.vector.z,
            self.base_link_cartesian_desired_velocity_rot.vector.x,
            self.base_link_cartesian_desired_velocity_rot.vector.y,
            self.base_link_cartesian_desired_velocity_rot.vector.z
            ]

    
    def wrench_callback(self,wrench_ext):
        """ 
            Get external wrench.
            
            Send example wrench:
            rostopic pub  /ur/wrench geometry_msgs/WrenchStamped '{header: {stamp: now, frame_id: base_link}, wrench:{force: {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}'
        """
    
        print("wrench_ext.wrench before fileter:")
        print(wrench_ext.wrench)
        if(numpy.abs(wrench_ext.wrench.force.x * self.cartesian_velocity_trans_max_limit) < 1.5):
            self.wrench_ext_filtered.wrench.force.x = 0.0
        if(numpy.abs(wrench_ext.wrench.force.y * self.cartesian_velocity_trans_max_limit) < 1.5):
            self.wrench_ext_filtered.wrench.force.y = 0.0
        if(numpy.abs(wrench_ext.wrench.force.z * self.cartesian_velocity_trans_max_limit) < 1.5):
            self.wrench_ext_filtered.wrench.force.z = 0.0
        if(numpy.abs(wrench_ext.wrench.torque.x * self.cartesian_velocity_trans_max_limit) < 1.5):
            self.wrench_ext_filtered.wrench.torque.x = 0.0
        if(numpy.abs(wrench_ext.wrench.torque.y * self.cartesian_velocity_trans_max_limit) < 1.5):
            self.wrench_ext_filtered.wrench.torque.y = 0.0
        if(numpy.abs(wrench_ext.wrench.torque.z * self.cartesian_velocity_trans_max_limit) < 1.5):
            self.wrench_ext_filtered.wrench.torque.z = 0.0
            
        #print("self.wrench_ext_filtered:")
        #print(self.wrench_ext_filtered)
        
    
    
    def transform_velocity(self,cartesian_velocity: numpy.array):
        """ 
            Transform the cartesian velocity from the 'wrist_3_link' frame to the 'base_link' frame.
        """
        # Get current time stamp
        now = rospy.Time()

        # Converse cartesian_velocity translation from numpy.array to vector3
        self.wrist_3_link_cartesian_velocity_trans.header.frame_id = 'wrist_3_link'
        self.wrist_3_link_cartesian_velocity_trans.header.stamp = now
        self.wrist_3_link_cartesian_velocity_trans.vector.x = cartesian_velocity[0]
        self.wrist_3_link_cartesian_velocity_trans.vector.y = cartesian_velocity[1]
        self.wrist_3_link_cartesian_velocity_trans.vector.z = cartesian_velocity[2]
        
        # Transform cartesian_velocity translation from 'wrist_3_link' frame to 'base_link' frame
        self.base_link_cartesian_velocity_trans = self.listener.transformVector3('base_link',self.wrist_3_link_cartesian_velocity_trans)
        
        # Converse cartesian_velocity rotation from numpy.array to vector3
        self.wrist_3_link_cartesian_velocity_rot.header.frame_id = 'wrist_3_link'
        self.wrist_3_link_cartesian_velocity_rot.header.stamp = now
        self.wrist_3_link_cartesian_velocity_rot.vector.x = cartesian_velocity[3]
        self.wrist_3_link_cartesian_velocity_rot.vector.y = cartesian_velocity[4]
        self.wrist_3_link_cartesian_velocity_rot.vector.z = cartesian_velocity[5]
        
        # Transform cartesian_velocity rotation from 'wrist_3_link' frame to 'base_link' frame
        self.base_link_cartesian_velocity_rot = self.listener.transformVector3('base_link',self.    wrist_3_link_cartesian_velocity_rot)
        
        # Converse cartesian_velocity from vector3 to numpy.array
        self.velocity_transformed = numpy.array([
            self.base_link_cartesian_velocity_trans.vector.x,
            self.base_link_cartesian_velocity_trans.vector.y,
            self.base_link_cartesian_velocity_trans.vector.z,
            self.base_link_cartesian_velocity_rot.vector.x,
            self.base_link_cartesian_velocity_rot.vector.y,
            self.base_link_cartesian_velocity_rot.vector.z
            ])
        
        return self.velocity_transformed

    def control_thread(self):
        """ 
            This thread calculates and publishes the target joint velocity using and admittance controller.
        """
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            # * Calculate velocity from external wrench and admittance in 'wrist_3_link' frame
            self.admittance_velocity[0] = self.wrench_ext_filtered.wrench.force.x * pow(self.D_trans_x,-1)
            self.admittance_velocity[1] = self.wrench_ext_filtered.wrench.force.y * pow(self.D_trans_y,-1)
            self.admittance_velocity[2] = self.wrench_ext_filtered.wrench.force.z * pow(self.D_trans_z,-1)
            self.admittance_velocity[3] = self.wrench_ext_filtered.wrench.torque.x * pow(self.D_rot_x,-1)
            self.admittance_velocity[4] = self.wrench_ext_filtered.wrench.torque.y * pow(self.D_rot_y,-1)
            self.admittance_velocity[5] = self.wrench_ext_filtered.wrench.torque.z * pow(self.D_rot_z,-1)
            
            
            self.admittance_velocity_transformed = self.transform_velocity(self.admittance_velocity)
            # print("self.admittance_velocity_transformed")
            # print(self.admittance_velocity_transformed)
            
            # * Add the desired_velocity in 'base_link' frame and admittance velocity in 'base_link' frame
            self.target_cartesian_velocity[0] = self.desired_velocity_transformed[0] + self.admittance_velocity_transformed[0]
            self.target_cartesian_velocity[1] = self.desired_velocity_transformed[1] + self.admittance_velocity_transformed[1]
            self.target_cartesian_velocity[2] = self.desired_velocity_transformed[2] + self.admittance_velocity_transformed[2]
            self.target_cartesian_velocity[3] = self.desired_velocity_transformed[3] + self.admittance_velocity_transformed[3]
            self.target_cartesian_velocity[4] = self.desired_velocity_transformed[4] + self.admittance_velocity_transformed[4]
            self.target_cartesian_velocity[5] = self.desired_velocity_transformed[5] + self.admittance_velocity_transformed[5]
            
            # self.target_cartesian_velocity[0] = self.desired_velocity_transformed[0] 
            # self.target_cartesian_velocity[1] = self.desired_velocity_transformed[1] 
            # self.target_cartesian_velocity[2] = self.desired_velocity_transformed[2] 
            # self.target_cartesian_velocity[3] = self.desired_velocity_transformed[3] 
            # self.target_cartesian_velocity[4] = self.desired_velocity_transformed[4] 
            # self.target_cartesian_velocity[5] = self.desired_velocity_transformed[5] 

            # print("target_cartesian_velocity: befor check for limits")
            # print(self.target_cartesian_velocity)

            # * Check self.target_cartesian_velocity for the min/max velocity limits
            # Calculate the norm of target_cartesian_velocity (trans and rot)
            target_cartesian_trans_velocity_norm = math.sqrt(pow(self.target_cartesian_velocity[0],2) + pow(self.target_cartesian_velocity[1],2) + pow(self.target_cartesian_velocity[2],2))
            
            target_cartesian_rot_velocity_norm = math.sqrt(pow(self.target_cartesian_velocity[3],2) + pow(self.target_cartesian_velocity[4],2) + pow(self.target_cartesian_velocity[5],2))
            
            #  Check for cartesian velocity max limit and set to max limit, if max limit is exceeded
            if target_cartesian_trans_velocity_norm > self.cartesian_velocity_trans_max_limit:
                for i in range(3):
                    self.target_cartesian_velocity[i] = (self.target_cartesian_velocity[i]/target_cartesian_trans_velocity_norm) * self.cartesian_velocity_trans_max_limit
                    
            if target_cartesian_rot_velocity_norm > self.cartesian_velocity_rot_max_limit:
                for i in range(3,6):
                    self.target_cartesian_velocity[i] = (self.target_cartesian_velocity[i]/target_cartesian_rot_velocity_norm) * self.cartesian_velocity_rot_max_limit
            
            # Check for cartesian velocity min limit and set to null, if min limit is understeps
            if target_cartesian_trans_velocity_norm < self.cartesian_velocity_trans_min_limit:
                for i in range(3):
                    self.target_cartesian_velocity[i] = 0.0
            
            if target_cartesian_rot_velocity_norm < self.cartesian_velocity_rot_min_limit:
                for i in range(3,6):
                    self.target_cartesian_velocity[i] = 0.0
            
            # print("target_cartesian_velocity: after check for limits")
            # print(self.target_cartesian_velocity)
            
            # * Get the current joint states 
            self.current_joint_states_array = self.group.get_current_joint_values() 
            
            #print("self.current_joint_states_array: ")
            #print(self.current_joint_states_array)
            
            # * Calculate the jacobian-matrix
            self.jacobian = self.group.get_jacobian_matrix(self.current_joint_states_array) 
            
            # * Calculate the inverse of the jacobian-matrix
            self.inverse_jacobian = numpy.linalg.inv(self.jacobian)

            # * Calculate the target joint velocity with the inverse jacobian-matrix and the target cartesain velociy
            self.target_joint_velocity.data = self.inverse_jacobian.dot(self.target_cartesian_velocity)
            
            #print("target_joint_velocity: ")
            #print(self.target_joint_velocity)
            
            # * Publish the target_joint_velocity
            self.joint_velocity_pub.publish(self.target_joint_velocity)
            
            # * Sleep for publish_rate
            rate.sleep()

    def shutdown(self):
        """ 
            This function is called by rospy.on_shutdown!
        """
        print("Shutdown amittcance controller:")
        print("Shutdown publisher joint velocity!")
        self.joint_velocity_pub.publish(self.shutdown_joint_velocity)
        print("Unregister from joint_velocity_pub!")
        self.joint_velocity_pub.unregister()
        print("Unregister from wrench_ext_sub!")
        self.wrench_ext_sub.unregister()
    
if __name__ == '__main__':
    ur_admittance_controller()
    
        #? Wozu das? -------------------------------------------------------------------------------------------------
        #self.wrench_force_filtered_x = self.wrench_force_filtered_x*(1-self.wrench_filter) + wrench_ext.wrench.wrench.force.x * self.wrench_filter 
        #self.wrench_force_filtered_y = self.wrench_force_filtered_y*(1-self.wrench_filter) + wrench_ext.wrench.wrench.force.y * self.wrench_filter 
        #self.wrench_force_filtered_z = self.wrench_force_filtered_z*(1-self.wrench_filter) + wrench_ext.wrench.wrench.force.z * self.wrench_filter 
        #? ----------------------------------------------------------------------------------------------------------