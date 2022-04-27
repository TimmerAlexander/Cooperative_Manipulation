#!/usr/bin/env python3

# /***************************************************************************
# *Admittance controller

# * Input: 
# * desired cartesian velocity from path planning: self.desired_velocity (In 'base_link' frame)
# * external wrench from the f/t sensor: self.wrench_ext (In 'wrist_3_link' frame)

# * Output: 
# * target joint velocity: self.target_joint_velocity (In 'base_link' frame)
# **************************************************************************/

import sys
import rospy
import tf
import moveit_commander
from geometry_msgs.msg import WrenchStamped, Vector3Stamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import numpy

#J_ur=self.group.get_jacobian_matrix(self.current_joint_states_array)

class ur_admittance_controller():
    
    def config(self):
        # Damping gains
        self.D_trans_x = 10
        self.D_trans_y = 10
        self.D_trans_z = 10
        self.D_rot_x = 10
        self.D_rot_y = 10
        self.D_rot_z = 10
        # Stiffness gains
        #self.C_x = 1.0
        #self.C_y = 1.0
        #self.C_z = 1.0
        # Min and max limits for the cartesian velocity (unit: [m/s])
        self.cartesian_velocity_min_limit = 0.001
        self.cartesian_velocity_max_limit = 100
        # Min and max limits for extern wrench
        self.wrench_ext_min_filter = 0.001
        self.wrench_ext_max_filter = 0.1
        # Set publish rate
        self.publish_rate = 100 # Hz
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
        #  Todo: Delete in ready code--------------------------------
        # Initialize desired velocity (xdot_desired_wrist_3_link)
        self.desired_velocity = numpy.array([0.0,0.0,0.01,0.0,0.0,0.0])
        # Todo: -----------------------------------------------------
        # Initialize desired velocity transformed form 'wrist_3_link' frame to 'base_link' frame (xdot_desired_baselink)
        self.desired_velocity_transformed = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
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
        rospy.loginfo("admittance controller running")
        
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
        self.joint_velocity_pub = rospy.Publisher("/" + self.namespace + "/ur_admittance_controller/command", Float64MultiArray, queue_size=1)
        
        # * Initialize subscriber:
        # Subscriber to "/ur/wrench"
        self.wrench_ext_sub = rospy.Subscriber("/" + self.namespace + "/ft_sensor/raw", WrenchStamped, self.wrench_callback)
        
        # * Initialize tf TransformListener
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("wrist_3_link","base_link", rospy.Time(), rospy.Duration(4.0))

        
        
        
        # * Run publish_joint_velocity_thread
        self.publish_joint_velocity_thread()
        
        rospy.spin()
    
    
    def wrench_callback(self,wrench_ext):
        """ 
        Get external wrench.
        
        Send example wrench:
        rostopic pub  /ur/wrench geometry_msgs/WrenchStamped '{header: {stamp: now, frame_id: base_link}, wrench:{force: {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}'
        """
        #? Wozu das? -------------------------------------------------------------------------------------------------
        #self.wrench_force_filtered_x = self.wrench_force_filtered_x*(1-self.wrench_filter) + wrench_ext.wrench.wrench.force.x * self.wrench_filter 
        #self.wrench_force_filtered_y = self.wrench_force_filtered_y*(1-self.wrench_filter) + wrench_ext.wrench.wrench.force.y * self.wrench_filter 
        #self.wrench_force_filtered_z = self.wrench_force_filtered_z*(1-self.wrench_filter) + wrench_ext.wrench.wrench.force.z * self.wrench_filter 
        #? ----------------------------------------------------------------------------------------------------------
        
        
        # Filter min wrench
        if abs(wrench_ext.wrench.force.x) < self.wrench_ext_min_filter:
            # Set extern wrench min limit
            wrench_ext.wrench.force.x = 0.0
        if abs(wrench_ext.wrench.force.y) < self.wrench_ext_min_filter:
            # Set extern wrench min limit
            wrench_ext.wrench.force.y = 0.0
        if abs(wrench_ext.wrench.force.z) < self.wrench_ext_min_filter:
            # Set extern wrench min limit
            wrench_ext.wrench.force.z = 0.0
        if abs(wrench_ext.wrench.torque.x) < self.wrench_ext_min_filter:
            # Set extern wrench min limit
            wrench_ext.wrench.torque.x = 0.0
        if abs(wrench_ext.wrench.torque.y) < self.wrench_ext_min_filter:
            # Set extern wrench min limit
            wrench_ext.wrench.torque.y = 0.0
        if abs(wrench_ext.wrench.torque.z) < self.wrench_ext_min_filter:
            # Set extern wrench min limit
            wrench_ext.wrench.torque.z = 0.0

        # Filter max wrench
        if abs(wrench_ext.wrench.force.x) > self.wrench_ext_max_filter:
            # Check for sign
            if numpy.sign(wrench_ext.wrench.force.x) == 1:
                # Set cartesian velocity max limit
                wrench_ext.wrench.force.x = self.wrench_ext_max_filter
            elif numpy.sign(wrench_ext.wrench.force.x) == -1:
            # Set cartesian velocity max limit
                wrench_ext.wrench.force.x = -self.wrench_ext_max_filter
            else:
                sys.exit('Sign could not be detected!') 
        if abs(wrench_ext.wrench.force.y) > self.wrench_ext_max_filter:
            # Check for sign
            if numpy.sign(wrench_ext.wrench.force.y) == 1:
                # Set cartesian velocity max limit
                wrench_ext.wrench.force.y = self.wrench_ext_max_filter
            elif numpy.sign(wrench_ext.wrench.force.y) == -1:
            # Set cartesian velocity max limit
                wrench_ext.wrench.force.y = -self.wrench_ext_max_filter
            else:
                sys.exit('Sign could not be detected!') 
        if abs(wrench_ext.wrench.force.z) > self.wrench_ext_max_filter:
            # Check for sign
            if numpy.sign(wrench_ext.wrench.force.z) == 1:
                # Set cartesian velocity max limit
                wrench_ext.wrench.force.z = self.wrench_ext_max_filter
            elif numpy.sign(wrench_ext.wrench.force.z) == -1:
            # Set cartesian velocity max limit
                wrench_ext.wrench.force.z = -self.wrench_ext_max_filter
            else:
                sys.exit('Sign could not be detected!') 
        if abs(wrench_ext.wrench.torque.x) > self.wrench_ext_max_filter:
            # Check for sign
            if numpy.sign(wrench_ext.wrench.torque.x) == 1:
                # Set cartesian velocity max limit
                wrench_ext.wrench.torque.x = self.wrench_ext_max_filter
            elif numpy.sign(wrench_ext.wrench.torque.x) == -1:
            # Set cartesian velocity max limit
                wrench_ext.wrench.torque.x = -self.wrench_ext_max_filter
            else:
                sys.exit('Sign could not be detected!')   
        if abs(wrench_ext.wrench.torque.y) > self.wrench_ext_max_filter:
            # Check for sign
            if numpy.sign(wrench_ext.wrench.torque.y) == 1:
                # Set cartesian velocity max limit
                wrench_ext.wrench.torque.y = self.wrench_ext_max_filter
            elif numpy.sign(wrench_ext.wrench.torque.y) == -1:
            # Set cartesian velocity max limit
                wrench_ext.wrench.torque.y = -self.wrench_ext_max_filter
            else:
                sys.exit('Sign could not be detected!')  
        if abs(wrench_ext.wrench.torque.z) > self.wrench_ext_max_filter:
            # Check for sign
            if numpy.sign(wrench_ext.wrench.torque.z) == 1:
                # Set cartesian velocity max limit
                wrench_ext.wrench.torque.z = self.wrench_ext_max_filter
            elif numpy.sign(wrench_ext.wrench.torque.z) == -1:
            # Set cartesian velocity max limit
                wrench_ext.wrench.torque.z = -self.wrench_ext_max_filter
            else:
                sys.exit('Sign could not be detected!')       
        
        #print("wrench_ext : ")
        #print(wrench_ext)
        
        self.wrench_ext_filtered = wrench_ext
    
    # ? In welchem Koordiantensystem wird die Bahn berechnet?
    def transform_velocity(self,cartesian_velocity: numpy.array):
        """ 
        Transform the cartesian velocity from the 'wrist_3_link' frame to the 'base_link' frame.
        """
        
        
        # -----------------------------
        # Get current time stamp
        now = rospy.Time()
        # Get position of wrist_3_link
        (curr_position,curr_orientation) = self.listener.lookupTransform('world', 'wrist_3_link', now)
        print("curr_position:")
        print(curr_position)
        # -----------------------------
        
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
        
        # Todo: Vielleicht runden!? x = round(number),digit)
        return self.velocity_transformed

    def publish_joint_velocity_thread(self):
        """ 
        This thread calculates and publishes the target joint velocity.
        """
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            
            #print("desired_velocity: ")
            #print(self.desired_velocity)
            
            # * Calculate velocity from external wrench and admittance in 'wrist_3_link' frame
            self.admittance_velocity[0] = self.wrench_ext_filtered.wrench.force.x * pow(self.D_trans_x,-1)
            self.admittance_velocity[1] = self.wrench_ext_filtered.wrench.force.y * pow(self.D_trans_y,-1)
            self.admittance_velocity[2] = self.wrench_ext_filtered.wrench.force.z * pow(self.D_trans_z,-1)
            self.admittance_velocity[3] = self.wrench_ext_filtered.wrench.torque.x * pow(self.D_rot_x,-1)
            self.admittance_velocity[4] = self.wrench_ext_filtered.wrench.torque.y * pow(self.D_rot_y,-1)
            self.admittance_velocity[5] = self.wrench_ext_filtered.wrench.torque.z * pow(self.D_rot_z,-1)
            
            # * Transform velocity admittance from 'wrist_3_link' frame to 'base_link' frame
            self.admittance_velocity_transformed = self.transform_velocity(self.admittance_velocity)
            
            #print("self.admittance_velocity_transformed")
            #print(self.admittance_velocity_transformed)


            # * Transform desired velocity from 'wrist_3_link' frame to 'base_link' frame
            self.desired_velocity_transformed = self.transform_velocity(self.desired_velocity)
            
            #print("self.desired_velocity_transformed")
            #print(self.desired_velocity_transformed)
            
            # * Add the desired_velocity in 'base_link' frame and velocity admittance in 'base_link' frame
            self.target_cartesian_velocity[0] = self.desired_velocity_transformed[0] + self.admittance_velocity_transformed[0]
            self.target_cartesian_velocity[1] = self.desired_velocity_transformed[1] + self.admittance_velocity_transformed[1]
            self.target_cartesian_velocity[2] = self.desired_velocity_transformed[2] + self.admittance_velocity_transformed[2]
            self.target_cartesian_velocity[3] = self.desired_velocity_transformed[3] + self.admittance_velocity_transformed[3]
            self.target_cartesian_velocity[4] = self.desired_velocity_transformed[4] + self.admittance_velocity_transformed[4]
            self.target_cartesian_velocity[5] = self.desired_velocity_transformed[5] + self.admittance_velocity_transformed[5]

            #print("target_cartesian_velocity: befor check for limits")
            #print(self.target_cartesian_velocity)
        
        
            # * Check self.target_cartesian_velocity for the min/max velocity limits
            # Check for cartesian velocity min limit
            if abs(self.target_cartesian_velocity[0]) < self.cartesian_velocity_min_limit:
                # Set cartesian velocity min limit
                self.target_cartesian_velocity[0] = 0.0
            if abs(self.target_cartesian_velocity[1]) < self.cartesian_velocity_min_limit:
                # Set cartesian velocity min limit
                self.target_cartesian_velocity[1] = 0.0
            if abs(self.target_cartesian_velocity[2]) < self.cartesian_velocity_min_limit:
                # Set cartesian velocity min limit
                self.target_cartesian_velocity[2] = 0.0
            if abs(self.target_cartesian_velocity[3]) < self.cartesian_velocity_min_limit:
                # Set cartesian velocity min limit
                self.target_cartesian_velocity[3] = 0.0
            if abs(self.target_cartesian_velocity[4]) < self.cartesian_velocity_min_limit:
                # Set cartesian velocity min limit
                self.target_cartesian_velocity[4] = 0.0
            if abs(self.target_cartesian_velocity[5]) < self.cartesian_velocity_min_limit:
                # Set cartesian velocity min limit
                self.target_cartesian_velocity[5] = 0.0
                
            # * Check for cartesian velocity max limit
            if abs(self.target_cartesian_velocity[0]) > self.cartesian_velocity_max_limit:
                # Check for sign
                if numpy.sign(self.target_cartesian_velocity[0]) == 1:
                    # Set cartesian velocity max limit
                    self.target_cartesian_velocity[0] = self.cartesian_velocity_max_limit
                elif numpy.sign(self.target_cartesian_velocity[0]) == -1:
                    # Set cartesian velocity max limit
                    self.target_cartesian_velocity[0] = -self.cartesian_velocity_max_limit
                else:
                    sys.exit('Sign could not be detected!') 
            if abs(self.target_cartesian_velocity[1]) > self.cartesian_velocity_max_limit:
                # Check for sign
                if numpy.sign(self.target_cartesian_velocity[1]) == 1:
                    # Set cartesian velocity max limit
                    self.target_cartesian_velocity[1] = self.cartesian_velocity_max_limit
                elif numpy.sign(self.target_cartesian_velocity[1]) == -1:
                    # Set cartesian velocity max limit
                    self.target_cartesian_velocity[1] = -self.cartesian_velocity_max_limit
                else:
                    sys.exit('Sign could not be detected!')
            if abs(self.target_cartesian_velocity[2]) > self.cartesian_velocity_max_limit:
                # Check for sign
                if numpy.sign(self.target_cartesian_velocity[2]) == 1:
                    # Set cartesian velocity max limit
                    self.target_cartesian_velocity[2] = self.cartesian_velocity_max_limit
                elif numpy.sign(self.target_cartesian_velocity[2]) == -1:
                    # Set cartesian velocity max limit
                    self.target_cartesian_velocity[2] = -self.cartesian_velocity_max_limit
                else:
                    sys.exit('Sign could not be detected!')
            if abs(self.target_cartesian_velocity[3]) > self.cartesian_velocity_max_limit:
                # Check for sign
                if numpy.sign(self.target_cartesian_velocity[3]) == 1:
                    # Set cartesian velocity max limit
                    self.target_cartesian_velocity[3] = self.cartesian_velocity_max_limit
                elif numpy.sign(self.target_cartesian_velocity[3]) == -1:
                    # Set cartesian velocity max limit
                    self.target_cartesian_velocity[3] = -self.cartesian_velocity_max_limit
                else:
                    sys.exit('Sign could not be detected!')
            if abs(self.target_cartesian_velocity[4]) > self.cartesian_velocity_max_limit:
                # Check for sign
                if numpy.sign(self.target_cartesian_velocity[4]) == 1:
                    # Set cartesian velocity max limit
                    self.target_cartesian_velocity[4] = self.cartesian_velocity_max_limit
                elif numpy.sign(self.target_cartesian_velocity[4]) == -1:
                    # Set cartesian velocity max limit
                    self.target_cartesian_velocity[4] = -self.cartesian_velocity_max_limit
                else:
                    sys.exit('Sign could not be detected!')
            if abs(self.target_cartesian_velocity[5]) > self.cartesian_velocity_max_limit:
                # Check for sign
                if numpy.sign(self.target_cartesian_velocity[5]) == 1:
                    # Set cartesian velocity max limit
                    self.target_cartesian_velocity[5] = self.cartesian_velocity_max_limit
                elif numpy.sign(self.target_cartesian_velocity[5]) == -1:
                    # Set cartesian velocity max limit
                    self.target_cartesian_velocity[5] = -self.cartesian_velocity_max_limit
                else:
                    sys.exit('Sign could not be detected!')
                    
                    
            #print("target_cartesian_velocity: after check for limits")
            #print(self.target_cartesian_velocity)
            
            
            # * Get the current joint states 
            self.current_joint_states_array = self.group.get_current_joint_values() 
            
            print("self.current_joint_states_array: ")
            print(self.current_joint_states_array)
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

#  Todo: shutdown_joint_velocity wird nicht gepublished
    def shutdown(self):
        """ This function is called rospy.on_shutdown!
        """
        print("Shutdown amittcance controller!")
        
        print("Publish shutdown joint velocity!")
        self.joint_velocity_pub.publish(self.shutdown_joint_velocity)
        print(self.shutdown_joint_velocity)
        
        print("Unregister from joint_velocity_pub!")
        self.joint_velocity_pub.unregister()
        
        print("Unregister from wrench_ext_sub!")
        self.wrench_ext_sub.unregister()
    
if __name__ == '__main__':
    ur_admittance_controller()









# ? Was ist das?--------------------------------------------------------
#        self.cmd_vel_filtered.linear.x = self.cmd_vel_filtered.linear.x * (1-self.cmd_vel_filter) + self.cmd_vel.linear.x * self.cmd_vel_filter
#        self.cmd_vel_filtered.linear.y = self.cmd_vel_filtered.linear.y * (1-self.cmd_vel_filter) + self.cmd_vel.linear.y * self.cmd_vel_filter
#        self.cmd_vel_filtered.linear.z = self.cmd_vel_filtered.linear.z * (1-self.cmd_vel_filter) + self.cmd_vel.linear.z * self.cmd_vel_filter

        # limit velocity
#        if abs(self.cmd_vel_filtered.linear.x)> self.velocity_threshhold:
#            self.cmd_vel_filtered.linear.x = self.cmd_vel_filtered.linear.x/abs(self.cmd_vel_filtered.linear.x) * self.velocity_threshhold

#       if abs(self.cmd_vel_filtered.linear.y)> self.velocity_threshhold:
#            self.cmd_vel_filtered.linear.y = self.cmd_vel_filtered.linear.y/abs(self.cmd_vel_filtered.linear.y) * self.velocity_threshhold

#        if abs(self.cmd_vel_filtered.linear.z)> self.velocity_threshhold:
#            self.cmd_vel_filtered.linear.z = self.cmd_vel_filtered.linear.z/abs(self.cmd_vel_filtered.linear.z) * self.velocity_threshhold

#    def check_velocity_limits(self,unfiltered_cartesian_velocity :numpy.array):

        
                
#        self.filtered_cartesian_velocity = unfiltered_cartesian_velocity
#        return self.filtered_cartesian_velocity


