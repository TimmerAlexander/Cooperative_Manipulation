#!/usr/bin/env python3

# /***************************************************************************
# *Admittance controller

# * Input: 
# * desired cartesian velocity from path planning: self.desired_velocity (In 'base_link' frame)
# * external wrench from the f/t sensor: self.wrench_ext (In 'tool0' frame)

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
        self.D_x = 10
        self.D_y = 10
        self.D_z = 10
        # Stiffness gains
        self.C_x = 1.0
        self.C_y = 1.0
        self.C_z = 1.0
        # Set min and max limits for the cartesian velocity
        self.cartesian_velocity_min_limit = 0.001
        self.cartesian_velocity_max_limit = 0.1
        # Set publish rate
        self.publish_rate = 100 # Hz
        # Initialize wrench_filtered
        self.wrench_filtered_x = 0.0
        self.wrench_filtered_y = 0.0
        self.wrench_filtered_z = 0.0
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
        # Initialize desired velocity (xdot_desired_tool0)
        self.desired_velocity = numpy.array([0.01,0.0,0.0,0.0,0.0,0.0])
        # Todo: --------------------------------
        # Initialize desired velocity transformed form 'tool0' frame to 'base_link' frame (xdot_desired_baselink)
        self.desired_velocity_transformed = numpy.array([0.01,0.0,0.0,0.0,0.0,0.0])
        # Initialize velocity from admittance (xdot_a_tool0)
        self.velocity_admittance = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Initialize velocity from admittance 'tool0' frame to 'base_link' frame  (xdot_a_baselink)
        self.velocity_admittance_transformed  = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Initialize target cartesian velocity array (xdot_target_baselink)
        self.target_cartesian_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Declare target joint velocity msgs (qdot_target_baselink)
        self.target_joint_velocity = Float64MultiArray()
        # Initialize velocity (xdot_desired_tool0)
        self.velocity_transformed = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.tool0_cartesian_velocity = Vector3Stamped()
        # Initialize shutdown joint velocity, called on shutdown 
        self.shutdown_joint_velocity = Float64MultiArray()
        self.shutdown_joint_velocity.data = [0.0,0.0,0.0,0.0,0.0,0.0]
        
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
        self.wrench_ext_sub = rospy.Subscriber("/" + self.namespace + "/wrench", WrenchStamped, self.wrench_cb)
        
        # * Initialize tf TransformListener
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("tool0","base_link", rospy.Time(), rospy.Duration(4.0))

        # * Run publish_joint_velocity_thread
        self.publish_joint_velocity_thread()
        
        rospy.spin()
    
    
    def wrench_cb(self,wrench_ext):
        """ Get external wrench.
        
        Send example wrenc:
        rostopic pub  /ur/wrench geometry_msgs/WrenchStamped '{header: {stamp: now, frame_id: base_link}, wrench:{force: {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}'
        """
        print("wrench_ext:")
        print(wrench_ext)
        
        #? Wozu das? ---------------------------------------------------------------------------------------------
        #self.wrench_filtered_x = self.wrench_filtered_x*(1-self.wrench_filter) + wrench_ext.wrench.force.x * self.wrench_filter 
        #self.wrench_filtered_y = self.wrench_filtered_y*(1-self.wrench_filter) + wrench_ext.wrench.force.y * self.wrench_filter 
        #self.wrench_filtered_z = self.wrench_filtered_z*(1-self.wrench_filter) + wrench_ext.wrench.force.z * self.wrench_filter 
        #? -------------------------------------------------------------------------------------------------------
        
        self.wrench_filtered_x = wrench_ext.wrench.force.x
        self.wrench_filtered_y = wrench_ext.wrench.force.y
        self.wrench_filtered_z = wrench_ext.wrench.force.z
        
    
    # ? In welchem Koordiantensystem wird die Bahn berechnet?
    def transform_velocity(self,cartesian_velocity: numpy.array):
        """ Transform the cartesian velocity from the 'tool0' frame to the 'base_link' frame
        """
        # Get current time stamp
        now = rospy.Time()
        
        # Converse cartesian_velocity from numpy.array to vector3
        self.tool0_cartesian_velocity.header.frame_id = 'tool0'
        self.tool0_cartesian_velocity.header.stamp = now
        self.tool0_cartesian_velocity.vector.x = cartesian_velocity[0]
        self.tool0_cartesian_velocity.vector.y = cartesian_velocity[1]
        self.tool0_cartesian_velocity.vector.z = cartesian_velocity[2]
        
        # Transform cartesian_velocity from 'tool0' frame to 'base_link' frame
        self.base_link_cartesian_velocity = self.listener.transformVector3('base_link',self.tool0_cartesian_velocity)
        
        # Converse cartesian_velocity from vector3 to numpy.array
        self.velocity_transformed = numpy.array([self.base_link_cartesian_velocity.vector.x,self.base_link_cartesian_velocity.vector.y,self.base_link_cartesian_velocity.vector.z,0.0,0.0,0.0])
        
        return self.velocity_transformed
        
    def publish_joint_velocity_thread(self):
        """ This thread calculates and publishes the target joint velocity.
        """
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            
            print("desired_velocity: ")
            print(self.desired_velocity)
            
            # Calculate velocity from external wrench and admittance in 'tool0' frame
            self.velocity_admittance[0] = self.wrench_filtered_x * pow(self.D_x,-1)
            self.velocity_admittance[1] = self.wrench_filtered_y * pow(self.D_y,-1)
            self.velocity_admittance[2] = self.wrench_filtered_z * pow(self.D_z,-1)
            
            # Transform velocity admittance from 'tool0' frame to 'base_link' frame
            self.velocity_admittance_transformed = self.transform_velocity(self.velocity_admittance)
            
            #print("self.velocity_admittance_transformed")
            #print(self.velocity_admittance_transformed)

            #  Todo: Uncomment in ready code--------------------------------
            # Calculate the target cartesian velocity using the admittance
            # for i in self.target_cartesian_velocity:
            #    self.target_cartesian_velocity[i] = self.desired_velocity[i] + self.velocity_admittance_transformed[i]
            #  Todo: ------------------------------------------------------

            # * Placeholder--------------------------------------------------------
            #  Todo: Uncomment in ready code--------------------------------
            # Transform desired velocity from 'tool0' frame to 'base_link' frame
            self.desired_velocity_transformed = self.transform_velocity(self.desired_velocity)
            
            print("self.desired_velocity_transformed")
            print(self.desired_velocity_transformed)
            
            for i in range(0,6):
                self.target_cartesian_velocity[i] = self.desired_velocity_transformed[i] + self.velocity_admittance_transformed[i]
            #  Todo: -----------------------------------------------------
            # * Placeholder--------------------------------------------------------

                # Check for cartesian velocity min limit
                if abs(self.target_cartesian_velocity[i]) < self.cartesian_velocity_min_limit:
                    # Set cartesian velocity min limit
                    self.target_cartesian_velocity[i] = 0.0

                # Check for cartesian velocity max limit
                if abs(self.target_cartesian_velocity[i]) > self.cartesian_velocity_max_limit:
                    # Check for sign
                    if numpy.sign(self.target_cartesian_velocity[0]) == 1:
                        # Set cartesian velocity max limit
                        self.target_cartesian_velocity[i] = self.cartesian_velocity_max_limit
                    elif numpy.sign(self.target_cartesian_velocity[0]) == -1:
                        # Set cartesian velocity max limit
                        self.target_cartesian_velocity[i] = -self.cartesian_velocity_max_limit
                    else:
                        sys.exit('Sign could not be detected!')
                        
            
            print("target_cartesian_velocity")
            print(self.target_cartesian_velocity)
            
            # Get the current joint states 
            self.current_joint_states_array = self.group.get_current_joint_values() 
            # Calculate the jacobian-matrix
            self.jacobian = self.group.get_jacobian_matrix(self.current_joint_states_array) 
            # Calculate the inverse of the jacobian-matrix
            self.inverse_jacobian = numpy.linalg.inv(self.jacobian)

            # Calculate the target joint velocity with the inverse jacobian-matrix and the target cartesain velociy
            self.target_joint_velocity.data = self.inverse_jacobian.dot(self.target_cartesian_velocity)
            
            print("target_joint_velocity: ")
            print(self.target_joint_velocity)
            
            # Publish the target_joint_velocity
            self.joint_velocity_pub.publish(self.target_joint_velocity)
            
            # Sleep for publish_rate
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
        
        # ? Brauche ich das? -------------------------------------
     #   joint_states_array=[joint_states.position[0],joint_states.position[1],joint_states.position[2],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
      
        
     #   self.J_ur=self.group.get_jacobian_matrix(joint_states_array)
        # ? ------------------------------------------------------

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





