#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import WrenchStamped, Twist
import tf

import threading

import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy

#J_ur=self.group.get_jacobian_matrix(joint_states_array)

class ur_admittance_controller():
    
    
    def __init__(self):
        # Load config
        self.config()
        # Initialize node
        rospy.init_node('admittance_controller_node', anonymous=True)
        rospy.loginfo("admittance controller running")
        # Get namespace from launch file
        self.namespace = rospy.get_param("~ur_ns")
        # Initialize move_it
        self.joint_vel = Float64MultiArray()
        group_name = 'manipulator'
        print("Initialize movit_commander. Group name: ",group_name)
        self.group = moveit_commander.MoveGroupCommander(group_name)
        # Initialize publisher
        self.pub = rospy.Publisher("/" + self.namespace + "/ur_admittance_controller/command", Float64MultiArray, queue_size=1)
        
        self.listener = tf.TransformListener()
        now = rospy.Time()
        self.listener.waitForTransform("base_link", "tool0", rospy.Time(), rospy.Duration(4.0))
        (self.initial_position,self.initial_orientation) = self.listener.lookupTransform('base_link', 'tool0', now)
        # Print initial position and orientation of 'tool0'
        print("Initial position: ",self.initial_position)
        print("Initial orientation: ",self.initial_orientation)
        # Desiered position and orientation of 'tool0'
        self.goal_position = self.initial_position
        self.goal_orientation = self.initial_orientation
        
        # Get '/ur/wrench'
        rospy.Subscriber("/" + self.namespace + "/wrench", WrenchStamped, self.wrench_cb)
        
        #rospy.spin()
        
        # start controller thread
        ctrl_thread = threading.Thread(target=control_thread, args = [self.publish_rate])
        ctrl_thread.start()
        
        

    def control_thread(rate):
        """Control loop
         """
        print("test")
        while not rospy.is_shutdown():

 
            rate.sleep()
        
        
    def wrench_cb(self,F_ext):
        
        # F_ext == external force
        print("F_ext:")
        print(F_ext)

        now = rospy.Time()
        (curr_position,curr_orientation) = self.listener.lookupTransform('base_link', 'tool0', now)

        self.position_diff_x = self.goal_position[0] - curr_position[0]
        self.position_diff_y = self.goal_position[1] - curr_position[1]
        self.position_diff_z = self.goal_position[2] - curr_position[2]
        
        if self.position_diff_x < self.position_diff_limit:
            self.position_diff_x = 0
        if self.position_diff_y < self.position_diff_limit:
            self.position_diff_y = 0
        if self.position_diff_z < self.position_diff_limit:
            self.position_diff_z = 0

        print("self.position_diff_x,self.position_diff_y,self.position_diff_z")
        print(self.position_diff_x,self.position_diff_y,self.position_diff_z)

        # Force difference
        self.F_diff_x = self.F_d_x*(1-self.wrench_filter) + F_ext.wrench.force.x * self.wrench_filter 
        self.F_diff_y = self.F_d_y*(1-self.wrench_filter) + F_ext.wrench.force.y * self.wrench_filter 
        self.F_diff_z = self.F_d_z*(1-self.wrench_filter) + F_ext.wrench.force.z * self.wrench_filter 

        print("F_diff_x",self.F_diff_x)
        print("F_diff_y",self.F_diff_y)
        print("F_diff_z",self.F_diff_z)

        self.cmd_vel.linear.x = (self.F_diff_x - self.position_diff_x * self.C_x) * pow(self.D_x,-1)
        self.cmd_vel.linear.y = (self.F_diff_y - self.position_diff_y * self.C_y) * pow(self.D_y,-1)
        self.cmd_vel.linear.z = (self.F_diff_z - self.position_diff_z * self.C_z) * pow(self.D_z,-1)

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

        #print("Publish Twist:",self.cmd_vel.linear.x,self.cmd_vel.linear.y)
        #print(self.position_diff_z)
#------------------------------------------------------------------------
        print("cmd_vel: ")
        print("self.cmd_vel.linear.x: ",self.cmd_vel.linear.x)
        print("self.cmd_vel.linear.y: ",self.cmd_vel.linear.y)
        print("self.cmd_vel.linear.z: ",self.cmd_vel.linear.z)
        print("self.cmd_vel.angular.x: ",self.cmd_vel.angular.x)
        print("self.cmd_vel.angular.y: ",self.cmd_vel.angular.y)
        print("self.cmd_vel.angular.z: ",self.cmd_vel.angular.z)
        
        # Save the cmd_vel in an array
        self.target_vel = numpy.array([self.cmd_vel.linear.x,self.cmd_vel.linear.y,self.cmd_vel.linear.z,self.cmd_vel.angular.x,self.cmd_vel.angular.y,self.cmd_vel.angular.z])
        
        #print("Publish self.target_vel: ")
        #print(self.target_vel)
        
    
        joint_states_array = self.group.get_current_joint_values()
        self.Jacobian_ur = self.group.get_jacobian_matrix(joint_states_array)
        
        print("Print Jacobian_ur: ",self.Jacobian_ur)

        inverse = numpy.linalg.inv(self.Jacobian_ur)
        self.target_dq = inverse.dot(self.target_vel)
        self.joint_vel.data = self.target_dq
        print("Joint Velocity: ")
        print(self.target_dq)
        
        
        print("self.joint_vel.data: ",self.joint_vel.data)
        self.pub.publish(self.joint_vel)
#--------------------------------------------------------------------
        #print("Publish Twist: ")
        #print(self.cmd_vel)
        #self.pub.publish(self.cmd_vel)
    

    def config(self):
        # Damping gains
        self.D_x = 0.1
        self.D_y = 0.1
        self.D_z = 0.1
        # Stiffness gains
        self.C_x = 1.0
        self.C_y = 1.0
        self.C_z = 1.0
        # Desired force 
        self.F_d_x = 0.0
        self.F_d_y = 0.0
        self.F_d_z = 0.0
        # Force difference
        
        self.position_diff_limit = 0.001
        self.publish_rate = 100
        # --------------------------
        self.cmd_vel = Twist()
        self.velocity_threshhold = 0.1
        self.cmd_vel_filtered = Twist()
        self.wrench_filter = 0.03
        self.cmd_vel_filter = 0.05

    
if __name__ == '__main__':
    ur_admittance_controller()