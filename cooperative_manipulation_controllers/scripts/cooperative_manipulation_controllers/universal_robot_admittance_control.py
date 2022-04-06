#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import WrenchStamped, Twist
import tf

import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy

#J_ur=self.group.get_jacobian_matrix(joint_states_array)

class ur_admittance_controller():
    
    
    def __init__(self):
        self.config()
        rospy.init_node('admittance_controller_node', anonymous=True)
        rospy.loginfo("admittance controller running")
        
        self.namespace = rospy.get_param("~ur_ns")
        
        #-------------------------------------------
        # Initialize move_it
        self.joint_vel = Float64MultiArray()
        group_name = 'manipulator'
        print("Initialize movit_commander. Group name: ",group_name)
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.pub = rospy.Publisher("/" + self.namespace + "/ur_admittance_controller/command", Float64MultiArray, queue_size=1)
        #-------------------------------------------
        
        #self.pub = rospy.Publisher("/" + self.namespace + "/ur_admittance_controller/command", Twist, queue_size = 1)
        self.listener = tf.TransformListener()
        now = rospy.Time()
        self.listener.waitForTransform("base_link", "tool0", rospy.Time(), rospy.Duration(4.0))
        (self.initial_position,self.initial_orientation) = self.listener.lookupTransform('base_link', 'tool0', now)
        print(self.initial_position)
        rospy.Subscriber("/" + self.namespace + "/wrench", WrenchStamped, self.wrench_cb)
        
    
        rospy.spin()
        

    def wrench_cb(self,wrench):
        print(wrench)

        now = rospy.Time()
        (curr_position,curr_orientation) = self.listener.lookupTransform('base_link', 'tool0', now)

        position_diff_x = self.initial_position[0] - curr_position[0]
        position_diff_y = self.initial_position[1] - curr_position[1]
        position_diff_z = self.initial_position[2] - curr_position[2]

        self.x_avg = self.x_avg*(1-self.wrench_filter) + wrench.wrench.force.x * self.wrench_filter 
        self.y_avg = self.y_avg*(1-self.wrench_filter) + wrench.wrench.force.y * self.wrench_filter 
        self.z_avg = self.z_avg*(1-self.wrench_filter) + wrench.wrench.force.z * self.wrench_filter 


        #print(position_diff_x,position_diff_y)
        self.cmd_vel.linear.x = self.x_avg * self.KPx - position_diff_x * self.Cx
        self.cmd_vel.linear.y = self.y_avg * self.KPy - position_diff_y * self.Cy
        self.cmd_vel.linear.z = self.z_avg * self.KPz + position_diff_z * self.Cz

        self.cmd_vel_filtered.linear.x = self.cmd_vel_filtered.linear.x * (1-self.cmd_vel_filter) + self.cmd_vel.linear.x * self.cmd_vel_filter
        self.cmd_vel_filtered.linear.y = self.cmd_vel_filtered.linear.y * (1-self.cmd_vel_filter) + self.cmd_vel.linear.y * self.cmd_vel_filter
        self.cmd_vel_filtered.linear.z = self.cmd_vel_filtered.linear.z * (1-self.cmd_vel_filter) + self.cmd_vel.linear.z * self.cmd_vel_filter

        # limit velocity
        if abs(self.cmd_vel_filtered.linear.x)> self.velocity_threshhold:
            self.cmd_vel_filtered.linear.x = self.cmd_vel_filtered.linear.x/abs(self.cmd_vel_filtered.linear.x) * self.velocity_threshhold

        if abs(self.cmd_vel_filtered.linear.y)> self.velocity_threshhold:
            self.cmd_vel_filtered.linear.y = self.cmd_vel_filtered.linear.y/abs(self.cmd_vel_filtered.linear.y) * self.velocity_threshhold

        if abs(self.cmd_vel_filtered.linear.z)> self.velocity_threshhold:
            self.cmd_vel_filtered.linear.z = self.cmd_vel_filtered.linear.z/abs(self.cmd_vel_filtered.linear.z) * self.velocity_threshhold

        #print("Publish Twist:",self.cmd_vel.linear.x,self.cmd_vel.linear.y)
        #print(position_diff_z)
#---------------------------------------------------------------------
        #print("cmd_vel: ")
        #print(self.cmd_vel.linear.x)
        #print(self.cmd_vel.linear.y)
        #print(self.cmd_vel.linear.z)
        #print(self.cmd_vel.angular.x)
        #print(self.cmd_vel.angular.y)
        #print(self.cmd_vel.angular.z)
        
        
        self.target_vel = numpy.array([self.cmd_vel.linear.x,self.cmd_vel.linear.y,self.cmd_vel.linear.z,self.cmd_vel.angular.x,self.cmd_vel.angular.y,self.cmd_vel.angular.z])
        
        #print("Publish self.target_vel: ")
        #print(self.target_vel)
        
        

        joint_states_array = self.group.get_current_joint_values()
        self.Jacobian_ur=self.group.get_jacobian_matrix(joint_states_array)
        
        #print("Publish Jacobian_ur: ",self.Jacobian_ur)
        #print(self.Jacobian_ur)
        
        inverse = numpy.linalg.inv(self.Jacobian_ur)
        self.target_dq = inverse.dot(self.target_vel)
        self.joint_vel.data = self.target_dq
        
        
        print("self.joint_vel.data: ",self.joint_vel.data)
        self.pub.publish(self.joint_vel)
#--------------------------------------------------------------------
        #print("Publish Twist: ")
        #print(self.cmd_vel)
        #self.pub.publish(self.cmd_vel)
    

    def config(self):
        self.cmd_vel = Twist()
        self.velocity_threshhold = 0.1
        self.KPx = 0.007
        self.KPy = 0.007
        self.KPz = 0.007
        self.Cx = 1.0
        self.Cy = 1.0
        self.Cz = 1.0
        self.x_avg = 0.0
        self.y_avg = 0.0
        self.z_avg = 0.0
        self.cmd_vel_filtered = Twist()
        self.wrench_filter = 0.03
        self.cmd_vel_filter = 0.05

    
if __name__ == '__main__':
    ur_admittance_controller()