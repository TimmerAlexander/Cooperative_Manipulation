#!/usr/bin/env python3

import rospy    
import numpy as np
from geometry_msgs.msg import Twist
import moveit_commander
import sys



class cooperative_movement():
    
    def config(self):
        self.radius = 0.1
        self.alpha = 0.0
        self.velocity_vector = np.array([0.0,0.0,1.0])
        self.cartesian_rotation_velocity = np.array([0.0,0.0,0.0])
        self.angular_velocity = 0.01 # [HZ] 
        self.publish_rate = 100 # [HZ] 
        self.now = 0.0
       
    def __init__(self):
        self.config()
        
        self.joint_velocity_msg = Twist()
        
        # * Initialize move_it
        moveit_commander.roscpp_initialize(sys.argv)
            
        # * Initialize node
        rospy.init_node('ur16e_singularity_test', anonymous=True)
        
        self.pub_joint_velocity = rospy.Publisher("/cooperative_manipulation/cartesian_velocity_command", Twist, queue_size=1)

        rospy.loginfo("Start ur16e_singularity_test ...")
     
        rate = rospy.Rate(self.publish_rate)
     
        self.stage_1(rate,6.0)
        self.stage_2(rate,12.0)
        self.stage_3(rate,8.0)
        self.stage_4(rate,8.0)
        self.stage_end(rate)
        
     
    def stage_1(self,rate,duration):
        """Send velocity command linear.x = 0.1

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        self.joint_velocity_msg.linear.x = 0.1
        self.joint_velocity_msg.linear.y = -0.2
        self.joint_velocity_msg.linear.z = 0.0
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        start_time = rospy.Time().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            # print(self.now - start_time)
            self.now = self.now + (1/self.publish_rate)
            self.pub_joint_velocity.publish(self.joint_velocity_msg)
            rate.sleep()
            
            
    def stage_2(self,rate,duration):
        """Send velocity command linear.x = -0.2
                                 linear.y = 0.05 

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        self.joint_velocity_msg.linear.x = -0.2
        self.joint_velocity_msg.linear.y = 0.05
        self.joint_velocity_msg.linear.z = 0.0
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        

        start_time = rospy.Time().to_sec()
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            # print(self.now - start_time)
            self.now = self.now + (1/self.publish_rate)
            self.pub_joint_velocity.publish(self.joint_velocity_msg)
            rate.sleep()
            
    def stage_3(self,rate,duration):
        """Send velocity command linear.z = -0.01

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        self.joint_velocity_msg.linear.x = 0.0
        self.joint_velocity_msg.linear.y = 0.0
        self.joint_velocity_msg.linear.z = -0.01
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        

        start_time = rospy.Time().to_sec()
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            # print(self.now - start_time)
            self.now = self.now + (1/self.publish_rate)
            self.pub_joint_velocity.publish(self.joint_velocity_msg)
            rate.sleep()
            
    def stage_4(self,rate,duration):
        """Send velocity command linear.y = -0.01
                                 linear.z = 0.01

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        self.joint_velocity_msg.linear.x = 0.0
        self.joint_velocity_msg.linear.y = -0.01
        self.joint_velocity_msg.linear.z = 0.01
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        

        start_time = rospy.Time().to_sec()
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            # print(self.now - start_time)
            self.now = self.now + (1/self.publish_rate)
            self.pub_joint_velocity.publish(self.joint_velocity_msg)
            rate.sleep()
    
    
            
    def stage_end(self,rate):
        """Send velocity command null

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        self.joint_velocity_msg.linear.x = 0.0
        self.joint_velocity_msg.linear.y = 0.0
        self.joint_velocity_msg.linear.z = 0.0
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        
    
        # * Publish the target_joint_velocity
        while not rospy.is_shutdown():
            self.pub_joint_velocity.publish(self.joint_velocity_msg)
            rate.sleep()
    
if __name__ == '__main__':
    cooperative_movement()