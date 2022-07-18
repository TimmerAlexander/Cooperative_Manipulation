#!/usr/bin/env python3

import rospy    
import numpy as np
from geometry_msgs.msg import Twist
import moveit_commander
import sys



class cooperative_movement():
    
    def config(self):
        self.publish_rate = 100 # [HZ] 
        self.now = 0.0
        self.path_sum = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        
    def __init__(self):
        self.config()
        
        self.joint_velocity_msg = Twist()
        
        # * Initialize move_it
        moveit_commander.roscpp_initialize(sys.argv)
            
        # * Initialize node
        rospy.init_node('ur16e_singularity_test', anonymous=True)
        
        self.pub_joint_velocity = rospy.Publisher("/cooperative_manipulation/cartesian_velocity_command", Twist, queue_size=1)
        
        rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo("Cooperative movement starts now!")
        
        self.stage_1(rate,4.0)
        self.stage_2(rate,6.0)
        self.stage_3(rate,8.0)
        self.stage_4(rate,2.0)
        
        
# ToDo: Use this template to add a stage!-------------------------------------------------------------------------------
# Template for stage.
        #self.stage_5(rate,8.0)
# ToDo:-----------------------------------------------------------------------------------------------------------------

# ! Do not changes this!------------------------------------------------------------------------------------------------
# This stage moves the robot back to the initposition.
        self.stage_back_to_init(rate,10.0)
# Last stage which stops the robot. 
        self.stage_end(rate,5.0)
# ! --------------------------------------------------------------------------------------------------------------------
        
    def stage_1(self,rate,duration):
        """Send velocity command linear.x = 0.1

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        self.joint_velocity_msg.linear.x = 0.05
        self.joint_velocity_msg.linear.y = 0.0
        self.joint_velocity_msg.linear.z = 0.0
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        
        self.compute_path(self.joint_velocity_msg,duration)
        
        start_time = rospy.Time().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
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
        self.joint_velocity_msg.linear.x = -0.02
        self.joint_velocity_msg.linear.y = 0.03
        self.joint_velocity_msg.linear.z = 0.0
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        
        self.compute_path(self.joint_velocity_msg,duration)

        start_time = rospy.Time().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
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
        
        self.compute_path(self.joint_velocity_msg,duration)

        start_time = rospy.Time().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
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
        self.joint_velocity_msg.linear.x = -0.01
        self.joint_velocity_msg.linear.y = 0.0
        self.joint_velocity_msg.linear.z = 0.0
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        
        self.compute_path(self.joint_velocity_msg,duration)

        start_time = rospy.Time().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = self.now + (1/self.publish_rate)
            self.pub_joint_velocity.publish(self.joint_velocity_msg)
            rate.sleep()
            
# ToDo: Use this template to add a stage!-------------------------------------------------------------------------------
# Template for stage.

        def stage_5(self,rate,duration):
            """Send velocity command 
                                    

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
            

            start_time = rospy.Time().to_sec()
            self.now = start_time 
            # * Publish the target_joint_velocity
            while(self.now - start_time <=  duration):
                self.now = self.now + (1/self.publish_rate)
                self.pub_joint_velocity.publish(self.joint_velocity_msg)
                rate.sleep()
# ToDo:-----------------------------------------------------------------------------------------------------------------
    

# ! Do not changes this!------------------------------------------------------------------------------------------------


    def compute_path(self,velocity_cmd,duration):
        """Compute the path the robot does.

        Args:
            velocity_cmd (_type_): _Current velocity command
            duration (float): Current publishing time
        """
        self.path_sum[0] += velocity_cmd.linear.x * duration
        self.path_sum[1] += velocity_cmd.linear.y * duration
        self.path_sum[2] += velocity_cmd.linear.z * duration
        self.path_sum[3] += velocity_cmd.angular.x * duration
        self.path_sum[4] += velocity_cmd.angular.y * duration
        self.path_sum[5] += velocity_cmd.angular.z * duration

# This stage moves the robot back to the initposition.
    def stage_back_to_init(self,rate,duration):
        """Send robot back to initposition.

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        self.joint_velocity_msg.linear.x = -self.path_sum[0]/duration
        self.joint_velocity_msg.linear.y = -self.path_sum[1]/duration
        self.joint_velocity_msg.linear.z = -self.path_sum[2]/duration
        self.joint_velocity_msg.angular.x = -self.path_sum[3]/duration
        self.joint_velocity_msg.angular.y = -self.path_sum[4]/duration
        self.joint_velocity_msg.angular.z = -self.path_sum[5]/duration
        
        start_time = rospy.Time().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = self.now + (1/self.publish_rate)
            self.pub_joint_velocity.publish(self.joint_velocity_msg)
            rate.sleep()


# Last stage which stops the robot.
    def stage_end(self,rate,duration):
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
        
        start_time = rospy.Time().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = self.now + (1/self.publish_rate)
            self.pub_joint_velocity.publish(self.joint_velocity_msg)
            rate.sleep()
# ! --------------------------------------------------------------------------------------------------------------------


if __name__ == '__main__':
    cooperative_movement()