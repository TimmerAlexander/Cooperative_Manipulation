#!/usr/bin/env python3

import rospy    
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import moveit_commander
import sys



class cooperative_movement():
    
    def config(self):
        self.publish_rate = 100 # [HZ] 
        self.now = 0.0
        self.path_sum = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.pub_trajectory = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.joint_velocity_msg = Twist()
        self.trajectory_msg = Float64MultiArray()
        
    def __init__(self):
        self.config()
        
        # * Initialize move_it
        moveit_commander.roscpp_initialize(sys.argv)
            
        # * Initialize node
        rospy.init_node('ur16e_singularity_test', anonymous=True)
        
        self.pub_cartesian_velocity_command = rospy.Publisher("/cooperative_manipulation/cartesian_velocity_command", Twist, queue_size=1)
        
        self.pub_world_trajectory = rospy.Publisher("/cooperative_manipulation/world_trajectory", Float64MultiArray, queue_size=1)
        
        rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo("Cooperative movement starts now!")
    
        self.stage_0(rate,1.0)
        self.stage_1(rate,9.0)
        self.stage_2(rate,9.0)
        # self.stage_3(rate,5.0)
        # self.stage_4(rate,5.0)
        

        
        
        # self.stage_5(rate,4.0)
        # self.stage_6(rate,4.0)
        
        
# ToDo: Use this template to add a stage!-------------------------------------------------------------------------------
# Template for stage.
        #self.stage_7(rate,8.0)
# ToDo:-----------------------------------------------------------------------------------------------------------------

# ! Do not changes this!------------------------------------------------------------------------------------------------
# This stage moves the robot back to the initposition.
        # self.stage_back_to_init(rate,10.0)
# Last stage which stops the robot. 
        self.stage_end(rate,1.0)
# ! --------------------------------------------------------------------------------------------------------------------
    
    def stage_0(self,rate,duration):
        """Send velocity command null

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        rospy.loginfo("Stage 0")
        rospy.loginfo("Robots stop. Wait until program starts! Duration: %f [s]",duration)
        
        self.joint_velocity_msg.linear.x = 0.0
        self.joint_velocity_msg.linear.y = 0.0
        self.joint_velocity_msg.linear.z = 0.0
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        
        joint_velocity_array = np.array([self.joint_velocity_msg.linear.x,
                                        self.joint_velocity_msg.linear.y,
                                        self.joint_velocity_msg.linear.z,
                                        self.joint_velocity_msg.angular.x,
                                        self.joint_velocity_msg.angular.y,
                                        self.joint_velocity_msg.angular.z])
                
        self.compute_path(self.joint_velocity_msg,duration)
        
        start_time = rospy.get_rostime().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = rospy.get_rostime().to_sec()
            self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
            
            # Publish the trajectory in 'world' frame
            self.pub_trajectory += joint_velocity_array/self.publish_rate
            self.trajectory_msg.data = self.pub_trajectory
            self.pub_world_trajectory.publish(self.trajectory_msg)
            
            rate.sleep()
            
            
    def stage_1(self,rate,duration):
        """Send velocity command linear.x = 0.05

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        rospy.loginfo("Stage 1")
        rospy.loginfo("linear.x = 0.05 [m/s], duration: %f [s]",duration)
        
        self.joint_velocity_msg.linear.x = 0.05
        self.joint_velocity_msg.linear.y = 0.0
        self.joint_velocity_msg.linear.z = 0.0
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        
        
        joint_velocity_array = np.array([self.joint_velocity_msg.linear.x,
                                        self.joint_velocity_msg.linear.y,
                                        self.joint_velocity_msg.linear.z,
                                        self.joint_velocity_msg.angular.x,
                                        self.joint_velocity_msg.angular.y,
                                        self.joint_velocity_msg.angular.z])
                
        self.compute_path(self.joint_velocity_msg,duration)
        
        start_time = rospy.get_rostime().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = rospy.get_rostime().to_sec()
            self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
            
            # Publish the trajectory in 'world' frame
            self.pub_trajectory += joint_velocity_array/self.publish_rate
            self.trajectory_msg.data = self.pub_trajectory
            self.pub_world_trajectory.publish(self.trajectory_msg)
            
            rate.sleep()
            

            
    def stage_2(self,rate,duration):
        """Send velocity command linear.x = -0.05

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        rospy.loginfo("Stage 2")
        rospy.loginfo("linear.x = -0.05 [m/s], duration: %f [s]",duration)
        
        self.joint_velocity_msg.linear.x = -0.05
        self.joint_velocity_msg.linear.y = 0.0
        self.joint_velocity_msg.linear.z = 0.0
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        
        
        joint_velocity_array = np.array([self.joint_velocity_msg.linear.x,
                                        self.joint_velocity_msg.linear.y,
                                        self.joint_velocity_msg.linear.z,
                                        self.joint_velocity_msg.angular.x,
                                        self.joint_velocity_msg.angular.y,
                                        self.joint_velocity_msg.angular.z])
                
        self.compute_path(self.joint_velocity_msg,duration)
        
        start_time = rospy.get_rostime().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = rospy.get_rostime().to_sec()
            self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
            
            # Publish the trajectory in 'world' frame
            self.pub_trajectory += joint_velocity_array/self.publish_rate
            self.trajectory_msg.data = self.pub_trajectory
            self.pub_world_trajectory.publish(self.trajectory_msg)
            
            rate.sleep()
            
        
    def stage_3(self,rate,duration):
        """Send velocity command linear.z =  0.05

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        rospy.loginfo("Stage 3")
        rospy.loginfo("linear.y = 0.05 [m/s], duration: %f [s]",duration)
        
        self.joint_velocity_msg.linear.x = 0.0
        self.joint_velocity_msg.linear.y = 0.09
        self.joint_velocity_msg.linear.z = 0.0
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        
        joint_velocity_array = np.array([self.joint_velocity_msg.linear.x,
                                        self.joint_velocity_msg.linear.y,
                                        self.joint_velocity_msg.linear.z,
                                        self.joint_velocity_msg.angular.x,
                                        self.joint_velocity_msg.angular.y,
                                        self.joint_velocity_msg.angular.z])
                
        self.compute_path(self.joint_velocity_msg,duration)
        
        start_time = rospy.get_rostime().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = rospy.get_rostime().to_sec()
            self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
            
            # Publish the trajectory in 'world' frame
            self.pub_trajectory += joint_velocity_array/self.publish_rate
            self.trajectory_msg.data = self.pub_trajectory
            self.pub_world_trajectory.publish(self.trajectory_msg)
            
            rate.sleep()
            
    def stage_4(self,rate,duration):
        """Send velocity command linear.y = -0.05

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        rospy.loginfo("Stage 4")
        rospy.loginfo("linear.y = -0.05 [m/s], duration: %f [s]",duration)
        
        self.joint_velocity_msg.linear.x = 0.0
        self.joint_velocity_msg.linear.y = -0.09
        self.joint_velocity_msg.linear.z = 0.0
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        
        joint_velocity_array = np.array([self.joint_velocity_msg.linear.x,
                                        self.joint_velocity_msg.linear.y,
                                        self.joint_velocity_msg.linear.z,
                                        self.joint_velocity_msg.angular.x,
                                        self.joint_velocity_msg.angular.y,
                                        self.joint_velocity_msg.angular.z])
                
        self.compute_path(self.joint_velocity_msg,duration)
        
        start_time = rospy.get_rostime().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = rospy.get_rostime().to_sec()
            self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
            
            # Publish the trajectory in 'world' frame
            self.pub_trajectory += joint_velocity_array/self.publish_rate
            self.trajectory_msg.data = self.pub_trajectory
            self.pub_world_trajectory.publish(self.trajectory_msg)
            
            rate.sleep()
            
    def stage_5(self,rate,duration):
        """Send velocity command 
                                

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        rospy.loginfo("Stage 5")
        rospy.loginfo("angular.z = 0.02 [m/s], duration: %f [s]",duration)
        
        self.joint_velocity_msg.linear.x = 0.0
        self.joint_velocity_msg.linear.y = 0.0
        self.joint_velocity_msg.linear.z = 0.05
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        

        joint_velocity_array = np.array([self.joint_velocity_msg.linear.x,
                                        self.joint_velocity_msg.linear.y,
                                        self.joint_velocity_msg.linear.z,
                                        self.joint_velocity_msg.angular.x,
                                        self.joint_velocity_msg.angular.y,
                                        self.joint_velocity_msg.angular.z])
                
        self.compute_path(self.joint_velocity_msg,duration)
        
        start_time = rospy.get_rostime().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = rospy.get_rostime().to_sec()
            self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
            
            # Publish the trajectory in 'world' frame
            self.pub_trajectory += joint_velocity_array/self.publish_rate
            self.trajectory_msg.data = self.pub_trajectory
            self.pub_world_trajectory.publish(self.trajectory_msg)
            
            rate.sleep()
            
    def stage_6(self,rate,duration):
        """Send velocity command 
                                

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        rospy.loginfo("Stage 6")
        rospy.loginfo("angular.z = -0.02 [m/s], duration: %f [s]",duration)
        
        self.joint_velocity_msg.linear.x = 0.0
        self.joint_velocity_msg.linear.y = 0.0
        self.joint_velocity_msg.linear.z = -0.05
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        

        joint_velocity_array = np.array([self.joint_velocity_msg.linear.x,
                                        self.joint_velocity_msg.linear.y,
                                        self.joint_velocity_msg.linear.z,
                                        self.joint_velocity_msg.angular.x,
                                        self.joint_velocity_msg.angular.y,
                                        self.joint_velocity_msg.angular.z])
                
        self.compute_path(self.joint_velocity_msg,duration)
        
        start_time = rospy.get_rostime().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = rospy.get_rostime().to_sec()
            self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
            
            # Publish the trajectory in 'world' frame
            self.pub_trajectory += joint_velocity_array/self.publish_rate
            self.trajectory_msg.data = self.pub_trajectory
            self.pub_world_trajectory.publish(self.trajectory_msg)
            
            rate.sleep()
            
# ToDo: Use this template to add a stage!-------------------------------------------------------------------------------
# Template for stage.

        def stage_7(self,rate,duration):
            """Send velocity command 
                                    

            Args:
                rate (Rate): Publish rate
                duration (float): Publishing time
            """
            rospy.loginfo("Stage 7")
            rospy.loginfo("Write velocity command to log... e.g. linear.x = 0.05 [m/s], duration: %f [s]",duration)
            
            self.joint_velocity_msg.linear.x = 0.0
            self.joint_velocity_msg.linear.y = 0.0
            self.joint_velocity_msg.linear.z = 0.0
            self.joint_velocity_msg.angular.x = 0.0
            self.joint_velocity_msg.angular.y = 0.0
            self.joint_velocity_msg.angular.z = 0.0
            

            joint_velocity_array = np.array([self.joint_velocity_msg.linear.x,
                                        self.joint_velocity_msg.linear.y,
                                        self.joint_velocity_msg.linear.z,
                                        self.joint_velocity_msg.angular.x,
                                        self.joint_velocity_msg.angular.y,
                                        self.joint_velocity_msg.angular.z])
                
        self.compute_path(self.joint_velocity_msg,duration)
        
        start_time = rospy.get_rostime().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = rospy.get_rostime().to_sec()
            self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
            
            # Publish the trajectory in 'world' frame
            self.pub_trajectory += joint_velocity_array/self.publish_rate
            self.trajectory_msg.data = self.pub_trajectory
            self.pub_world_trajectory.publish(self.trajectory_msg)
            
            rate.sleep()
# ToDo:-----------------------------------------------------------------------------------------------------------------
    

# ! Do not changes this!------------------------------------------------------------------------------------------------

    def publish_trajectory(self,rate):
        """
        
        
        
        """
        pub_trajectory = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        
        # * Publish the target_joint_velocity
        while np.linalg.norm(pub_trajectory) <= np.linalg.norm(self.path_sum):
            
            pub_trajectory += self.path_sum/self.publish_rate
            self.trajectory_msg.data = pub_trajectory
            self.pub_world_trajectory.publish(self.trajectory_msg)
            rate.sleep()
        
            
        
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
        rospy.loginfo("The robots moves to start position. Duration: %f [s]",duration)
        
        self.joint_velocity_msg.linear.x = -self.path_sum[0]/duration
        self.joint_velocity_msg.linear.y = -self.path_sum[1]/duration
        self.joint_velocity_msg.linear.z = -self.path_sum[2]/duration
        self.joint_velocity_msg.angular.x = -self.path_sum[3]/duration
        self.joint_velocity_msg.angular.y = -self.path_sum[4]/duration
        self.joint_velocity_msg.angular.z = -self.path_sum[5]/duration
        
        joint_velocity_array = np.array([self.joint_velocity_msg.linear.x,
                                        self.joint_velocity_msg.linear.y,
                                        self.joint_velocity_msg.linear.z,
                                        self.joint_velocity_msg.angular.x,
                                        self.joint_velocity_msg.angular.y,
                                        self.joint_velocity_msg.angular.z])
                
        self.compute_path(self.joint_velocity_msg,duration)
        
        start_time = rospy.get_rostime().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = rospy.get_rostime().to_sec()
            self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
            
            # Publish the trajectory in 'world' frame
            self.pub_trajectory += joint_velocity_array/self.publish_rate
            self.trajectory_msg.data = self.pub_trajectory
            self.pub_world_trajectory.publish(self.trajectory_msg)
            
            rate.sleep()

# Last stage which stops the robot.
    def stage_end(self,rate,duration):
        """Send velocity command null

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        rospy.loginfo("Robots stop. Wait until program finished! Duration: %f [s]",duration)
                
        self.joint_velocity_msg.linear.x = 0.0
        self.joint_velocity_msg.linear.y = 0.0
        self.joint_velocity_msg.linear.z = 0.0
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        
        joint_velocity_array = np.array([self.joint_velocity_msg.linear.x,
                                        self.joint_velocity_msg.linear.y,
                                        self.joint_velocity_msg.linear.z,
                                        self.joint_velocity_msg.angular.x,
                                        self.joint_velocity_msg.angular.y,
                                        self.joint_velocity_msg.angular.z])
                
        self.compute_path(self.joint_velocity_msg,duration)
        
        start_time = rospy.get_rostime().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = rospy.get_rostime().to_sec()
            self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
            
            # Publish the trajectory in 'world' frame
            self.pub_trajectory += joint_velocity_array/self.publish_rate
            self.trajectory_msg.data = self.pub_trajectory
            self.pub_world_trajectory.publish(self.trajectory_msg)
            
            rate.sleep()
# ! --------------------------------------------------------------------------------------------------------------------


if __name__ == '__main__':
    cooperative_movement()
