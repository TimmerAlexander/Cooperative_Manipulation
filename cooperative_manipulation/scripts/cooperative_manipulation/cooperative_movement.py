#!/usr/bin/env python3

import rosbag
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
        self.joint_acc_msg = Float64MultiArray()
        self.acc_vel = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.current_vel = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.acceleration_duration = 25 #[Hz] = 0.02 [s]
        self.vel_cmd_msg = Float64MultiArray()

        
    def __init__(self):
        self.config()
        
        # * Initialize move_it
        moveit_commander.roscpp_initialize(sys.argv)
            
        # * Initialize node
        rospy.init_node('ur16e_singularity_test', anonymous=True)
        
        self.pub_cartesian_velocity_command = rospy.Publisher("/cooperative_manipulation/cartesian_velocity_command", Twist, queue_size=1)
        
        self.pub_world_trajectory = rospy.Publisher("/cooperative_manipulation/world_trajectory", Float64MultiArray, queue_size=1)
        
        
        self.cmd_vel_publihser = rospy.Publisher('/measurement/cmd_vel',Float64MultiArray,tcp_nodelay=True,queue_size=1)
        
        rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo("Cooperative movement starts now!")
        
        rospy.loginfo("Wait until program starts!")
        rospy.sleep(0.5)
        
        self.stage_init(rate,1.0)
        
        self.velocity_command(1,rate,5.0,0.0,0.0,0.0,0.0,0.05,0.0)



# ! Do not changes this!------------------------------------------------------------------------------------------------
# This stage moves the robot back to the initposition.
        # self.stage_back_to_init(rate,10.0)
# Last stage which stops the robot. 
        self.stage_end(rate,1.0)
        
       
# ! --------------------------------------------------------------------------------------------------------------------
    def velocity_command(self,stage: int,rate: float,duration: float,x_lin: float,y_lin: float,z_lin: float,x_rot: float,y_rot: float,z_rot: float):
        """ Set a velocity command for a specific duration.

        Args:
            stage (int): Number of stage.
            rate (float): Publish time.
            duration (float): Publish duration. 
            x_lin (float): Linear velocity in x[m/s].
            y_lin (float): Linear velocity in y[m/s].
            z_lin (float): Linear velocity in z[m/s].
            x_rot (float): Angular velocity around x[rad/s].
            y_rot (float): Angular velocity around y[rad/s].
            z_rot (float): Angular velocity around z[rad/s].
        """
        rospy.loginfo("Stage command: %f",stage)
        rospy.loginfo("x_lin %f[m/s],y_lin %f[m/s],z_lin %f[m/s],x_rot %f[rad/s],y_rot %f[rad/s],z_rot %f[rad/s]",x_lin,y_lin,z_lin,x_rot,y_rot,z_rot)
        rospy.loginfo("Duration: %f [s]",duration)
        
        self.joint_velocity_msg.linear.x = x_lin
        self.joint_velocity_msg.linear.y = y_lin
        self.joint_velocity_msg.linear.z = z_lin
        self.joint_velocity_msg.angular.x = x_rot
        self.joint_velocity_msg.angular.y = y_rot
        self.joint_velocity_msg.angular.z = z_rot
        
        
        
        joint_velocity_array = np.array([self.joint_velocity_msg.linear.x,
                                        self.joint_velocity_msg.linear.y,
                                        self.joint_velocity_msg.linear.z,
                                        self.joint_velocity_msg.angular.x,
                                        self.joint_velocity_msg.angular.y,
                                        self.joint_velocity_msg.angular.z])
        
        self.vel_cmd_msg.data = joint_velocity_array
        self.cmd_vel_publihser.publish(self.vel_cmd_msg)
        
        self.compute_path(self.joint_velocity_msg,duration)
        
        acc_vel_diff = joint_velocity_array - self.current_vel
        
        start_time = rospy.get_rostime().to_sec()
        self.now = start_time 
        print(start_time )        
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = rospy.get_rostime().to_sec()
            
            # Acceleration to desired velocity
            if np.any((np.round(self.acc_vel,3))!= (np.round(joint_velocity_array,3))):
                for acc in range(len(joint_velocity_array)):
                    # Acceleration
                    if acc_vel_diff[acc] > 0.0:
                    
                        if self.acc_vel[acc] != joint_velocity_array[acc]:
                            self.acc_vel[acc] += acc_vel_diff[acc]/self.acceleration_duration
                        
                        if self.acc_vel[acc] >= joint_velocity_array[acc]:
                            self.acc_vel[acc] = joint_velocity_array[acc]
                    # Deceleration
                    if acc_vel_diff[acc] < 0.0:
                        
                        if self.acc_vel[acc] != joint_velocity_array[acc]:
                            self.acc_vel[acc] += acc_vel_diff[acc]/self.acceleration_duration
                        
                        if self.acc_vel[acc] <= joint_velocity_array[acc]:
                            self.acc_vel[acc] = joint_velocity_array[acc]
                        
                self.joint_velocity_msg.linear.x = np.round(self.acc_vel[0],3)
                self.joint_velocity_msg.linear.y = np.round(self.acc_vel[1],3)
                self.joint_velocity_msg.linear.z = np.round(self.acc_vel[2],3)
                self.joint_velocity_msg.angular.x = np.round(self.acc_vel[3],3)
                self.joint_velocity_msg.angular.y = np.round(self.acc_vel[4],3)
                self.joint_velocity_msg.angular.z = np.round(self.acc_vel[5],3)
                
                self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
                # Publish the trajectory in 'world' frame
                self.pub_trajectory += self.acc_vel/self.publish_rate
                self.trajectory_msg.data = self.pub_trajectory
                self.pub_world_trajectory.publish(self.trajectory_msg)
                
                self.current_vel = self.acc_vel
            
            else:
                self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
            
                # Publish the trajectory in 'world' frame
                self.pub_trajectory += joint_velocity_array/self.publish_rate
                self.trajectory_msg.data = self.pub_trajectory
                self.pub_world_trajectory.publish(self.trajectory_msg)
                
                self.current_vel = joint_velocity_array
            
            rate.sleep()
        
    def stage_init(self,rate,duration):
        """Send velocity command null

        Args:
            rate (Rate): Publish rate
            duration (float): Publishing time
        """
        rospy.loginfo("Stage init")
        rospy.loginfo("Init movement! Duration: %f [s]",duration)
        
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
        
        self.vel_cmd_msg.data = joint_velocity_array
        self.cmd_vel_publihser.publish(self.vel_cmd_msg)
        
        self.compute_path(self.joint_velocity_msg,duration)
        
        start_time = rospy.get_rostime().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = rospy.get_rostime().to_sec()
            self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
            
            rate.sleep()
        

# ! Do not changes this!------------------------------------------------------------------------------------------------
    def compute_path(self,velocity_cmd,duration):
        """Compute the path the robot does.

        Args:
            velocity_cmd (_type_): Current velocity command
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
         

        start_time = rospy.get_rostime().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = rospy.get_rostime().to_sec()
            self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
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
        
        self.vel_cmd_msg.data = joint_velocity_array
        self.cmd_vel_publihser.publish(self.vel_cmd_msg)
        
        self.compute_path(self.joint_velocity_msg,duration)
        
        
        acc_vel_diff = joint_velocity_array - self.current_vel
        
        start_time = rospy.get_rostime().to_sec()
        self.now = start_time 
        # * Publish the target_joint_velocity
        while(self.now - start_time <=  duration):
            self.now = rospy.get_rostime().to_sec()
            
            # Acceleration to desired velocity
            if np.any((np.round(self.acc_vel,3))!= (np.round(joint_velocity_array,3))):
                for acc in range(len(joint_velocity_array)):
                    # Acceleration
                    if acc_vel_diff[acc] > 0.0:
                    
                        if self.acc_vel[acc] != joint_velocity_array[acc]:
                            self.acc_vel[acc] += acc_vel_diff[acc]/self.acceleration_duration
                        
                        if self.acc_vel[acc] >= joint_velocity_array[acc]:
                            self.acc_vel[acc] = joint_velocity_array[acc]
                    # Deceleration
                    if acc_vel_diff[acc] < 0.0:
                        
                        if self.acc_vel[acc] != joint_velocity_array[acc]:
                            self.acc_vel[acc] += acc_vel_diff[acc]/self.acceleration_duration
                        
                        if self.acc_vel[acc] <= joint_velocity_array[acc]:
                            self.acc_vel[acc] = joint_velocity_array[acc]
                        
                self.joint_velocity_msg.linear.x = np.round(self.acc_vel[0],3)
                self.joint_velocity_msg.linear.y = np.round(self.acc_vel[1],3)
                self.joint_velocity_msg.linear.z = np.round(self.acc_vel[2],3)
                self.joint_velocity_msg.angular.x = np.round(self.acc_vel[3],3)
                self.joint_velocity_msg.angular.y = np.round(self.acc_vel[4],3)
                self.joint_velocity_msg.angular.z = np.round(self.acc_vel[5],3)
                
                self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
                # Publish the trajectory in 'world' frame
                self.pub_trajectory += self.acc_vel/self.publish_rate
                self.trajectory_msg.data = self.pub_trajectory
                self.pub_world_trajectory.publish(self.trajectory_msg)
                
                self.current_vel = self.acc_vel
            
            else:
                self.pub_cartesian_velocity_command.publish(self.joint_velocity_msg)
            
                # Publish the trajectory in 'world' frame
                self.pub_trajectory += joint_velocity_array/self.publish_rate
                self.trajectory_msg.data = self.pub_trajectory
                self.pub_world_trajectory.publish(self.trajectory_msg)
                
                self.current_vel = joint_velocity_array
            
            
            rate.sleep()
        
# ! --------------------------------------------------------------------------------------------------------------------


if __name__ == '__main__':
    cooperative_movement()
