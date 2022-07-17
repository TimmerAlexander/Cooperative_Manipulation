#!/usr/bin/env python3

# /***************************************************************************

# **************************************************************************/


import rospy    
import numpy as np
from geometry_msgs.msg import Twist
import moveit_commander
import sys



class ur16e_singularity_test():
    
    def config(self):
        self.radius = 1.0
        self.alpha = 0.0
        self.angular_velocity_vector = np.array([0.0,0.0,1.57]) # [HZ]
        self.cartesian_rotation_velocity = np.array([0.0,0.0,0.0])
        self.publish_rate = 100 # [HZ] 
       
    def __init__(self):
        self.config()
        
        self.joint_velocity_msg = Twist()
        
        # * Initialize move_it
        moveit_commander.roscpp_initialize(sys.argv)
            
        # * Initialize node
        rospy.init_node('ur16e_singularity_test', anonymous=True)
        
        self.pub_joint_velocity = rospy.Publisher("/cooperative_manipulation/cartesian_velocity_command", Twist, queue_size=100)
        
        rate = rospy.Rate(self.publish_rate)

        start_time = rospy.Time().now()
        
        rospy.loginfo("Start ur16e_singularity_test ...")
        while not rospy.is_shutdown():
            self.circular_movement()
            now = rospy.Time().now()
            self.alpha = self.alpha + (self.angular_velocity_vector[2] * (now.to_sec() - start_time.to_sec()))
            print(now.to_sec())
            print(start_time.to_sec())
            print(now.to_sec() - start_time.to_sec())
            rate.sleep()
            
    def circular_movement(self):
        
        radius_vector = np.array([
            np.sin(self.alpha) * self.radius,
            np.cos(self.alpha) * self.radius,
            0.0
        ])
        cartesian_velocity = np.asarray(np.cross(self.angular_velocity_vector,radius_vector))
        self.joint_velocity_msg.linear.x = cartesian_velocity[0]
        self.joint_velocity_msg.linear.y = cartesian_velocity[1]
        self.joint_velocity_msg.linear.z = cartesian_velocity[2]
        self.joint_velocity_msg.angular.x = 0.0
        self.joint_velocity_msg.angular.y = 0.0
        self.joint_velocity_msg.angular.z = 0.0
        
        # * Publish the target_joint_velocity
        self.pub_joint_velocity.publish(self.joint_velocity_msg)
    
if __name__ == '__main__':
    ur16e_singularity_test()