#!/usr/bin/env python3

# /***************************************************************************

# **************************************************************************/


import rospy    
import numpy as np
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64MultiArray
from cooperative_manipulation_controllers.msg import SingularityAvoidance
import moveit_commander
import sys



class singularity_avoidance_node():
    
    def config(self):
        
        self.publish_rate = 100 # [HZ] 
        self.singularity_msg = SingularityAvoidance()
        self.singularity_stop = False
        self.singularity_avoidance_velocity = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        
        
    def __init__(self):
        
        self.config()
        
        # * Initialize node
        rospy.init_node('singularity_avoidance_node', anonymous=True)
        
        rospy.loginfo('Start singularity_avoidance_node ...')
        
        # Subscriber
        self.cartesian_msg_sub = rospy.Subscriber(
            '/cooperative_manipulation/ur16e/singularity_velocity', 
            SingularityAvoidance, 
            self.singularity_velocity_callback,
            queue_size=1,
            tcp_nodelay=True)
        
        
        # Publisher     
        self.singularity_velocity_pub = rospy.Publisher(
            "/cooperative_manipulation/singularity_velocity",
            SingularityAvoidance,
            queue_size=1)


        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():

                
            # Publish
            self.singularity_msg.singularity_stop = self.singularity_stop
            self.singularity_msg.singularity_velocity = self.singularity_avoidance_velocity
            
            
            self.singularity_velocity_pub.publish(self.singularity_msg)
            rate.sleep()
            

    
    def singularity_velocity_callback(self,singularity_velocity):
        """

        """
        self.singularity_stop = singularity_velocity.singularity_stop
        self.singularity_avoidance_velocity = singularity_velocity.singularity_velocity
            
   
if __name__ == '__main__':
    singularity_avoidance_node()
