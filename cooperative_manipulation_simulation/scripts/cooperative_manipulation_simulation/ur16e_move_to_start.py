#!/usr/bin/env python3

# /***************************************************************************

# **************************************************************************/


import rospy    
from std_msgs.msg import Float64MultiArray



class ur16e_start_pose():
    
            
    def __init__(self):
        
        self.joint_position_msg = Float64MultiArray()
            
        # * Initialize node
        rospy.init_node('ur16e_start_pose', anonymous=True)
        # * Get namespace for topics from launch file
        self.namespace = rospy.get_param("~ur_ns")
        # * Get joint poses from launch file
        self.shoulder_pan_joint = float(rospy.get_param("~shoulder_pan_joint"))
        self.shoulder_lift_joint = float(rospy.get_param("~shoulder_lift_joint"))
        self.elbow_joint = float(rospy.get_param("~elbow_joint"))
        self.wrist_1_joint = float(rospy.get_param("~wrist_1_joint"))
        self.wrist_2_joint = float(rospy.get_param("~wrist_2_joint"))
        self.wrist_3_joint = float(rospy.get_param("~wrist_3_joint"))
        
        self.pub_joint_position = rospy.Publisher("/" + self.namespace + "/ur16e_start_position_controller/command", Float64MultiArray, queue_size=1)
        
        rate = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            
            self.joint_position_msg.data = [self.shoulder_pan_joint,self.shoulder_lift_joint,self.elbow_joint,self.wrist_1_joint,self.wrist_2_joint,self.wrist_3_joint]
            
            self.pub_joint_position.publish(self.joint_position_msg)
            rate.sleep()
            
if __name__ == '__main__':
    ur16e_start_pose()