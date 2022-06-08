#!/usr/bin/env python3



import rospy    
from std_msgs.msg import Float64MultiArray
import numpy
import moveit_commander


class ur16e_move_to_start():
    
    def config(self):
        self.velocity_cmd = 0.01
        self.joint_velocity_msg = Float64MultiArray()
        self.shutdown_joint_velocity_msg = Float64MultiArray()
        
        
    def __init__(self):
        
        self.config()
            
        # * Initialize node
        rospy.init_node('ur16e_start_pose', anonymous=True)
        
        try:
            print("Start ur16e_move_to_star.py")
            group_name = 'manipulator'
            print("Initialize movit_commander. Group name: ",group_name)
            self.group = moveit_commander.MoveGroupCommander(group_name)
        except Exception as e: 
            print(e)
        
        # * Initialize on_shutdown clean up
        rospy.on_shutdown(self.shutdown)
        
        # * Get namespace for topics from launch file
        self.namespace = rospy.get_param("~ur_ns")
        # * Get joint poses from launch file
        self.shoulder_pan_joint_pose = float(rospy.get_param("~shoulder_pan_joint"))
        self.shoulder_lift_joint_pose = float(rospy.get_param("~shoulder_lift_joint"))
        self.elbow_joint_pose = float(rospy.get_param("~elbow_joint"))
        self.wrist_1_joint_pose = float(rospy.get_param("~wrist_1_joint"))
        self.wrist_2_joint_pose = float(rospy.get_param("~wrist_2_joint"))
        self.wrist_3_joint_pose = float(rospy.get_param("~wrist_3_joint"))
        # Save the 
        self.joint_poses = [self.elbow_joint_pose,self.shoulder_lift_joint_pose,self.shoulder_pan_joint_pose,self.wrist_1_joint_pose,self.wrist_2_joint_pose,self.wrist_3_joint_pose]
        
        
        self.pub_joint_velocity = rospy.Publisher("/" + self.namespace + "/ur16e_start_position_controller/command", Float64MultiArray, queue_size=1)
        
        self.shutdown_joint_velocity_msg.data = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_velocity_msg.data = [0.0,0.0,0.0,0.0,0.0,0.0]

        
        for i in range(len(self.joint_poses)):
            
            self.current_joint_states_array = self.group.get_current_joint_values() 
            self.delta_pose = round(self.joint_poses[i] - self.current_joint_states_array[i],3)
            self.joint_velocity_msg.data[i] = numpy.sign(self.delta_pose) * self.velocity_cmd

            while self.delta_pose != 0.0 and not rospy.is_shutdown():
                
                self.pub_joint_velocity.publish(self.joint_velocity_msg)
                self.current_joint_states_array = self.group.get_current_joint_values() 
                
                print("self.joint_poses[{}]:".format(i),self.joint_poses[i])
                print("self.current_joint_states_array[{}]:".format(i),self.current_joint_states_array[i])
                print("self.delta_pose:",self.delta_pose)

                self.delta_pose = round(self.joint_poses[i] - self.current_joint_states_array[i],3)
                
            self.joint_velocity_msg.data = [0.0,0.0,0.0,0.0,0.0,0.0]
            self.pub_joint_velocity.publish(self.joint_velocity_msg)
        self.pub_joint_velocity.publish(self.shutdown_joint_velocity_msg)
    
    def shutdown(self):
        """ 
        This function is called by rospy.on_shutdown!
         """
        print("Shutdown ur16e move to start :")
        print("Shutdown publisher joint velocity!")
        self.pub_joint_velocity.publish(self.shutdown_joint_velocity_msg)
        print("Unregister from joint_velocity_pub!")
        self.pub_joint_velocity.unregister()

                    

            # self.joint_position_msg.data = [self.shoulder_pan_joint,self.shoulder_lift_joint,self.elbow_joint,self.wrist_1_joint,self.wrist_2_joint,self.wrist_3_joint]
            
            # self.pub_joint_velocity.publish(self.joint_position_msg)
            


if __name__ == '__main__':
    ur16e_move_to_start()