#!/usr/bin/env python2.7
import rospy    
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy
import moveit_commander


class ur16e_move_to_start():
    
    def config(self):
        self.velocity_cmd = 0.01
        self.Kp  = 0.1
        self.max_vel = 0.1
        self.min_vel = 0.005
        self.joint_velocity_msg = Float64MultiArray()
        self.shutdown_joint_velocity_msg = Float64MultiArray()
        self.publish_rate = 100
        self.current_joint_states_array = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.shutdown_joint_velocity_msg.data = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_velocity_msg.data = [0.0,0.0,0.0,0.0,0.0,0.0]
        
        
    def __init__(self):
        
        self.config()
            
        # * Initialize node
        rospy.init_node('ur16e_start_pose', anonymous=True)
        # try:
        #     print("Start ur16e_move_to_star.py")
        #     group_name = 'manipulator'
        #     print("Initialize movit_commander. Group name: ",group_name)
        #     self.group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers=20.0)
        # except Exception as e: 
        #     print(e)
        
        # * Initialize on_shutdown clean up
        rospy.on_shutdown(self.shutdown)
        
       
        # * Get joint poses from launch file
        self.shoulder_pan_joint_pose = float(rospy.get_param("~shoulder_pan_joint"))
        self.shoulder_lift_joint_pose = float(rospy.get_param("~shoulder_lift_joint"))
        self.elbow_joint_pose = float(rospy.get_param("~elbow_joint"))
        self.wrist_1_joint_pose = float(rospy.get_param("~wrist_1_joint"))
        self.wrist_2_joint_pose = float(rospy.get_param("~wrist_2_joint"))
        self.wrist_3_joint_pose = float(rospy.get_param("~wrist_3_joint"))
        # Save the 
        self.joint_poses = [self.shoulder_pan_joint_pose,self.shoulder_lift_joint_pose,self.elbow_joint_pose,self.wrist_1_joint_pose,self.wrist_2_joint_pose,self.wrist_3_joint_pose]
        
        print("Desired joint poses:")
        print(self.joint_poses)
        # ----------------------------------------------
        # Simulation
        # * Get namespace for topics from launch file
        # self.namespace = rospy.get_param("~ur_ns")

        # self.pub_joint_velocity = rospy.Publisher("/" + self.namespace + "/ur_admittance_controller/command", Float64MultiArray, queue_size=1)
        
        # self.joint_states_sub = rospy.Subscriber("/" + self.namespace + "/joint_states",
        #     JointState,
        #     self.joint_state_callback,
        #     queue_size=1)

        # ----------------------------------------------
        # Hardware
        self.pub_joint_velocity = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=1)
        
        
        
        self.joint_states_sub = rospy.Subscriber("/joint_states",JointState,self.joint_state_callback)

        rospy.wait_for_message("/joint_states",JointState,timeout=5.0)

        print("wait for messag")
        print(self.current_joint_states_array )
        # ----------------------------------------------
        self.run()
        rospy.spin()
   

    def run(self):
        
        rate = rospy.Rate(self.publish_rate)
        for i in range(len(self.joint_poses)):
            
            self.delta_pose = round(self.joint_poses[i] - self.current_joint_states_array[i],3)

            while self.delta_pose != 0.0 and not rospy.is_shutdown():
                
                
                self.delta_pose = round(self.joint_poses[i] - self.current_joint_states_array[i],3)

                vel_cmd = self.Kp * self.delta_pose
                if abs(vel_cmd) > self.max_vel:
                    self.joint_velocity_msg.data[i] = numpy.sign(vel_cmd) * self.max_vel
                elif abs(vel_cmd) < self.min_vel:
                    self.joint_velocity_msg.data[i] = numpy.sign(vel_cmd) * self.min_vel
                else:
                    self.joint_velocity_msg.data[i] = vel_cmd

                self.pub_joint_velocity.publish(self.joint_velocity_msg)
                
                print("self.joint_poses[{}]:".format(i),self.joint_poses[i])
                print("self.current_joint_states_array[{}]:".format(i),self.current_joint_states_array[i])
                print("self.delta_pose:",self.delta_pose)

                self.delta_pose = round(self.joint_poses[i] - self.current_joint_states_array[i],3)
                rate.sleep()

            self.joint_velocity_msg.data = [0.0,0.0,0.0,0.0,0.0,0.0]
            self.pub_joint_velocity.publish(self.joint_velocity_msg)

        self.pub_joint_velocity.publish(self.shutdown_joint_velocity_msg)
        
        
    
    def joint_state_callback(self,current_joint_states):
        """
            Get current joint state.

            self.current_joint_states_array = [
                shoulder_pan_joint,
                shoulder_lift_joint,
                elbow_joint,
                wrist_1_joint,
                wrist_2_joint,
                wrist_3_joint]
        """
        self.current_joint_states_array = [
            current_joint_states.position[2],
            current_joint_states.position[1],
            current_joint_states.position[0],
            current_joint_states.position[3],
            current_joint_states.position[4],
            current_joint_states.position[5]]





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