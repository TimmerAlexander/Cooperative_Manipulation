#!/usr/bin/env python3

import rospy
from franka_gripper.msg import GraspActionGoal

if __name__ == "__main__":
    '''
    This node is used to set the pose of gripper. You can also change the pose of gripper by 
    publishing the following topic in terminal.
    '''
    rospy.init_node("grippe_state")
    pub = rospy.Publisher('franka_gripper/grasp/goal',
        GraspActionGoal,
        tcp_nodelay=True,
        queue_size=10)

    command_msg = GraspActionGoal()
    command_msg.goal.width = 0.01
    command_msg.goal.epsilon.inner = 0.005
    command_msg.goal.epsilon.outer = 0.005
    command_msg.goal.speed = 0.1
    command_msg.goal.force = 60
    
    rospy.sleep(0.5)
    start = rospy.Time.now().to_sec()

    rospy.loginfo("Attempting to change the state of gripper...")
    rospy.sleep(0.5)
    
    while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start < 1.):
        # print rospy.Time.now()
        command_msg.header.stamp = rospy.Time.now()
        pub.publish(command_msg)

    rospy.loginfo("Gripper forced to target. Complete!")