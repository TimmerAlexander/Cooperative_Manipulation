#!/usr/bin/env python3

import rospy
from franka_gripper.msg import GraspActionGoal

if __name__ == "__main__":
    '''
    This node is used to set the pose of gripper. You can also change the pose of gripper by 
    publishing the following topic in terminal.
    '''
    rospy.init_node("grippe_state")
    
    namespace = rospy.get_param("~panda_ns")
    
    panda_goal_width = rospy.get_param("~panda_goal_width")
    panda_goal_epsilon_inner = rospy.get_param("~panda_goal_epsilon_inner")
    panda_goal_epsilon_outer = rospy.get_param("~panda_goal_epsilon_outer")
    panda_goal_speed = rospy.get_param("~panda_goal_speed")
    panda_goal_force = rospy.get_param("~panda_goal_force")

    
    pub = rospy.Publisher('/' + namespace + '/franka_gripper/grasp/goal',
        GraspActionGoal,
        tcp_nodelay=True,
        queue_size=10)
    
    command_msg = GraspActionGoal()
    command_msg.goal.width = panda_goal_width
    command_msg.goal.epsilon.inner = panda_goal_epsilon_inner
    command_msg.goal.epsilon.outer = panda_goal_epsilon_outer
    command_msg.goal.speed = panda_goal_speed
    command_msg.goal.force = panda_goal_force
    
    rospy.sleep(0.5)
    start = rospy.Time.now().to_sec()

    rospy.loginfo("Attempting to change the state of gripper...")
    rospy.sleep(0.5)
    
    while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start < 1.):
        # print rospy.Time.now()
        command_msg.header.stamp = rospy.Time.now()
        pub.publish(command_msg)

    rospy.loginfo("Gripper forced to target. Complete!")