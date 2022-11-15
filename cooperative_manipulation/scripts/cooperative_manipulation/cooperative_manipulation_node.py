#!/usr/bin/env python3

# /***************************************************************************

# **************************************************************************/


import rospy , tf, tf2_ros
import numpy as np
from cooperative_manipulation_controllers.msg import SingularityAvoidance, WorkspaceViolation
from geometry_msgs.msg import TransformStamped, PoseStamped





class singularity_avoidance_node():
    
    def config(self):

        self.publish_rate = 200 # [HZ] 
        self.singularity_velocity = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.singular_velocity_msg = SingularityAvoidance()
        
        self.workspace_violation_msg = WorkspaceViolation()
        self.singularity_avoidance_stop = False
        self.workspace_violation = False
        
        self.gripper_distance = 1.20 #[m]
        
    def __init__(self):
        
        self.config()
        # * Initialize node
        rospy.init_node('cooperative_manipulation_node', anonymous=True)
        
        #* Publish singularity velocity
        self.singularity_velocity_pub = rospy.Publisher(
            "/cooperative_manipulation/singularity_velocity",
            SingularityAvoidance,
            queue_size=1)
        
        # Publish singularity velocity
        self.workspace_violation_pub = rospy.Publisher(
            "/cooperative_manipulation/workspace",
            WorkspaceViolation,
            queue_size=1)
        
        #* Subscriber to singularity avoidance velocity of the ur16e
        self.singularity_msg_sub = rospy.Subscriber(
            '/cooperative_manipulation/ur16e/singularity_velocity', 
            SingularityAvoidance, 
            self.singularity_velocity_callback,
            queue_size=1,
            tcp_nodelay=True)
        
        #* Subscriber to workspace violation of the Franka Emika
        self.workspace_msg_sub = rospy.Subscriber(
            '/cooperative_manipulation/franka/workspace',
            WorkspaceViolation,
            self.workspace_violation_callback, 
            queue_size=1,
            tcp_nodelay=True)
        
        #* Initialize tf TransformBroadcaster
        # self.brodacaster = tf2_ros.StaticTransformBroadcaster()
        self.brodacaster = tf.TransformBroadcaster()
        
        self.tf_listener = tf.TransformListener()
        
        
        # Set gripper distance from 'world' frame
        robot_distance_x = self.gripper_distance/2
        #* Get ur16e tf frames
        # Wait for Transform
        rospy.loginfo("Wait for transformation '/base_link' to '/ur16e_gripper")
        self.tf_listener.waitForTransform("/base_link","/ur16e_gripper", rospy.Time(), rospy.Duration(60.0))

        # Get EE pose in 'base_link' frame
        ur_tf_time_EE = self.tf_listener.getLatestCommonTime("/base_link", "/ur16e_gripper")
        ur_base_EE_position, ur_base_EE_quaternion = self.tf_listener.lookupTransform("/base_link", "/ur16e_gripper", ur_tf_time_EE)
        # Transform EE pose from 'base_link' frame to 'world' frame
        ur_EE_world_pose = self.transform_pose_stamped('/base_link','/world',ur_base_EE_position,ur_base_EE_quaternion)

        self.ur_offset_x = -robot_distance_x - ur_EE_world_pose .pose.position.x
        self.ur_offset_y = -ur_EE_world_pose .pose.position.y
        
        
        #* Get panda tf frames
        # Wait for Transform
        rospy.loginfo("Wait for transformation 'panda_link0' to 'panda_EE'.")
        self.tf_listener.waitForTransform("panda_link0","panda_EE", rospy.Time(), rospy.Duration(60.0))
        
        # Get EE pose in 'panda_link0' frame
        panda_tf_time_EE = self.tf_listener.getLatestCommonTime("/panda_link0", "/panda_EE")
        panda_link0_EE_position, panda_link0_EE_quaternion = self.tf_listener.lookupTransform("/panda_link0", "/panda_EE", panda_tf_time_EE)
        # Transform EE pose from 'panda_link0' frame to 'world' frame
        panda_EE_world_pose = self.transform_pose_stamped('/panda_link0','/world',panda_link0_EE_position,panda_link0_EE_quaternion)
        
        self.panda_offset_x = robot_distance_x - panda_EE_world_pose.pose.position.x
        self.panda_offset_y = -panda_EE_world_pose.pose.position.y
        

        

        rospy.loginfo("Start cooperative manipulation ...")
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            
            self.set_panda_link0_frame()
            self.set_base_link_frame()
            #* Publish the singualrity_velocity msg
            self.singular_velocity_msg.singularity_stop = self.singularity_avoidance_stop
            self.singular_velocity_msg.singularity_velocity = self.singularity_velocity
            self.singularity_velocity_pub.publish(self.singular_velocity_msg)
            
            #* Publish the workspace violation msg
            self.workspace_violation_msg.workspace_violation = self.workspace_violation
            self.workspace_violation_pub.publish(self.workspace_violation_msg)
            
            rate.sleep()
        rospy.spin()

    #* Callback function for the singularity velocity
    def singularity_velocity_callback(self,singularity_velocity):
        """
            Get the singularity avoidance velocity in 'world' frame.
        Args:
            singularity_velocity (Float64MultiArray): Singularity avoidance velocity
        """
        self.singularity_avoidance_stop = singularity_velocity.singularity_stop
        self.singularity_velocity = singularity_velocity.singularity_velocity
        
    #* Callback function for workspace violation
    def workspace_violation_callback(self,workspace):
        """
            Get the singularity avoidance velocity in 'world' frame.
        Args:
            singularity_velocity (Float64MultiArray): Singularity avoidance velocity
        """
        self.workspace_violation = workspace.workspace_violation        
    
    def set_panda_link0_frame(self):
        """
            Reset panda base frame. 
        """
        self.brodacaster.sendTransform((self.panda_offset_x, self.panda_offset_y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 3.14),
                     rospy.Time.now(),
                     "panda_link0",
                     "world")
        
    def set_base_link_frame(self):
        """
            Reste ur base frame.
        """
        self.brodacaster.sendTransform((self.ur_offset_x, self.ur_offset_y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 3.14),
                     rospy.Time.now(),
                     "base_link",
                     "world")

    def transform_pose_stamped(self,source_frame: str,target_frame: str,source_pose_position: np.array,source_pose_orientation: np.array):
        """_summary_

        Args:
            source_frame (str): _description_
            target_frame (str): _description_
            source_pose_position (np.array): _description_
            source_pose_orientation (np.array): _description_

        Returns:
            _type_: _description_
        """
        now = rospy.Time()
        
        source_position = PoseStamped()
        source_position.header.frame_id = source_frame
        source_position.header.stamp = now
        source_position.pose.position.x = source_pose_position[0]
        source_position.pose.position.y = source_pose_position[1]
        source_position.pose.position.z = source_pose_position[2]
        source_position.pose.orientation.x = source_pose_orientation[0]
        source_position.pose.orientation.y = source_pose_orientation[1]
        source_position.pose.orientation.z = source_pose_orientation[2]
        source_position.pose.orientation.w = source_pose_orientation[3]
        
        target_pose = self.tf_listener.transformPose(target_frame,source_position)
 
        return target_pose
        
if __name__ == '__main__':
    singularity_avoidance_node()