#!/usr/bin/env python3

# /***************************************************************************

# **************************************************************************/

"""
    Description...
    
    Admittance controller
    
    Input: 
    * Desired cartesian velocity of the EE: desired_velocity (In 'world' frame)
    * External wrench from the f/t sensor: wrench_ext (In 'wrist_3_link' frame)
    
    Output: 
    * Target joint velocity: self.target_joint_velocity (In 'base_link' frame)
"""
from cmath import pi
import sys, copy, numpy, math, rospy, tf, tf2_ros, moveit_commander
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import WrenchStamped, Vector3Stamped, Twist, TransformStamped, PoseStamped
from std_msgs.msg import Float64MultiArray, Float32, Bool
from cooperative_manipulation_controllers.msg import SingularityAvoidance, WorkspaceViolation


class ur_admittance_controller():
    
    def config(self):
        # Control thread publish rate [Hz]
        self.publish_rate = 100
        #* Admittance controler values
        # Array with the contact forces/torques
        self.wrench_contact = numpy.array([1.0,1.0,1.0,0.1,0.1,0.1])
        #self.wrench_contact = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Array to store the wrench difference
        self.wrench_diff = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Admittance gains
        self.A_trans_x = 200
        self.A_trans_y = 200
        self.A_trans_z = 200
        self.A_rot_x = 100
        self.A_rot_y = 100
        self.A_rot_z = 100
        # Kp gains
        self.Kp_trans_x = 1.25
        self.Kp_trans_y = 1.25
        self.Kp_trans_z = 1.25
        self.Kp_rot_x = 0.125
        self.Kp_rot_y = 0.125
        self.Kp_rot_z = 0.125
        #* Min and max limits for the cartesian velocity (trans/rot) [m/s]
        self.cartesian_velocity_trans_max_limit = 0.1
        self.cartesian_velocity_rot_max_limit = 0.1
        #* Bandpass filter
        # Force threshold 
        self.wrench_filter_force_x = 0.7
        self.wrench_filter_force_y = 0.7
        self.wrench_filter_force_z = 0.7
        # Torque treshold 
        self.wrench_filter_torque_x = 0.1
        self.wrench_filter_torque_y = 0.1
        self.wrench_filter_torque_z = 0.1
        #* Average filter
        # Lists to store the wrench values
        self.average_filter_list_force_x = []
        self.average_filter_list_force_y = []
        self.average_filter_list_force_z = []
        self.average_filter_list_torque_x = []
        self.average_filter_list_torque_y = []
        self.average_filter_list_torque_z = []
        # Wrench average list length
        self.average_filter_list_length = 100
        # Wrench average list lengthself.singularity_stop == False
        self.average_wrench_ext_filtered_array = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        #* Set gripper offset
        self.ur16e_gripper_offset = 0.03 # [m]
        # * Initialize the needed velocity data types
        # Initialize desired velocity transformed form 'wrorld' frame to 'base_link' frame (xdot_desired_base_link)
        self.base_link_desired_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.world_cartesian_velocity_trans = Vector3Stamped()
        self.world_cartesian_velocity_rot = Vector3Stamped()
        # Initialize trajectory velocity for object rotation
        self.world_trajectory_velocity = numpy.array([0.0,0.0,0.0])
        # Initialize velocity from admittance (xdot_a_base_link)
        self.admittance_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Initialize target cartesian velocity array (xdot_target_base_link)
        self.target_cartesian_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Declare target joint velocity msgs (qdot_target_base_ink) (unit: [radian/s])
        self.target_joint_velocity = Float64MultiArray()
        #* Array to store the desired endeffector pose
        self.trajectory_EE_pose = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        #* Franka workspace violation
        self.workspace_violation = False
        #* Singulariy avoidance: OLMM
        self.singularity_velocity_world = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        
        self.reconstruct_trajektory = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        
        self.singularity_stop = False
        self.singularity_stop_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        
        self.singularity_exit_threshold = 0.06 
        
        self.singularity_entry_threshold = 0.05
        
        self.singularity_min_threshold = 0.015
        self.singularity_stop_threshold = 0.01
        
        self.adjusting_scalar = 0.0
        
        self.singularity_velocity_cmd = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.singularity_avoidance_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.singularity_velocity_msg = SingularityAvoidance()
        self.bool_singularity = False
        self.bool_reduce_singularity_offset = False
        self.singularity_trans_offset_accuracy = 0.001 # [m]
        self.singularity_rot_offset_accuracy = 0.0174 # [rad] ~ 1 [°]
        self.last_target_trans_cartesian_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0]) 
        self.last_target_rot_cartesian_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0]) 
        #* Initialize shutdown joint velocity, called on shutdown 
        self.shutdown_joint_velocity = Float64MultiArray()
        self.shutdown_joint_velocity.data = [0.0,0.0,0.0,0.0,0.0,0.0]
        
        
    def __init__(self):
        # * Load config parameters
        self.config()
        
        # * Initialize node
        rospy.init_node('admittance_controller_node', anonymous=True)
        # * Initialize on_shutdown clean up
        rospy.on_shutdown(self.shutdown)
        
        # * Initialize tf TransformBroadcaster
        self.brodacaster = tf2_ros.StaticTransformBroadcaster()
        # * Initialize tf TransformListener
        self.tf_listener = tf.TransformListener()
        rospy.loginfo("Wait for transformation 'wrist_3_link' to 'base_link'.")
        self.tf_listener.waitForTransform("wrist_3_link","base_link", rospy.Time(), rospy.Duration(5.0))
        rospy.loginfo("Wait for transformation 'world' to 'wrist_3_link'.")
        self.tf_listener.waitForTransform("world","wrist_3_link", rospy.Time(), rospy.Duration(5.0))
        # rospy.loginfo("Wait for transformation 'world' to 'base_link'.")
        self.tf_listener.waitForTransform("world","base_link", rospy.Time(), rospy.Duration(5.0))
        
        # Initialize the 'ur16e_gripper' frame in tf tree
        self.set_gripper_offset()
        
        # Wait for transformations from 'world' to 'ur16e_gripper' and 'world' to 'panda_EE'
        rospy.loginfo("Wait for transformation 'world' to 'ur16e_gripper'.")
        self.tf_listener.waitForTransform("world","ur16e_gripper", rospy.Time(), rospy.Duration(10.0))
        rospy.loginfo("Wait for transformation 'world' to 'panda_EE'.")
        self.tf_listener.waitForTransform("world","panda_EE", rospy.Time(), rospy.Duration(60.0))

        # * Get namespace for topics from launch file
        self.namespace = rospy.get_param("~ur_ns")
        
        # * Initialize move_it
        moveit_commander.roscpp_initialize(sys.argv)
        try:
            group_name = 'manipulator'
            print("Initialize movit_commander. Group name: ",group_name)
            self.group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers=5.0)
        except Exception as e: 
            print(e)
        
        # * Initialize publisher:
        # Publish final joint velocity to "/ur/joint_group_vel_controller/command"
        self.joint_velocity_pub = rospy.Publisher(
            "/" + self.namespace + "/joint_group_vel_controller/command",
            Float64MultiArray,
            queue_size=1)
        
        # Publish singularity velocity
        self.singularity_velocity_pub = rospy.Publisher(
            "/cooperative_manipulation/ur16e/singularity_velocity",
            SingularityAvoidance,
            queue_size=1)
        
        # * Initialize subscriber:
        # Subscriber to "/ur/wrench"
        self.wrench_ext_sub = rospy.Subscriber(
            "/" + self.namespace + "/wrench",
            WrenchStamped,
            self.wrench_callback,queue_size=1)
        
        # Subscriber to "/ur/cooperative_manipulation/cartesian_velocity_command"
        self.cartesian_velocity_command_sub = rospy.Subscriber(
            "/cooperative_manipulation/cartesian_velocity_command",
            Twist,
            self.cartesian_velocity_command_callback,queue_size=1)
        
        
        # Subscriber to workspace violation of the Franka Emika
        self.workspace_msg_sub = rospy.Subscriber(
            "/cooperative_manipulation/workspace",
            WorkspaceViolation,
            self.workspace_violation_callback, 
            queue_size=1,
            tcp_nodelay=True)
        
        
        
        # Publisher and Subscriber for measurements-------------------------------------------------------------
        # * Initialize subscriber:

        self.delta_pos_msg = Float64MultiArray()
        self.delta_ori_msg = Float64MultiArray()
        self.sigma_msg = Float32()
        self.singularity_msg = Bool()
        self.offset_msg = Bool()
        
        self.delta_pos_publisher = rospy.Publisher(
                '/' + self.namespace + '/measurement/delta_pos',
                Float64MultiArray,
                tcp_nodelay=True,
                queue_size=1)
        
        self.delta_ori_publisher = rospy.Publisher(
                '/' + self.namespace + '/measurement/delta_ori',
                Float64MultiArray,
                tcp_nodelay=True,
                queue_size=1)
        
        self.sigma_publisher = rospy.Publisher(
                '/' + self.namespace + '/measurement/sigma',
                Float32,
                tcp_nodelay=True,
                queue_size=1)
        
        self.sigularity_publisher = rospy.Publisher(
                '/' + self.namespace + '/measurement/sigularity',
                Bool,
                tcp_nodelay=True,
                queue_size=1)
        
        self.offset_publisher = rospy.Publisher(
                '/' + self.namespace + '/measurement/reduce_offset',
                Bool,
                tcp_nodelay=True,
                queue_size=1)

        
        
        
        # Publisher and Subscriber for measurements-------------------------------------------------------------
        
        
        
        
        
        # Converse input_vector rotation from numpy.array to vector3
        self.world_cartesian_velocity_rot.header.frame_id = 'world'
        self.world_cartesian_velocity_rot.header.stamp = rospy.Time()
        self.world_cartesian_velocity_rot.vector.x = 0.0
        self.world_cartesian_velocity_rot.vector.y = 0.0
        self.world_cartesian_velocity_rot.vector.z = 0.0
        
        # * Get the start EE pose
        # Get start EE pose in 'base_link' frame
        start_EE_pose = self.group.get_current_pose('wrist_3_link')
        print(start_EE_pose)
        
        start_EE_pose_quaternion = (start_EE_pose.pose.orientation.x,
                                    start_EE_pose.pose.orientation.y,
                                    start_EE_pose.pose.orientation.z,
                                    start_EE_pose.pose.orientation.w)

        # Transform quaternion to euler
        start_EE_pose_euler = euler_from_quaternion(start_EE_pose_quaternion)
        
        self.start_trajectory_EE_pose_array = numpy.array([start_EE_pose.pose.position.x,
                                                            start_EE_pose.pose.position.y,
                                                            start_EE_pose.pose.position.z,
                                                            start_EE_pose_euler[0],
                                                            start_EE_pose_euler[1],
                                                            start_EE_pose_euler[2]])
        
        self.trajectory_EE_pose_array = self.start_trajectory_EE_pose_array
        
        # * Run control_thread
        rospy.loginfo("Recieved messages; Launch ur16e Admittance control.")
        self.control_thread()
        
        rospy.spin()
        
    def set_gripper_offset(self):
        """
            Set the gripper offset from 'wirst_3_link' frame.
        """
        static_gripper_offset = TransformStamped()
        static_gripper_offset.header.stamp = rospy.Time.now()
        static_gripper_offset.header.frame_id = "wrist_3_link"
        static_gripper_offset.child_frame_id = "ur16e_gripper"
        static_gripper_offset.transform.translation.x = 0.0
        static_gripper_offset.transform.translation.y = 0.0
        static_gripper_offset.transform.translation.z = self.ur16e_gripper_offset
        static_gripper_offset.transform.rotation.x = 0.0
        static_gripper_offset.transform.rotation.y = 0.0
        static_gripper_offset.transform.rotation.z = 0.0
        static_gripper_offset.transform.rotation.w = 1.0

        self.brodacaster.sendTransform(static_gripper_offset) 
    
    def transform_pose_stamped(self,source_frame: str,target_frame: str,source_pose: numpy.array):
        """ Transforms a cartesian PoseStamped array from the source frame to the target frame. 

        Args:
            source_frame (str): The frame to transform from 
            target_frame (str): The frame to transform to
            source_pose (numpy.array): The PoseStamped in source frame as array

        Returns:
            numpy.array: The PoseStamped in target frame as array
        """
        now = rospy.Time()
        
        source_position = PoseStamped()
        source_position.header.frame_id = source_frame
        source_position.header.stamp = now
        source_position.pose.position.x = source_pose[0]
        source_position.pose.position.y = source_pose[1]
        source_position.pose.position.z = source_pose[2]
        
        source_frame_quaternion = quaternion_from_euler(source_pose[3],source_pose[4],source_pose[5])
                
        source_position.pose.orientation.x = source_frame_quaternion[0]
        source_position.pose.orientation.y = source_frame_quaternion[1]
        source_position.pose.orientation.z = source_frame_quaternion[2]
        source_position.pose.orientation.w = source_frame_quaternion[3]
        
        target_frame_position = self.tf_listener.transformPose(target_frame,source_position)
        
        target_frame_quaternion =  (target_frame_position.pose.orientation.x,
                                    target_frame_position.pose.orientation.y,
                                    target_frame_position.pose.orientation.z,
                                    target_frame_position.pose.orientation.w)
        
        target_frame_euler = euler_from_quaternion(target_frame_quaternion)
        
        target_pose = numpy.array([target_frame_position.pose.position.x,
                                    target_frame_position.pose.position.y,
                                    target_frame_position.pose.position.z,
                                    target_frame_euler[0],
                                    target_frame_euler[1],
                                    target_frame_euler[2]])  
        return target_pose
    
    def scalar_adjusting_function(self,current_sigma):
        """Compute the adjusting scalar for the OLMM. 3 varaitions to calculate the adjusting scalar are presented.

            Source: QIU, Changwu; CAO, Qixin; MIAO, Shouhong. An on-line task modification method for singularity avoidance of robot manipulators. Robotica, 2009, 27. Jg., Nr. 4, S. 539-546.

        Args:
            current_sigma (float): The current singular value

        Returns:
            float: The adjusting_scalar
        """
        
        
        # 0.
        # adjusting_scalar = (1-current_sigma)
        # 1.
        # adjusting_scalar = (1-(current_sigma/self.singularity_entry_threshold))
        # 2.
        # adjusting_scalar = (1-(current_sigma/self.singularity_entry_threshold)**(1/2))
        # 3.
        adjusting_scalar = (1-(current_sigma/self.singularity_entry_threshold)**(3/2))
        # 4.
        adjusting_scalar = 0.5 * (1 + math.cos(pi*((current_sigma-self.singularity_min_threshold)/(self.singularity_entry_threshold - self.singularity_min_threshold))))
        
        return adjusting_scalar 
    
    def transform_vector(self,source_frame: str,target_frame: str,input_vector: numpy.array):
        """ 
            Transforms a vector from source frame to target frame.

        Args:
            source_frame (str): The frame to transform from 
            target_frame (str): The frame to transform to
            input_array (numpy.array): The vector in source frame as array

        Returns:
            numpy.array: The vector in target frame as array
        """
        source_frame_cartesian_velocity_trans = Vector3Stamped()
        source_frame_cartesian_velocity_rot = Vector3Stamped()
        
        # Get current time stamp
        now = rospy.Time()
 
        # Converse input_vector translation from numpy.array to vector3
        source_frame_cartesian_velocity_trans.header.frame_id = source_frame
        source_frame_cartesian_velocity_trans.header.stamp = now
        source_frame_cartesian_velocity_trans.vector.x = input_vector[0]
        source_frame_cartesian_velocity_trans.vector.y = input_vector[1]
        source_frame_cartesian_velocity_trans.vector.z = input_vector[2]
        
        # Transform input_vector translation from 'wrist_3_link' frame to 'base_link' frame
        target_frame_cartesian_velocity_trans = self.tf_listener.transformVector3(target_frame,source_frame_cartesian_velocity_trans)
        
        # Converse input_vector rotation from numpy.array to vector3
        source_frame_cartesian_velocity_rot.header.frame_id = source_frame
        source_frame_cartesian_velocity_rot.header.stamp = now
        source_frame_cartesian_velocity_rot.vector.x = input_vector[3]
        source_frame_cartesian_velocity_rot.vector.y = input_vector[4]
        source_frame_cartesian_velocity_rot.vector.z = input_vector[5]
        
        # Transform input_vector rotation from 'wrist_3_link' frame to 'base_link' frame
        target_frame_cartesian_velocity_transrot = self.tf_listener.transformVector3(target_frame,source_frame_cartesian_velocity_rot)
        
        # Converse input_vector from vector3 to numpy.array
        output_vector = numpy.array([
            target_frame_cartesian_velocity_trans.vector.x,
            target_frame_cartesian_velocity_trans.vector.y,
            target_frame_cartesian_velocity_trans.vector.z,
            target_frame_cartesian_velocity_transrot.vector.x,
            target_frame_cartesian_velocity_transrot.vector.y,
            target_frame_cartesian_velocity_transrot.vector.z
            ])
        
        return output_vector
    
        
    def workspace_violation_callback(self,workspace_violation):
        """_summary_

        Args:
            workspace_violation (_type_): _description_
        """
        self.workspace_violation = workspace_violation.workspace_violation
    
    def cartesian_velocity_command_callback(self,desired_velocity):
        """
            Get the cartesian velocity command and transform it from from the 'world' frame to the 'base_link' and 'wrist_link_3' frame.
            
            Send example velocity:
            rostopic pub -r 10 cooperative_manipulation/cartesian_velocity_command geometry_msgs/Twist "linear:
            x: 0.0
            y: 0.0
            z: 0.0
            angular:
            x: 0.0
            y: 0.0
            z: 0.0" 
        """
        # Get current time stamp
        now = rospy.Time()

        # Calculate the trajectory velocity of the manipulator for a rotation of the object-----------------------------
        # Get ur16e_current_position, ur16e_current_quaternion of the 'wrist_3_link' in frame in the 'world' frame 
        ur16e_tf_time = self.tf_listener.getLatestCommonTime("world", "wrist_3_link")
        ur16e_current_position, ur16e_current_quaternion = self.tf_listener.lookupTransform("/world", "/wrist_3_link", ur16e_tf_time)

        # Get ur16e_current_position, ur16e_current_quaternion of the 'ur16e_gripper' in frame in the 'world' frame 
        ur16e_tf_time = self.tf_listener.getLatestCommonTime("world", "ur16e_gripper")
        ur16e_gripper_position, ur16e_gripper_quaternion = self.tf_listener.lookupTransform("/world", "/ur16e_gripper", ur16e_tf_time)

        # # Get self.panda_current_position, self.panda_current_quaternion of the 'panda_EE' frame in the 'world' frame 
        panda_tf_time = self.tf_listener.getLatestCommonTime("world", "panda_EE")
        panda_gripper_position, panda_gripper_quaternion = self.tf_listener.lookupTransform("world", "panda_EE", panda_tf_time)

        # Object rotation around x axis 
        if desired_velocity.angular.x != 0.0:
            ur16e_current_position_x = numpy.array([
                0.0,
                ur16e_current_position[1],
                ur16e_current_position[2]])
            
            self.robot_distance_x = numpy.array([0.0,
                panda_gripper_position[1] - ur16e_gripper_position[1],
                panda_gripper_position[2] - ur16e_gripper_position[2],
            ])
            
            
            center_x = (numpy.linalg.norm(self.robot_distance_x)/2) * (1/numpy.linalg.norm(self.robot_distance_x)) * self.robot_distance_x + ur16e_gripper_position
            world_desired_rotation_x = numpy.array([desired_velocity.angular.x,0.0,0.0])
            world_radius_x = ur16e_current_position_x - center_x
            self.world_trajectory_velocity_x = numpy.cross(world_desired_rotation_x,world_radius_x)
            self.world_trajectory_velocity = self.world_trajectory_velocity + self.world_trajectory_velocity_x
            
            
        # Object rotation around y axis 
        if desired_velocity.angular.y != 0.0: 
            ur16e_current_position_y = numpy.array([
                ur16e_current_position[0],
                0.0,
                ur16e_current_position[2]
                ]) 
            
            self.robot_distance_y = numpy.array([
                panda_gripper_position[0] - ur16e_gripper_position[0],
                0.0,
                panda_gripper_position[2] - ur16e_gripper_position[2],
                ])
            
            center_y = (numpy.linalg.norm(self.robot_distance_y)/2) * (1/numpy.linalg.norm(self.robot_distance_y)) * self.robot_distance_y + ur16e_gripper_position
            world_desired_rotation_y = numpy.array([0.0,desired_velocity.angular.y,0.0])
            world_radius_y = ur16e_current_position_y - center_y
            self.world_trajectory_velocity_y = numpy.cross(world_desired_rotation_y,world_radius_y)
            self.world_trajectory_velocity = self.world_trajectory_velocity + self.world_trajectory_velocity_y 

        # Object rotation around z axis 
        if desired_velocity.angular.z != 0.0:
            ur16e_current_position_z = numpy.array([
                ur16e_current_position[0],
                ur16e_current_position[1],
                0.0,
                ]) 
                            
            self.robot_distance_z = numpy.array([
                panda_gripper_position[0] - ur16e_gripper_position[0],
                panda_gripper_position[1] - ur16e_gripper_position[1],
                0.0,
                ])
            
            
            center_z = (numpy.linalg.norm(self.robot_distance_z)/2) * (1/numpy.linalg.norm(self.robot_distance_z)) * self.robot_distance_z + ur16e_gripper_position
            world_desired_rotation_z = numpy.array([0.0,0.0,desired_velocity.angular.z])
            world_radius_z = ur16e_current_position_z - center_z
            self.world_trajectory_velocity_z = numpy.cross(world_desired_rotation_z,world_radius_z)
            self.world_trajectory_velocity = self.world_trajectory_velocity + self.world_trajectory_velocity_z     
            


        # Transform the velocity from 'world' frame to 'base_link' frame------------------------------------------------
        # Converse cartesian_velocity translation to vector3
        self.world_cartesian_velocity_trans.header.frame_id = 'world'
        self.world_cartesian_velocity_trans.header.stamp = now
        self.world_cartesian_velocity_trans.vector.x = desired_velocity.linear.x + self.world_trajectory_velocity[0]
        self.world_cartesian_velocity_trans.vector.y = desired_velocity.linear.y + self.world_trajectory_velocity[1]
        self.world_cartesian_velocity_trans.vector.z = desired_velocity.linear.z + self.world_trajectory_velocity[2]
        
        # Transform cartesian_velocity translation from 'world' frame to 'base_link' frame
        self.base_link_cartesian_desired_velocity_trans = self.tf_listener.transformVector3('base_link',self.world_cartesian_velocity_trans)
        
        
        # Converse cartesian_velocity rotation to vector3
        self.world_cartesian_velocity_rot.header.frame_id = 'world'
        self.world_cartesian_velocity_rot.header.stamp = now
        self.world_cartesian_velocity_rot.vector.x = desired_velocity.angular.x
        self.world_cartesian_velocity_rot.vector.y = desired_velocity.angular.y
        self.world_cartesian_velocity_rot.vector.z = desired_velocity.angular.z
        
        # Transform cartesian_velocity rotation from 'world' frame to 'base_link' frame
        self.base_link_cartesian_desired_velocity_rot = self.tf_listener.transformVector3('base_link',self.world_cartesian_velocity_rot)
        
        # Converse cartesian_velocity from vector3 to numpy.array
        self.base_link_desired_velocity = numpy.array([
            self.base_link_cartesian_desired_velocity_trans.vector.x,
            self.base_link_cartesian_desired_velocity_trans.vector.y,
            self.base_link_cartesian_desired_velocity_trans.vector.z,
            self.base_link_cartesian_desired_velocity_rot.vector.x ,
            self.base_link_cartesian_desired_velocity_rot.vector.y ,
            self.base_link_cartesian_desired_velocity_rot.vector.z 
            ])
        
        # Set the trajectory velocity for an object rotation to zero
        self.world_trajectory_velocity = [0.0,0.0,0.0]
    
    def wrench_callback(self,wrench_ext):
        """ 
            Get external wrench in 'tool0_controller' frame.
            
        """
        # * Average filter
        # Fill the empty lists with wrench values
        if len(self.average_filter_list_force_x) < self.average_filter_list_length:
            # 2. Add the new wrench to the list 
            self.average_filter_list_force_x.append(wrench_ext.wrench.force.x) 
            self.average_filter_list_force_y.append(wrench_ext.wrench.force.y) 
            self.average_filter_list_force_z.append(wrench_ext.wrench.force.z) 
            self.average_filter_list_torque_x.append(wrench_ext.wrench.torque.x)
            self.average_filter_list_torque_y.append(wrench_ext.wrench.torque.y)
            self.average_filter_list_torque_z.append(wrench_ext.wrench.torque.z )
            # 3. Calculate the average 
            self.average_force_x = sum(self.average_filter_list_force_x)/len(self.average_filter_list_force_x)
            self.average_force_y = sum(self.average_filter_list_force_y)/len(self.average_filter_list_force_y)
            self.average_force_z = sum(self.average_filter_list_force_z)/len(self.average_filter_list_force_z)
            self.average_torque_x = sum(self.average_filter_list_torque_x)/len(self.average_filter_list_torque_x)
            self.average_torque_y = sum(self.average_filter_list_torque_y)/len(self.average_filter_list_torque_y)
            self.average_torque_z = sum(self.average_filter_list_torque_z)/len(self.average_filter_list_torque_z)
            
        # If the lists reached the length of self.average_filter_list_length
        elif len(self.average_filter_list_force_x) == self.average_filter_list_length:
            # 1. Delete the first element in the list 
            self.average_filter_list_force_x.pop(0)
            self.average_filter_list_force_y.pop(0)
            self.average_filter_list_force_z.pop(0)
            self.average_filter_list_torque_x.pop(0)
            self.average_filter_list_torque_y.pop(0)
            self.average_filter_list_torque_z.pop(0)
            # 2. Add the new wrench to the list 
            self.average_filter_list_force_x.append(wrench_ext.wrench.force.x)
            self.average_filter_list_force_y.append(wrench_ext.wrench.force.y)
            self.average_filter_list_force_z.append(wrench_ext.wrench.force.z)
            self.average_filter_list_torque_x.append(wrench_ext.wrench.torque.x)
            self.average_filter_list_torque_y.append(wrench_ext.wrench.torque.y)
            self.average_filter_list_torque_z.append(wrench_ext.wrench.torque.z)
            # 3. Calculate the average 
            self.average_force_x = sum(self.average_filter_list_force_x)/self.average_filter_list_length
            self.average_force_y = sum(self.average_filter_list_force_y)/self.average_filter_list_length
            self.average_force_z = sum(self.average_filter_list_force_z)/self.average_filter_list_length
            self.average_torque_x = sum(self.average_filter_list_torque_x)/self.average_filter_list_length
            self.average_torque_y = sum(self.average_filter_list_torque_y)/self.average_filter_list_length
            self.average_torque_z = sum(self.average_filter_list_torque_z)/self.average_filter_list_length
            
            
        self.average_wrench_array = (self.average_force_x,
                                     self.average_force_y,
                                     self.average_force_z,
                                     self.average_torque_x,
                                     self.average_torque_y,
                                     self.average_torque_z)

        # * Transform wrench from 'tool0_controller' frame to 'base_link' frame
        self.average_wrench_array_transformed = self.transform_vector('tool0_controller','base_link',self.average_wrench_array)
        

        #* Band-passfilter
        if numpy.linalg.norm(self.base_link_desired_velocity) == 0.0:
            #print("bandsperre")
            if numpy.abs(self.average_wrench_array_transformed[0]) < self.wrench_filter_force_x:
                self.average_wrench_ext_filtered_array[0] = 0.0
            else: 
                self.average_wrench_ext_filtered_array[0] = self.average_wrench_array_transformed[0] - numpy.sign(self.average_wrench_array_transformed[0]) * self.wrench_filter_force_x
                
            if numpy.abs(self.average_wrench_array_transformed[1]) < self.wrench_filter_force_y:
                    self.average_wrench_ext_filtered_array[1] = 0.0
            else: 
                self.average_wrench_ext_filtered_array[1] = self.average_wrench_array_transformed[1] - numpy.sign(self.average_wrench_array_transformed[1]) * self.wrench_filter_force_y

            if numpy.abs(self.average_wrench_array_transformed[2]) < self.wrench_filter_force_z:
                self.average_wrench_ext_filtered_array[2] = 0.0
            else: 
                self.average_wrench_ext_filtered_array[2] = self.average_wrench_array_transformed[2] - numpy.sign(self.average_wrench_array_transformed[2]) *  self.wrench_filter_force_z
            
            if numpy.abs(self.average_wrench_array_transformed[3]) < self.wrench_filter_torque_x:
                self.average_wrench_ext_filtered_array[3] = 0.0
            else: 
                self.average_wrench_ext_filtered_array[3] = self.average_wrench_array_transformed[3]- numpy.sign(self.average_wrench_array_transformed[3]) * self.wrench_filter_torque_x
                
            if numpy.abs(self.average_wrench_array_transformed[4]) < self.wrench_filter_torque_y:
                self.average_wrench_ext_filtered_array[4] = 0.0
            else: 
                self.average_wrench_ext_filtered_array[4] = self.average_wrench_array_transformed[4] - numpy.sign(self.average_wrench_array_transformed[4]) * self.wrench_filter_torque_y
                
            if numpy.abs(self.average_wrench_array_transformed[5]) < self.wrench_filter_torque_z:
                self.average_wrench_ext_filtered_array[5] = 0.0
            else: 
                self.average_wrench_ext_filtered_array[5] = self.average_wrench_array_transformed[5] - numpy.sign(self.average_wrench_array_transformed[5]) * self.wrench_filter_torque_z
        else:
            #print("no bandsperre")
            self.average_wrench_ext_filtered_array[0] = self.average_wrench_array_transformed[0]
            self.average_wrench_ext_filtered_array[1] = self.average_wrench_array_transformed[1]
            self.average_wrench_ext_filtered_array[2] = self.average_wrench_array_transformed[2]
            self.average_wrench_ext_filtered_array[3] = self.average_wrench_array_transformed[3]
            self.average_wrench_ext_filtered_array[4] = self.average_wrench_array_transformed[4]
            self.average_wrench_ext_filtered_array[5] = self.average_wrench_array_transformed[5]
    
        #print(self.average_wrench_ext_filtered_array)
    def control_thread(self):
        """ 
            This thread calculates and publishes the target joint velocity using and admittance controller.
        """
        rate = rospy.Rate(self.publish_rate)
        old_euler = numpy.array([0.0,0.0,0.0])
        while not rospy.is_shutdown():
            #*Get current EE pose in 'base_link' frame
            current_EE_pose = self.group.get_current_pose('wrist_3_link')
            
            current_EE_quaternion = (current_EE_pose.pose.orientation.x,
                                    current_EE_pose.pose.orientation.y,
                                    current_EE_pose.pose.orientation.z,
                                    current_EE_pose.pose.orientation.w)

            # Transform quaternion to euler
            current_EE_euler = euler_from_quaternion(current_EE_quaternion)
            
            current_EE_pose_array = numpy.array([current_EE_pose.pose.position.x,
                                                current_EE_pose.pose.position.y,
                                                current_EE_pose.pose.position.z,
                                                current_EE_euler[0],
                                                current_EE_euler[1],
                                                current_EE_euler[2]])
            
            #*  Calculate desired EE trajectory pose
            # Position
            self.trajectory_EE_pose_array[0] = numpy.array(self.trajectory_EE_pose_array[0] + (self.base_link_desired_velocity[0]/self.publish_rate))
            self.trajectory_EE_pose_array[1] = numpy.array(self.trajectory_EE_pose_array[1] + (self.base_link_desired_velocity[1]/self.publish_rate))
            self.trajectory_EE_pose_array[2] = numpy.array(self.trajectory_EE_pose_array[2] + (self.base_link_desired_velocity[2]/self.publish_rate))
            

            # Orientation
            # Transform cartesian_velocity rotation from 'world' frame to 'tool0' frame
            self.tool0_velocity_rot = self.tf_listener.transformVector3('tool0',self.world_cartesian_velocity_rot)
            
            print("self.tool0_velocity_rot")
            print(self.tool0_velocity_rot)
            
            tool0_velocity_rot_array = numpy.array([self.tool0_velocity_rot.vector.x,
                                                    self.tool0_velocity_rot.vector.y,
                                                    self.tool0_velocity_rot.vector.z])
            
            self.trajectory_EE_pose_array[3] = numpy.array(self.trajectory_EE_pose_array[3] + (tool0_velocity_rot_array[0]/self.publish_rate))
            self.trajectory_EE_pose_array[4] = numpy.array(self.trajectory_EE_pose_array[4] + (tool0_velocity_rot_array[1]/self.publish_rate))
            self.trajectory_EE_pose_array[5] = numpy.array(self.trajectory_EE_pose_array[5] - (tool0_velocity_rot_array[2]/self.publish_rate))
            
            
            #* Set rotation difference to zero
            # Difference from trajectory pose and current pose
            pose_diff = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
            pose_diff = numpy.array(self.trajectory_EE_pose_array  - current_EE_pose_array)
            
            pose_trans_diff = pose_diff[0:3]
            pose_rot_diff = pose_diff[-3:]
            
            
            # Calcuate the correct orientation
            for rot in range(len(pose_rot_diff)):
                if abs(pose_rot_diff[rot]) > 3.14: 
                    pose_rot_diff[rot] = pose_rot_diff[rot] - numpy.sign(pose_rot_diff[rot]) * 6.28
                    
            # print("pose_trans_diff ")
            # print(pose_trans_diff )
            print("pose_rot_diff")
            print(pose_rot_diff)
            
            # For measurements------------------------------------------------------------------------------------------
            self.delta_pos_msg.data = pose_trans_diff
            self.delta_ori_msg.data = pose_rot_diff

            self.delta_pos_publisher.publish(self.delta_pos_msg)
            self.delta_ori_publisher.publish(self.delta_ori_msg)
            # For measurements------------------------------------------------------------------------------------------
        
            #* Compute the wrench difference
            for wrench in range(len(self.average_wrench_ext_filtered_array )):
                if self.average_wrench_ext_filtered_array[wrench] != 0.0:
                    
                    self.wrench_diff[wrench] = (-1) * numpy.sign(self.average_wrench_ext_filtered_array[wrench]) * (self.wrench_contact[wrench] - numpy.linalg.norm(self.average_wrench_ext_filtered_array[wrench]))
                else:
                    self.wrench_diff[wrench] = 0.0
            
            # * Calculate velocity from wrench difference and admittance in 'wrist_3_link' frame
            self.admittance_velocity[0] = (self.wrench_diff[0]  + (pose_trans_diff[0] * self.Kp_trans_x * self.A_trans_x))/ self.A_trans_x
            self.admittance_velocity[1] = (self.wrench_diff[1] + (pose_trans_diff[1] * self.Kp_trans_y * self.A_trans_y)) / self.A_trans_y 
            self.admittance_velocity[2] = (self.wrench_diff[2] + (pose_trans_diff[2] * self.Kp_trans_z * self.A_trans_z)) / self.A_trans_z 
            
            # self.admittance_velocity[3] = ((pose_rot_diff[0] * self.Kp_rot_x * self.A_rot_x )) / self.A_rot_x 
            # self.admittance_velocity[4] = ((pose_rot_diff[1] * self.Kp_rot_y * self.A_rot_y)) / self.A_rot_y
            # self.admittance_velocity[5] = ((pose_rot_diff[2] * self.Kp_rot_z * self.A_rot_z )) / self.A_rot_z    
            
            self.admittance_velocity[3] = (self.wrench_diff[3] + (pose_rot_diff[0] * self.Kp_rot_x * self.A_rot_x )) / self.A_rot_x 
            self.admittance_velocity[4] = (self.wrench_diff[4] + (pose_rot_diff[1] * self.Kp_rot_y * self.A_rot_y)) / self.A_rot_y
            self.admittance_velocity[5] = (self.wrench_diff[5] +  (pose_rot_diff[2] * self.Kp_rot_z * self.A_rot_z )) / self.A_rot_z

            # * Add the desired_velocity in 'base_link' frame and admittance velocity in 'base_link' frame
            self.target_cartesian_velocity[0] = self.base_link_desired_velocity[0] + self.admittance_velocity[0]
            self.target_cartesian_velocity[1] = self.base_link_desired_velocity[1] + self.admittance_velocity[1]
            self.target_cartesian_velocity[2] = self.base_link_desired_velocity[2] + self.admittance_velocity[2]
            self.target_cartesian_velocity[3] = self.base_link_desired_velocity[3] + self.admittance_velocity[3]
            self.target_cartesian_velocity[4] = self.base_link_desired_velocity[4] + self.admittance_velocity[4]
            self.target_cartesian_velocity[5] = self.base_link_desired_velocity[5] + self.admittance_velocity[5]

            # # * Check self.target_cartesian_velocity for the max velocity limits
            # # Calculate the norm of target_cartesian_velocity (trans and rot)
            # target_cartesian_trans_velocity_norm = math.sqrt(pow(self.target_cartesian_velocity[0],2) + pow(self.target_cartesian_velocity[1],2) + pow(self.target_cartesian_velocity[2],2))
            
            # target_cartesian_rot_velocity_norm = math.sqrt(pow(self.target_cartesian_velocity[3],2) + pow(self.target_cartesian_velocity[4],2) + pow(self.target_cartesian_velocity[5],2))
            
            # #  Check for cartesian velocity max limit and set to max limit, if max limit is exceeded
            # if target_cartesian_trans_velocity_norm > self.cartesian_velocity_trans_max_limit:
            #     for i in range(3):
            #         self.target_cartesian_velocity[i] = (self.target_cartesian_velocity[i]/target_cartesian_trans_velocity_norm) * self.cartesian_velocity_trans_max_limit
                    
            # if target_cartesian_rot_velocity_norm > self.cartesian_velocity_rot_max_limit:
            #     for i in range(3,6):
            #         self.target_cartesian_velocity[i] = (self.target_cartesian_velocity[i]/target_cartesian_rot_velocity_norm) * self.cartesian_velocity_rot_max_limit
                    
            # * Get the current joint states 
            self.current_joint_states_array = self.group.get_current_joint_values() 
            
            # * Calculate the jacobian-matrix
            self.jacobian = self.group.get_jacobian_matrix(self.current_joint_states_array)
            
# * Singulartiy avoidance: OLMM-----------------------------------------------------------------------------------------
            # QIU, Changwu; CAO, Qixin; MIAO, Shouhong. An on-line task modification method for singularity avoidance of robot manipulators. Robotica, 2009, 27. Jg., Nr. 4, S. 539-546.
            
            # Do a singular value decomposition of the jacobian
            u,s,v = numpy.linalg.svd(self.jacobian,full_matrices=True)
            
            sigma_min = min(s) 

            # For measurements------------------------------------------------------------------------------------------
            self.sigma_msg.data = sigma_min 
            self.sigma_publisher.publish(self.sigma_msg)

            # For measurements------------------------------------------------------------------------------------------

            
            for sigma in range(len(s)):
                
                    # If a singularity is detected
                    if s[sigma] < self.singularity_entry_threshold:
                        if self.bool_singularity == False:
                            self.bool_singularity = True
                            rospy.loginfo("Activate OLMM")
                            
                        #print(sigma_min)
                        if s[sigma] < self.singularity_stop_threshold:
                            rospy.loginfo("Singularity stop activated! Sigma %f is smaller then threshol %f",sigma_min,self.singularity_min_threshold)
                            self.singularity_stop = True
                        else:
                            
                            if s[sigma] > self.singularity_min_threshold:
                                self.singularity_avoidance_velocity = numpy.dot(self.scalar_adjusting_function(s[sigma]),numpy.dot(numpy.dot(u[:,sigma],self.target_cartesian_velocity),u[:,sigma]))

                            else:
                                self.singularity_avoidance_velocity = numpy.dot(1,numpy.dot(numpy.dot(u[:,sigma],self.target_cartesian_velocity),u[:,sigma]))
            # if  self.singularity_entry_threshold < sigma_min < self.singularity_exit_threshold and self.bool_singularity == True:
            #     self.singularity_avoidance_velocity = numpy.dot(self.scalar_adjusting_function(s[sigma]),numpy.dot(numpy.dot(u[:,sigma],self.target_cartesian_velocity),u[:,sigma]))

                
            # If a singularity was detected but the singularity_counter is null, 
            if sigma_min > self.singularity_entry_threshold and self.bool_singularity == True:

                #* Reset the bool_singularity and acitivate the bool_reduce_singularity_offset
                self.bool_singularity = False
                self.bool_reduce_singularity_offset = True
                rospy.loginfo("Deactivate OLMM")
                #* Reset the singularity velocity world to zero
                self.singularity_avoidance_velocity = [0.0,0.0,0.0,0.0,0.0,0.0]
                
            #* Transform the singularity avoidance velocity into 'world' frame
            self.singularity_velocity_world = self.transform_vector('base_link','world',self.singularity_avoidance_velocity)
            #* Publish the singularity avoidance velocity to rosmaster
            self.singularity_velocity_msg.singularity_stop = self.singularity_stop
            self.singularity_velocity_msg.singularity_velocity = self.singularity_velocity_world
            self.singularity_velocity_pub.publish(self.singularity_velocity_msg)  
                
            #* Check for trajectory differences:
            if self.bool_reduce_singularity_offset == True:
                
                print("numpy.linalg.norm(pose_trans_diff)")
                print(numpy.linalg.norm(pose_trans_diff))
                print("numpy.linalg.norm(pose_rot_diff)")
                print(numpy.linalg.norm(pose_rot_diff))
                
                # self.reconstruct_trajektory[0] = -(pose_trans_diff[0]  * self.Kp_trans_x * self.A_trans_x)/ self.A_trans_x
                # self.reconstruct_trajektory[1] = -(pose_trans_diff[1]  * self.Kp_trans_y * self.A_trans_y)/ self.A_trans_y
                # self.reconstruct_trajektory[2] = -(pose_trans_diff[2]  * self.Kp_trans_z * self.A_trans_z)/ self.A_trans_z
                
                
                # self.reconstruct_trajektory[3] = -(pose_rot_diff[0]  * self.Kp_rot_x * self.A_rot_x ) / self.A_rot_x 
                # self.reconstruct_trajektory[4] = -(pose_rot_diff[1]  * self.Kp_rot_y * self.A_rot_y ) / self.A_rot_y 
                # self.reconstruct_trajektory[5] = -(pose_rot_diff[2]  * self.Kp_rot_z * self.A_rot_z ) / self.A_rot_z 
                
                #* Transform the singularity avoidance velocity into 'world' frame
                self.singularity_velocity_world = self.transform_vector('base_link','world',self.reconstruct_trajektory)
                #* Publish the singularity avoidance velocity to rosmaster
                self.singularity_velocity_msg.singularity_stop = self.singularity_stop
                self.singularity_velocity_msg.singularity_velocity = self.singularity_velocity_world
                self.singularity_velocity_pub.publish(self.singularity_velocity_msg)  
                
                if numpy.linalg.norm(pose_trans_diff) < self.singularity_trans_offset_accuracy and numpy.linalg.norm(pose_rot_diff) < self.singularity_rot_offset_accuracy:
                    
                    self.bool_reduce_singularity_offset = False
                    
                    self.singularity_velocity_world = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0]) 
                    
                    rospy.loginfo("Endeffector trans difference %f [m] is samller then threshold %f [m]",numpy.linalg.norm(pose_trans_diff),self.singularity_trans_offset_accuracy)
                    rospy.loginfo("Endeffector rot difference %f [rad] is samller then threshold %f [rad]",numpy.linalg.norm(pose_rot_diff),self.singularity_rot_offset_accuracy) 
                
                
            
            # For measurements------------------------------------------------------------------------------------------
            self.singularity_msg.data = self.bool_singularity 
            self.sigularity_publisher.publish(self.singularity_msg)
            
            
            
            self.offset_msg.data = self.bool_reduce_singularity_offset
            self.offset_publisher.publish(self.offset_msg)

            # For measurements------------------------------------------------------------------------------------------
# * Singulartiy avoidance: OLMM-----------------------------------------------------------------------------------------
            #* Subract the target cartesian velocity with the singularity avoidance velocity
            if self.bool_singularity == True and self.singularity_stop == False:
                self.target_cartesian_velocity = self.target_cartesian_velocity - self.singularity_avoidance_velocity
                
            elif (self.bool_singularity == True and self.singularity_stop == True) or self.workspace_violation == True:
                self.target_cartesian_velocity = [0.0,0.0,0.0,0.0,0.0,0.0]
            
            # * Calculate the inverse of the jacobian-matrix
            self.inverse_jacobian = numpy.linalg.inv(self.jacobian)
            
            # * Calculate the target joint velocity with the inverse jacobian-matrix and the target cartesain velociy
            self.target_joint_velocity.data = self.inverse_jacobian.dot(self.target_cartesian_velocity)
            
            # * Publish the target_joint_velocity
            self.joint_velocity_pub.publish(self.target_joint_velocity)
            
            # * Sleep for publish_rate
            rate.sleep()

    def shutdown(self):
        """ 
            This function is called by rospy.on_shutdown!
        """
        print("Shutdown amittcance controller:")
        print("Shutdown publisher joint velocity!")
        self.joint_velocity_pub.publish(self.shutdown_joint_velocity)
        print("Unregister from joint_velocity_pub!")
        self.joint_velocity_pub.unregister()
        print("Unregister from wrench_ext_sub!")
        self.wrench_ext_sub.unregister()
        
    
if __name__ == '__main__':
    ur_admittance_controller()
    