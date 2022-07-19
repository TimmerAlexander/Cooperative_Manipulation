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
import sys
import numpy, math
import rospy
import tf
import tf2_ros
import moveit_commander
from geometry_msgs.msg import WrenchStamped, Vector3Stamped, Twist, TransformStamped
from std_msgs.msg import Float64MultiArray


class ur_admittance_controller():
    
    def config(self):
        # Stiffness gains
        self.P_trans_x = 5
        self.P_trans_y = 5
        self.P_trans_z = 5
        self.P_rot_x = 5
        self.P_rot_y = 5
        self.P_rot_z = 5
        # Damping gains
        self.D_trans_x = 5
        self.D_trans_y = 5
        self.D_trans_z = 5
        self.D_rot_x = 5
        self.D_rot_y = 5
        self.D_rot_z = 5
        
        self.world_z_vector = numpy.array([0.0,0.0,1.0])
        self.wrist_link_3_rot_axis = numpy.array([0.0,0.0,0.0])
        
        self.wrist_link_3_desired_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        
        
        # Min and max limits for the cartesian velocity (trans/rot) [m/s]
        self.cartesian_velocity_trans_min_limit = 0.0009
        self.cartesian_velocity_trans_max_limit = 0.1
        self.cartesian_velocity_rot_min_limit = 0.001
        self.cartesian_velocity_rot_max_limit = 0.1
        # Control thread publish rate [Hz]
        self.publish_rate = 100
        # Initialize wrench_ext_filtered
        self.wrench_force_filtered_x = 0.0
        self.wrench_force_filtered_y = 0.0
        self.wrench_force_filtered_z = 0.0
        # Wrench filter treshold (when not cooperative_manipulation: 0.05 - 0.1(cmd_vel: 0.8 or higher))
        self.wrench_filter = 0.1
        # 
        self.force_filter_factor = 140.0
        self.torque_filter_trans_factor = 10.9
        self.torque_filter_factor = 10.9

        
        
        self.force_filter_factor_array = numpy.array([0.0,0.0,0.0])
        self.torque_filter_factor_array = numpy.array([0.0,0.0,0.0,])

        self.average_filter_list_force_x = []
        self.average_filter_list_force_y = []
        self.average_filter_list_force_z = []
        self.average_filter_list_torque_x = []
        self.average_filter_list_torque_y = []
        self.average_filter_list_torque_z = []
        self.average_filter_list_length = 100

        self.average_force_x = 0.0
        self.average_force_y = 0.0
        self.average_force_z = 0.0
        self.average_torque_x = 0.0
        self.average_torque_y = 0.0
        self.average_torque_z = 0.0
        # Initialize trajectory velocity for object rotation
        self.world_trajectory_velocity = numpy.array([0.0,0.0,0.0])
        # The gripper offset
        self.r16e_gripper_offset_trans_z = 0.16163
        # * Initialize the needed velocity data types:
        # Initialize desired velocity transformed form 'wrorld' frame to 'base_link' frame (xdot_desired_baselink)
        self.base_link_desired_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.world_cartesian_velocity_trans = Vector3Stamped()
        self.world_cartesian_velocity_rot = Vector3Stamped()
        self.wrist_link_3_cartesian_desired_velocity_trans  = Vector3Stamped()
        self.wrist_link_3_cartesian_desired_velocity_rot  = Vector3Stamped()
        
        
        self.world_rot_velocity_cross_product_vector = Vector3Stamped()
        # Initialize velocity from admittance (xdot_a_wrist_3_link)
        self.admittance_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Initialize velocity from admittance 'wrist_3_link' frame to 'base_link' frame  (xdot_a_baselink)
        self.admittance_velocity_transformed  = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Initialize target cartesian velocity array (xdot_target_baselink)
        self.target_cartesian_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Declare target joint velocity msgs (qdot_target_baselink) (unit: [radian/s])
        self.target_joint_velocity = Float64MultiArray()
        # Initialize velocity (xdot_desired_wrist_3_link)
        self.velocity_transformed = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.wrist_3_link_cartesian_velocity_trans = Vector3Stamped()
        self.wrist_3_link_cartesian_velocity_rot = Vector3Stamped()
        # Initialize shutdown joint velocity, called on shutdown 
        self.shutdown_joint_velocity = Float64MultiArray()
        self.shutdown_joint_velocity.data = [0.0,0.0,0.0,0.0,0.0,0.0]
        # Declare the wrench variables
        self.wrench_ext_filtered = WrenchStamped()
        self.wrench_difference = WrenchStamped()        
        self.wrench_ext_filtered_trans_array = numpy.array([])
        self.wrench_ext_filtered_rot_array = numpy.array([])
        # Singulariy avoidance: OLMM
        self.sigma_limit = 0.05
        self.adjusting_scalar = 0.0
        self.singular_velocity = numpy.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.singular_velocity_msg = Float64MultiArray()
        
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
        rospy.loginfo("Wait for transformation 'world' to 'base_link'.")
        self.tf_listener.waitForTransform("world","base_link", rospy.Time(), rospy.Duration(5.0))
        
        # Initialize the 'ur16e_gripper' frame in tf tree
        self.set_gripper_offset()
        # Wait for transformations from 'world' to 'ur16e_gripper' and 'world' to 'panda_gripper'
        rospy.loginfo("Wait for transformation 'world' to 'ur16e_gripper'.")
        self.tf_listener.waitForTransform("world","ur16e_gripper", rospy.Time(), rospy.Duration(10.0))
        rospy.loginfo("Wait for transformation 'world' to '/panda/panda_link8'.")
        self.tf_listener.waitForTransform("world","panda/panda_link8", rospy.Time(), rospy.Duration(10.0))

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
        # Publish final joint velocity to "/ur/ur_admittance_controller/command"
        self.joint_velocity_pub = rospy.Publisher(
            "/" + self.namespace + "/ur_admittance_controller/command",
            Float64MultiArray,
            queue_size=1)
        
        # Publish singularity velocity
        self.singularity_velocity_pub = rospy.Publisher(
            "/cooperative_manipulation/singularity_velocity",
            Float64MultiArray,
            queue_size=1)
        
        # Publish final joint velocity to "/ur/ur_admittance_controller/command"
        self.wrench_filter_pub = rospy.Publisher(
            "/" + self.namespace + "/ur_admittance_controller/wrench_filter",
            WrenchStamped,
            queue_size=1)
        
        # * Initialize subscriber:
        # Subscriber to "/ur/wrench"
        self.wrench_ext_sub = rospy.Subscriber(
            "/" + self.namespace + "/ft_sensor/raw",
            WrenchStamped,
            self.wrench_callback,queue_size=1)
        
        # Subscriber to "/ur/cooperative_manipulation/cartesian_velocity_command"
        self.cartesian_velocity_command_sub = rospy.Subscriber(
            "/cooperative_manipulation/cartesian_velocity_command",
            Twist,
            self.cartesian_velocity_command_callback,queue_size=1)
        
        # * Initialize tf TransformListener
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("wrist_3_link","base_link", rospy.Time(), rospy.Duration(4.0))

        self.listener.waitForTransform("world","base_link", rospy.Time(), rospy.Duration(4.0))


        self.tf_time = self.listener.getLatestCommonTime("/world", "/wrist_3_link")
        self.current_position, self.current_quaternion = self.listener.lookupTransform("/world", "/wrist_3_link", self.tf_time)
        
        # Wait for messages to be populated before proceeding
        rospy.wait_for_message("/" + self.namespace + "/ft_sensor/raw",WrenchStamped,timeout=5.0)
        
        
        rospy.loginfo("Recieved messages; Launch ur16e Admittance control.")
        
        
        # * Run control_thread
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
        static_gripper_offset.transform.translation.z = self.r16e_gripper_offset_trans_z
        static_gripper_offset.transform.rotation.x = 0.0
        static_gripper_offset.transform.rotation.y = 0.0
        static_gripper_offset.transform.rotation.z = 0.0
        static_gripper_offset.transform.rotation.w = 1.0

        self.brodacaster.sendTransform(static_gripper_offset)
        
    def scalar_adjusting_function(self,current_sigma):
        """Compute the adjusting scalar for the OLMM. 3 varaitions to calculate the adjusting scalar are presented.

        Args:
            current_sigma (float): The current singular value

        Returns:
            float: The adjusting_scalar
        """
        # 1.
        # adjusting_scalar = (1-(current_sigma/self.sigma_limit))
        # # 2.
        # adjusting_scalar = (1-(current_sigma/self.sigma_limit)**(1/2))
        # 3.
        adjusting_scalar = (1-(current_sigma/self.sigma_limit)**(3/2))
        
        return adjusting_scalar
           
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
        # print("desired_velocity")
        # print(desired_velocity)
        
        # ToDo ------------------------------------------------------------------
        
        # Get current time stamp
        now = rospy.Time()
        
        # print("desired_velocity")
        # print(desired_velocity)
    
        # Calculate the trajectory velocity of the manipulator for a rotation of the object
        # Get ur16e_current_position, ur16e_current_quaternion of the 'wrist_3_link' in frame in the 'world' frame 
        ur16e_tf_time = self.tf_listener.getLatestCommonTime("/world", "/wrist_3_link")
        ur16e_current_position, ur16e_current_quaternion = self.tf_listener.lookupTransform("/world", "/wrist_3_link", ur16e_tf_time)

        # Get ur16e_current_position, ur16e_current_quaternion of the 'ur16e_gripper' in frame in the 'world' frame 
        ur16e_tf_time = self.tf_listener.getLatestCommonTime("/world", "/ur16e_gripper")
        ur16e_gripper_position, ur16e_gripper_quaternion = self.tf_listener.lookupTransform("/world", "/ur16e_gripper", ur16e_tf_time)

        # Get self.panda_current_position, self.panda_current_quaternion of the '/panda/panda_link8' frame in the 'world' frame 
        panda_tf_time = self.tf_listener.getLatestCommonTime("/world", "/panda/panda_link8")
        panda_gripper_position, panda_gripper_quaternion = self.tf_listener.lookupTransform("/world", "/panda/panda_gripper", panda_tf_time)

        # print("self.ur16e_current_position, self.ur16e_current_quaternion")
        # print(self.ur16e_current_position, self.ur16e_current_quaternion)
        # print("self.panda_current_position, self.panda_current_quaternion")
        # print(self.panda_position, self.panda_current_quaternion)
        self.world_trajectory_velocity = [0.0,0.0,0.0]
        # Object rotation around x axis 
        if desired_velocity.angular.x != 0.0:
            ur16e_current_position_x = numpy.array([
                0.0,
                ur16e_current_position[1],
                ur16e_current_position[2]
                ])
            
            self.robot_distance_x = numpy.array([
                0.0,
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


        # ToDo ------------------------------------------------------------------
        
        # Get current time stamp
        now = rospy.Time()

        # Converse cartesian_velocity translation to vector3
        self.world_cartesian_velocity_trans.header.frame_id = 'world'
        self.world_cartesian_velocity_trans.header.stamp = now
        self.world_cartesian_velocity_trans.vector.x = desired_velocity.linear.x + self.world_trajectory_velocity[0]
        self.world_cartesian_velocity_trans.vector.y = desired_velocity.linear.y + self.world_trajectory_velocity[1]
        self.world_cartesian_velocity_trans.vector.z = desired_velocity.linear.z + self.world_trajectory_velocity[2]
        
        # Transform cartesian_velocity translation from 'world' frame to 'base_link' frame
        self.base_link_cartesian_desired_velocity_trans = self.listener.transformVector3('base_link',self.world_cartesian_velocity_trans)
        
        
        # Converse cartesian_velocity rotation to vector3
        self.world_cartesian_velocity_rot.header.frame_id = 'world'
        self.world_cartesian_velocity_rot.header.stamp = now
        self.world_cartesian_velocity_rot.vector.x = desired_velocity.angular.x
        self.world_cartesian_velocity_rot.vector.y = desired_velocity.angular.y
        self.world_cartesian_velocity_rot.vector.z = desired_velocity.angular.z
        
        # Transform cartesian_velocity rotation from 'world' frame to 'base_link' frame
        self.base_link_cartesian_desired_velocity_rot = self.listener.transformVector3('base_link',self.    world_cartesian_velocity_rot)
        
        # Converse cartesian_velocity from vector3 to numpy.array
        self.base_link_desired_velocity = [
            self.base_link_cartesian_desired_velocity_trans.vector.x,
            self.base_link_cartesian_desired_velocity_trans.vector.y,
            self.base_link_cartesian_desired_velocity_trans.vector.z,
            self.base_link_cartesian_desired_velocity_rot.vector.x,
            self.base_link_cartesian_desired_velocity_rot.vector.y,
            self.base_link_cartesian_desired_velocity_rot.vector.z
            ]
        

        
        self.world_desired_velocity = [
            self.world_cartesian_velocity_trans.vector.x,
            self.world_cartesian_velocity_trans.vector.y,
            self.world_cartesian_velocity_trans.vector.z
            ]
        

        
        # Transform cartesian_velocity rotation from 'world' frame to 'wrist_3_link' frame
        self.wrist_link_3_cartesian_desired_velocity_trans = self.listener.transformVector3('wrist_3_link',self.world_cartesian_velocity_trans)
        
        # Transform cartesian_velocity rotation from 'world' frame to 'wrist_3_link' frame
        self.wrist_link_3_cartesian_desired_velocity_rot = self.listener.transformVector3('wrist_3_link',self.world_cartesian_velocity_trans)
        
        self.wrist_link_3_desired_velocity = [
            self.wrist_link_3_cartesian_desired_velocity_trans.vector.x,
            self.wrist_link_3_cartesian_desired_velocity_trans.vector.y,
            self.wrist_link_3_cartesian_desired_velocity_trans.vector.z,
            self.wrist_link_3_cartesian_desired_velocity_rot.vector.x,
            self.wrist_link_3_cartesian_desired_velocity_rot.vector.y,
            self.wrist_link_3_cartesian_desired_velocity_rot.vector.z
            ]
        
        
        # print("self.wrist_link_3_desired_velocity")
        # print(self.wrist_link_3_desired_velocity )
        
        self.world_rot_velocity_cross_product_array = numpy.cross(self.world_desired_velocity,self.world_z_vector)
        
        # print("self.world_rot_velocity_cross_product_array")
        # print(self.world_rot_velocity_cross_product_array)
        
        # Converse self.world_rot_velocity_cross_product_array rotation to vector3
        self.world_rot_velocity_cross_product_vector.header.frame_id = 'world'
        self.world_rot_velocity_cross_product_vector.header.stamp = now
        self.world_rot_velocity_cross_product_vector.vector.x = self.world_rot_velocity_cross_product_array[0]
        self.world_rot_velocity_cross_product_vector.vector.y = self.world_rot_velocity_cross_product_array[1]
        self.world_rot_velocity_cross_product_vector.vector.z = self.world_rot_velocity_cross_product_array[2]
        
        
        # Transform cartesian_velocity rotation from 'world' frame to 'wrist_3_link' frame
        self.wrist_link_3_rot_axis_vector = self.listener.transformVector3('wrist_3_link',self.world_rot_velocity_cross_product_vector)
        
        self.wrist_link_3_rot_axis = [
            self.wrist_link_3_rot_axis_vector.vector.x,
            self.wrist_link_3_rot_axis_vector.vector.y,
            self.wrist_link_3_rot_axis_vector.vector.z
            ]
        
        # print("self.wrist_link_3_rot_axis")
        # print(self.wrist_link_3_rot_axis)
    
    def wrench_callback(self,wrench_ext):
        """ 
            Get external wrench in wrist_link_3.
            
            Send example wrench:
            rostopic pub  /ur/wrench geometry_msgs/WrenchStamped '{header: {stamp: now, frame_id: base_link}, wrench:{force: {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}'
        """
    
        # print("wrench_ext.wrench before filtered:")
        # print(wrench_ext.wrench)
        
        
        for f in range(3):
            if self.wrist_link_3_desired_velocity[f] != 0.0:
                # self.force_filter_factor = 141
                self.force_filter_factor_array[f] = self.force_filter_factor * numpy.abs(self.wrist_link_3_desired_velocity[f])   
            else:
                self.force_filter_factor_array[f] = 0.0
            
            if self.wrist_link_3_rot_axis[f]!= 0.0:
                # self.torque_filter_trans_factor = 10.9
                self.torque_filter_factor_array[f] = self.torque_filter_trans_factor * numpy.abs(self.wrist_link_3_rot_axis[f])
                
                if f == 3:
                    self.torque_filter_factor_array[f] = self.torque_filter_factor_z * numpy.abs(self.wrist_link_3_rot_axis[f])
            else:
                self.torque_filter_factor_array[f] = 0.0


        # for f in range(3,6):
        #     if self.desired_velocity_wrist_link_3[f] != 0.0:
        #         self.torque_filter_factor_array[f] = self.torque_filter_factor_array[f] + self.torque_filter_factor * numpy.abs(self.desired_velocity_wrist_link_3[f])
        #     else:
        #         self.ftorque_filter_factor_array[f] = 0.0

        # print("self.force_filter_factor_array:")
        # print(self.force_filter_factor_array)

        # print("self.torque_filter_factor_array:")
        # print(self.torque_filter_factor_array)

        # * Average filter
        # Fill the empty lists with wrench values
        if len(self.average_filter_list_force_x) < self.average_filter_list_length:
            # 2. Add the new wrench to the list 
            self.average_filter_list_force_x.append(wrench_ext.wrench.force.x - numpy.sign(wrench_ext.wrench.force.x) * self.force_filter_factor_array[0])
            self.average_filter_list_force_y.append(wrench_ext.wrench.force.y - numpy.sign(wrench_ext.wrench.force.y) *  self.force_filter_factor_array[1])
            self.average_filter_list_force_z.append(wrench_ext.wrench.force.z - numpy.sign(wrench_ext.wrench.force.z) *  self.force_filter_factor_array[2])
            self.average_filter_list_torque_x.append(wrench_ext.wrench.torque.x - numpy.sign(wrench_ext.wrench.torque.x) * self.torque_filter_factor_array[0])
            self.average_filter_list_torque_y.append(wrench_ext.wrench.torque.y - numpy.sign(wrench_ext.wrench.torque.y) *  self.torque_filter_factor_array[1])
            self.average_filter_list_torque_z.append(wrench_ext.wrench.torque.z - numpy.sign(wrench_ext.wrench.torque.z) *  self.torque_filter_factor_array[2])
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
            self.average_filter_list_force_x.append(wrench_ext.wrench.force.x - numpy.sign(wrench_ext.wrench.force.x) * self.force_filter_factor_array[0])
            self.average_filter_list_force_y.append(wrench_ext.wrench.force.y - numpy.sign(wrench_ext.wrench.force.y) *  self.force_filter_factor_array[1])
            self.average_filter_list_force_z.append(wrench_ext.wrench.force.z - numpy.sign(wrench_ext.wrench.force.z) *  self.force_filter_factor_array[2])
            self.average_filter_list_torque_x.append(wrench_ext.wrench.torque.x - numpy.sign(wrench_ext.wrench.torque.x) * self.torque_filter_factor_array[0])
            self.average_filter_list_torque_y.append(wrench_ext.wrench.torque.y - numpy.sign(wrench_ext.wrench.torque.y) *  self.torque_filter_factor_array[1])
            self.average_filter_list_torque_z.append(wrench_ext.wrench.torque.z - numpy.sign(wrench_ext.wrench.torque.z) *  self.torque_filter_factor_array[2])
            # 3. Calculate the average 
            self.average_force_x = sum(self.average_filter_list_force_x)/self.average_filter_list_length
            self.average_force_y = sum(self.average_filter_list_force_y)/self.average_filter_list_length
            self.average_force_z = sum(self.average_filter_list_force_z)/self.average_filter_list_length
            self.average_torque_x = sum(self.average_filter_list_torque_x)/self.average_filter_list_length
            self.average_torque_y = sum(self.average_filter_list_torque_y)/self.average_filter_list_length
            self.average_torque_z = sum(self.average_filter_list_torque_z)/self.average_filter_list_length
            
            
        # print("self.average_force_x:")
        # print(self.average_force_x)
        # print("self.average_force_y:")
        # print(self.average_force_y)
        # print("self.average_force_z:")
        # print(self.average_force_z)
        
        # print("self.average_torque_x:")
        # print(self.average_torque_x)
        # print("self.average_torque_y:")
        # print(self.average_torque_y)
        # print("self.average_torque_z:")
        # print(self.average_torque_z)
        
        # * Band-passfilter
        if numpy.abs(self.average_force_x) < self.wrench_filter:
            self.wrench_ext_filtered.wrench.force.x = 0.0
        else: 
            self.wrench_ext_filtered.wrench.force.x = self.average_force_x - numpy.sign(self.average_force_x) * self.wrench_filter
            
        if(numpy.abs(self.average_force_y) < self.wrench_filter):
            self.wrench_ext_filtered.wrench.force.y = 0.0
        else: 
            self.wrench_ext_filtered.wrench.force.y = self.average_force_y - numpy.sign(self.average_force_y) * self.wrench_filter

        if(numpy.abs(self.average_force_z) < self.wrench_filter):
            self.wrench_ext_filtered.wrench.force.z = 0.0
        else: 
           self.wrench_ext_filtered.wrench.force.z = self.average_force_z - numpy.sign(self.average_force_z) * self.wrench_filter
           
        if(numpy.abs(self.average_torque_x) < self.wrench_filter):
            self.wrench_ext_filtered.wrench.torque.x = 0.0
        else: 
            self.wrench_ext_filtered.wrench.torque.x = self.average_torque_x - numpy.sign(self.average_torque_x) * self.wrench_filter
            
        if(numpy.abs(self.average_torque_y) < self.wrench_filter):
            self.wrench_ext_filtered.wrench.torque.y = 0.0
        else: 
            self.wrench_ext_filtered.wrench.torque.y = self.average_torque_y - numpy.sign(self.average_torque_y) * self.wrench_filter
        if(numpy.abs(self.average_torque_z) < self.wrench_filter):
            self.wrench_ext_filtered.wrench.torque.z = 0.0
        else: 
            self.wrench_ext_filtered.wrench.torque.z = self.average_torque_z - numpy.sign(self.average_torque_z) * self.wrench_filter
            
            
        # print("self.wrench_ext_filtered")
        # print(self.wrench_ext_filtered)
        self.wrench_filter_pub.publish(self.wrench_ext_filtered) 
    
    
    def transform_velocity(self,cartesian_velocity):
        """ 
            Transform the cartesian velocity from the 'wrist_3_link' frame to the 'base_link' frame.
        """
        # Get current time stamp
        now = rospy.Time()

        # Converse cartesian_velocity translation from numpy.array to vector3
        self.wrist_3_link_cartesian_velocity_trans.header.frame_id = 'wrist_3_link'
        self.wrist_3_link_cartesian_velocity_trans.header.stamp = now
        self.wrist_3_link_cartesian_velocity_trans.vector.x = cartesian_velocity[0]
        self.wrist_3_link_cartesian_velocity_trans.vector.y = cartesian_velocity[1]
        self.wrist_3_link_cartesian_velocity_trans.vector.z = cartesian_velocity[2]
        
        # Transform cartesian_velocity translation from 'wrist_3_link' frame to 'base_link' frame
        self.base_link_cartesian_velocity_trans = self.listener.transformVector3('base_link',self.wrist_3_link_cartesian_velocity_trans)
        
        # Converse cartesian_velocity rotation from numpy.array to vector3
        self.wrist_3_link_cartesian_velocity_rot.header.frame_id = 'wrist_3_link'
        self.wrist_3_link_cartesian_velocity_rot.header.stamp = now
        self.wrist_3_link_cartesian_velocity_rot.vector.x = cartesian_velocity[3]
        self.wrist_3_link_cartesian_velocity_rot.vector.y = cartesian_velocity[4]
        self.wrist_3_link_cartesian_velocity_rot.vector.z = cartesian_velocity[5]
        
        # Transform cartesian_velocity rotation from 'wrist_3_link' frame to 'base_link' frame
        self.base_link_cartesian_velocity_rot = self.listener.transformVector3('base_link',self.    wrist_3_link_cartesian_velocity_rot)
        
        # Converse cartesian_velocity from vector3 to numpy.array
        self.velocity_transformed = numpy.array([
            self.base_link_cartesian_velocity_trans.vector.x,
            self.base_link_cartesian_velocity_trans.vector.y,
            self.base_link_cartesian_velocity_trans.vector.z,
            self.base_link_cartesian_velocity_rot.vector.x,
            self.base_link_cartesian_velocity_rot.vector.y,
            self.base_link_cartesian_velocity_rot.vector.z
            ])
        
        return self.velocity_transformed

    def control_thread(self):
        """ 
            This thread calculates and publishes the target joint velocity using and admittance controller.
        """
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            
            # * Calculate velocity from wrench difference and admittance in 'wrist_3_link' frame
            self.admittance_velocity[0] = numpy.sign(self.wrench_ext_filtered.wrench.force.x) * (numpy.abs(self.wrench_ext_filtered.wrench.force.x) * pow((self.P_trans_x * (numpy.abs(self.wrench_ext_filtered.wrench.force.x)/self.publish_rate) + self.D_trans_x),-1))
            
            self.admittance_velocity[1] = numpy.sign(self.wrench_ext_filtered.wrench.force.y) * (numpy.abs(self.wrench_ext_filtered.wrench.force.y) * pow((self.P_trans_y * (numpy.abs(self.wrench_ext_filtered.wrench.force.y)/self.publish_rate) + self.D_trans_y),-1))         
            
            self.admittance_velocity[2] = numpy.sign(self.wrench_ext_filtered.wrench.force.z) * (numpy.abs(self.wrench_ext_filtered.wrench.force.z) * pow((self.P_trans_z * (numpy.abs(self.wrench_ext_filtered.wrench.force.z)/self.publish_rate) + self.D_trans_z),-1))     
            
            self.admittance_velocity[3] = numpy.sign(self.wrench_ext_filtered.wrench.torque.x) * (numpy.abs(self.wrench_ext_filtered.wrench.torque.x) * pow((self.P_rot_x * (numpy.abs(self.wrench_ext_filtered.wrench.torque.x)/self.publish_rate) + self.D_rot_x),-1))    
                                                                                
            self.admittance_velocity[4] = numpy.sign(self.wrench_ext_filtered.wrench.torque.y) * (numpy.abs(self.wrench_ext_filtered.wrench.torque.y) * pow((self.P_rot_y * (numpy.abs(self.wrench_ext_filtered.wrench.torque.y)/self.publish_rate) + self.D_rot_y),-1))
            
            self.admittance_velocity[5] = numpy.sign(self.wrench_ext_filtered.wrench.torque.z) * (numpy.abs(self.wrench_ext_filtered.wrench.torque.z) * pow((self.P_rot_z * (numpy.abs(self.wrench_ext_filtered.wrench.torque.z)/self.publish_rate) + self.D_rot_z),-1))
            
            self.admittance_velocity_transformed = self.transform_velocity(self.admittance_velocity)
            
            # print("self.admittance_velocity_transformed")
            # print(self.admittance_velocity_transformed)
            
            # print("self.base_link_desired_velocity")
            # print(self.base_link_desired_velocity)
            
            # * Add the desired_velocity in 'base_link' frame and admittance velocity in 'base_link' frame
            self.target_cartesian_velocity[0] = self.base_link_desired_velocity[0] + self.admittance_velocity_transformed[0]
            self.target_cartesian_velocity[1] = self.base_link_desired_velocity[1] + self.admittance_velocity_transformed[1]
            self.target_cartesian_velocity[2] = self.base_link_desired_velocity[2] + self.admittance_velocity_transformed[2]
            self.target_cartesian_velocity[3] = self.base_link_desired_velocity[3] + self.admittance_velocity_transformed[3]
            self.target_cartesian_velocity[4] = self.base_link_desired_velocity[4] + self.admittance_velocity_transformed[4]
            self.target_cartesian_velocity[5] = self.base_link_desired_velocity[5] + self.admittance_velocity_transformed[5]
            
            # self.target_cartesian_velocity[0] = self.base_link_desired_velocity[0] 
            # self.target_cartesian_velocity[1] = self.base_link_desired_velocity[1] 
            # self.target_cartesian_velocity[2] = self.base_link_desired_velocity[2] 
            # self.target_cartesian_velocity[3] = self.base_link_desired_velocity[3] 
            # self.target_cartesian_velocity[4] = self.base_link_desired_velocity[4] 
            # self.target_cartesian_velocity[5] = self.base_link_desired_velocity[5] 

            # print("target_cartesian_velocity: befor check for limits")
            # print(self.target_cartesian_velocity)

            # * Check self.target_cartesian_velocity for the min/max velocity limits
            # Calculate the norm of target_cartesian_velocity (trans and rot)
            target_cartesian_trans_velocity_norm = math.sqrt(pow(self.target_cartesian_velocity[0],2) + pow(self.target_cartesian_velocity[1],2) + pow(self.target_cartesian_velocity[2],2))
            
            target_cartesian_rot_velocity_norm = math.sqrt(pow(self.target_cartesian_velocity[3],2) + pow(self.target_cartesian_velocity[4],2) + pow(self.target_cartesian_velocity[5],2))
            

            # print("target_cartesian_trans_velocity_norm")
            # print(target_cartesian_trans_velocity_norm)

            #  Check for cartesian velocity max limit and set to max limit, if max limit is exceeded
            if target_cartesian_trans_velocity_norm > self.cartesian_velocity_trans_max_limit:
                for i in range(3):
                    self.target_cartesian_velocity[i] = (self.target_cartesian_velocity[i]/target_cartesian_trans_velocity_norm) * self.cartesian_velocity_trans_max_limit
                    
            if target_cartesian_rot_velocity_norm > self.cartesian_velocity_rot_max_limit:
                for i in range(3,6):
                    self.target_cartesian_velocity[i] = (self.target_cartesian_velocity[i]/target_cartesian_rot_velocity_norm) * self.cartesian_velocity_rot_max_limit
            
            # Check for cartesian velocity min limit and set to null, if min limit is understeps
            if target_cartesian_trans_velocity_norm < self.cartesian_velocity_trans_min_limit:
                for i in range(3):
                    self.target_cartesian_velocity[i] = 0.0
            
            if target_cartesian_rot_velocity_norm < self.cartesian_velocity_rot_min_limit:
                for i in range(3,6):
                    self.target_cartesian_velocity[i] = 0.0
            
            # print("target_cartesian_velocity: after check for limits")
            # print(self.target_cartesian_velocity)
            
            # * Get the current joint states 
            self.current_joint_states_array = self.group.get_current_joint_values() 
            
            #print("self.current_joint_states_array: ")
            #print(self.current_joint_states_array)
            
            # * Calculate the jacobian-matrix
            self.jacobian = self.group.get_jacobian_matrix(self.current_joint_states_array)
            
            
            # * Calculate the inverse of the jacobian-matrix
            self.inverse_jacobian = numpy.linalg.inv(self.jacobian)
            
            
            # * Singulartiy avoidance: OLMM
            u,s,v = numpy.linalg.svd(self.jacobian,full_matrices=True)
            
            for sigma in range(len(s)):
                if s[sigma] < self.sigma_limit:
                    self.singular_velocity = numpy.dot(self.scalar_adjusting_function(s[sigma]),numpy.dot(numpy.dot(u[:,sigma],self.target_cartesian_velocity),u[:,sigma]))
                    self.singular_velocity += self.singular_velocity
                    
                    # print("sigma")
                    # print(s[sigma])
                    # print("self.singular_velocity")
                    # print(self.singular_velocity)
                self.target_cartesian_velocity = self.target_cartesian_velocity - self.singular_velocity
                
                # Publish the calculated singular_velocity, in 'ur/base_link' frame
                self.singular_velocity_msg.data = self.singular_velocity
                self.singularity_velocity_pub.publish(self.singular_velocity_msg)
                self.singular_velocity = [0.0,0.0,0.0,0.0,0.0,0.0]
                
                
                
                
            # * Calculate the target joint velocity with the inverse jacobian-matrix and the target cartesain velociy
            self.target_joint_velocity.data = self.inverse_jacobian.dot(self.target_cartesian_velocity)
            
            # print("ur16e_admittance_velocity: ")
            # print(self.target_joint_velocity)
            
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
    