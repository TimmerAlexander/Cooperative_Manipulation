#!/usr/bin/env python3

# /***************************************************************************

#
# @package: panda_siimulator_examples
# @metapackage: panda_simulator
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
#

# **************************************************************************/

# /***************************************************************************
# Copyright (c) 2019-2021, Saif Sidhik

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# **************************************************************************/

"""
    Description...

    Impedance controller

    Input:
    * Desired cartesian velocity of the EE: desired_velocity (In 'world' frame)

    Output:
    * Joint effort: self.command_msg.effort (Float64MultiArray)
"""

import numpy
import rospy
import tf
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float64MultiArray

class franka_impedance_controller():

    def config(self):
        # Min and max limits for the cartesian velocity (trans/rot) (unit: [m/s],[rad/s])
        self.cartesian_velocity_trans_min_limit = 0.001
        self.cartesian_velocity_trans_max_limit = 0.1
        self.cartesian_velocity_rot_min_limit = 0.001
        self.cartesian_velocity_rot_max_limit = 0.1
        # Control thread publish rate
        self.publish_rate = 100 # [Hz]
        # Create joint command message 
        self.command_msg = Float64MultiArray()
        # Initialize desired translational and rotation velocity
        self.desired_velocity_trans_transformed  = numpy.array([0.0,0.0,0.0])
        self.desired_velocity_rot_transformed  = numpy.array([0.0,0.0,0.0])
        # Initialize trajectory velocity for object rotation
        self.world_trajectory_velocity = numpy.array([0.0,0.0,0.0])


    def __init__(self):
        # * Load config parameters
        self.config()
        
        # * Initialize node
        rospy.init_node("franka_hardware_node")
        
        # * Get namespace for topics from launch file
        self.namespace = rospy.get_param("~panda_ns")

        # * Initialize tf TransformListener
        self.tf_listener = tf.TransformListener()

        # Wait for transformations in tf tree
        rospy.loginfo("Wait for transformation '/panda_link8' to 'world'.")
        self.tf_listener.waitForTransform("/panda_link8","/world", rospy.Time(), rospy.Duration(5.0))
        rospy.loginfo("Wait for transformation '/panda_link0' to '/panda_link8'.")
        self.tf_listener.waitForTransform("/panda_link0","/panda_link8", rospy.Time(), rospy.Duration(5.0))
        rospy.loginfo("Wait for transformation 'world' to '/panda_link0'.")
        self.tf_listener.waitForTransform("world","/panda_link0", rospy.Time(), rospy.Duration(5.0))

        # Wait for transformations from 'world' to 'panda_gripper' and 'world' to 'ur16e_gripper'
        rospy.loginfo("Wait for transformation 'world' to '/panda_EE'.")
        self.tf_listener.waitForTransform("world","/panda_EE", rospy.Time(), rospy.Duration(10.0))
        # rospy.loginfo("Wait for transformation 'world' to 'ur16e_gripper'.")
        # self.tf_listener.waitForTransform("world","ur16e_gripper", rospy.Time(), rospy.Duration(10.0))

        # * Initialize subscriber:
        self.cartesian_msg_sub = rospy.Subscriber(
            '/cooperative_manipulation/cartesian_velocity_command',
            Twist,
            self.cartesian_msg_callback,
            queue_size=1,
            tcp_nodelay=True)

        # * Initialize publisher:
        # Also create a publisher to publish joint commands
        self.velocity_command_publisher = rospy.Publisher(
                '/' + self.namespace + '/franka_impedance_controller/desired_velocity',
                Float64MultiArray,
                tcp_nodelay=True,
                queue_size=1)

        # * Initialize on_shutdown clean up
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("Launch Franka Hardware Node.")
        # # * Run controller thread
        self.control_thread()

        rospy.spin()

    def control_thread(self):
        """
            Actual control loop. Uses goal pose from the feedback thread
            and current robot states from the subscribed messages to compute
            task-space force, and then the corresponding joint torques.
        """
        # Set rospy.rate
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            # * Check self.target_cartesian_trans_velocity and self.target_cartesian_trot_velocity for the min/max velocity limits
            # Calculate the norm of target_cartesian_velocity (trans and rot)
            target_cartesian_trans_velocity_norm = numpy.linalg.norm(self.desired_velocity_trans_transformed)
            target_cartesian_rot_velocity_norm = numpy.linalg.norm(self.desired_velocity_rot_transformed)

            # print("numpy.linalg.norm(self.desired_velocity_trans_transformed)")
            # print(numpy.linalg.norm(self.desired_velocity_trans_transformed))

            # Check whether the trans/rot velocity  limit has been exceeded. If the trans/rot velocity max limit has been exceeded, then normalize the velocity to the length of the velocity upper limit
            if target_cartesian_trans_velocity_norm > self.cartesian_velocity_trans_max_limit:
                for i in range(3):
                    self.desired_velocity_trans_transformed[i] = (self.desired_velocity_trans_transformed[i]/target_cartesian_trans_velocity_norm) * self.cartesian_velocity_trans_max_limit

            if target_cartesian_rot_velocity_norm > self.cartesian_velocity_rot_max_limit:
                for i in range(3):
                    self.desired_velocity_rot_transformed[i] = (self.desired_velocity_rot_transformed[i]/target_cartesian_rot_velocity_norm) * self.cartesian_velocity_rot_max_limit


            # Check whether the velocity limit has been undershot. If the velocity  has fallen below the min velocity limit, then set the velocity  to zero
            if target_cartesian_trans_velocity_norm < self.cartesian_velocity_trans_min_limit:
                for i in range(3):
                    self.desired_velocity_trans_transformed[i] = 0.0

            if target_cartesian_rot_velocity_norm < self.cartesian_velocity_rot_min_limit:
                for i in range(3):
                    self.desired_velocity_rot_transformed[i] = 0.0
                    
            self.command_msg.data = numpy.append(self.desired_velocity_trans_transformed,self.desired_velocity_rot_transformed) 

            # print("self.command_msg")
            # print(self.command_msg)
            # Publish the velocity command
            self.velocity_command_publisher.publish(self.command_msg)
            rate.sleep()

    def cartesian_msg_callback(self,desired_velocity):
        """
            Get the cartesian velocity command and transform it from the 'world' frame to the 'panda_link8' (EE-frame)frame and from the 'panda_link8' frame to the 'panda_link0' (0-frame)frame.

            rostopic pub -r 10 /cooperative_manipulation/cartesian_velocity_command geometry_msgs/Twist "linear:
            x: 0.0
            y: 0.0
            z: 0.0
            angular:
            x: 0.0
            y: 0.0
            z: 0.0"

        Args:
            desired_velocity (geometry_msgs.msg.Twist): Desired cartesian velocity
        """
        # Get current time stamp
        now = rospy.Time()
        #------------------------------
        # Calculate the trajectory velocity of the manipulator for a rotation of the object
        # Get self.panda_current_position, self.panda_current_quaternion of the '/panda_link8' frame in the 'world' frame
        panda_tf_time = self.tf_listener.getLatestCommonTime("/world", "/panda_link8")
        panda_current_position, panda_current_quaternion = self.tf_listener.lookupTransform("/world", "/panda_link8", panda_tf_time)


        # Get self.panda_current_position, self.panda_current_quaternion of the '/panda_EE' frame in the 'world' frame
        panda_tf_time = self.tf_listener.getLatestCommonTime("/world", "/panda_EE")
        panda_EE_position, panda_EE_quaternion = self.tf_listener.lookupTransform("/world", "/panda_EE", panda_tf_time)

        # Get ur16e_current_position, ur16e_current_quaternion of the 'wrist_3_link' in frame in the 'world' frame
        # ur16e_tf_time = self.tf_listener.getLatestCommonTime("/world", "/wrist_3_link")
        # ur16e_gripper_position, ur16e_gripper_quaternion = self.tf_listener.lookupTransform("/world", "/ur16e_gripper", ur16e_tf_time)

        # print("self.ur16e_current_position, self.ur16e_current_quaternion")
        # print(self.ur16e_current_position, self.ur16e_current_quaternion)
        # print("self.panda_current_position, self.panda_current_quaternion")
        # print(self.panda_position, self.panda_current_quaternion)

        # Object rotation around x axis
        # if desired_velocity.angular.x != 0.0:
        #     panda_current_position_x = numpy.array([
        #         0.0,
        #         panda_current_position[1],
        #         panda_current_position[2]
        #         ])

        #     self.robot_distance_x = numpy.array([
        #         0.0,
        #         ur16e_gripper_position[1] - panda_EE_position[1],
        #         ur16e_gripper_position[2] - panda_EE_position[2],
        #     ])

        #     center_x = (numpy.linalg.norm(self.robot_distance_x)/2) * (1/numpy.linalg.norm(self.robot_distance_x)) * self.robot_distance_x + panda_EE_position
        #     world_desired_rotation_x = numpy.array([desired_velocity.angular.x,0.0,0.0])
        #     world_radius_x = panda_current_position_x - center_x
        #     self.world_trajectory_velocity_x = numpy.cross(world_desired_rotation_x,world_radius_x)
        #     self.world_trajectory_velocity = self.world_trajectory_velocity + self.world_trajectory_velocity_x

        # # Object rotation around y axis
        # if desired_velocity.angular.y != 0.0:
        #     panda_current_position_y = numpy.array([
        #         panda_current_position[0],
        #         0.0,
        #         panda_current_position[2]
        #         ])

        #     self.robot_distance_y = numpy.array([
        #         ur16e_gripper_position[0] - panda_EE_position[0],
        #         0.0,
        #         ur16e_gripper_position[2] - panda_EE_position[2],
        #         ])

        #     center_y = (numpy.linalg.norm(self.robot_distance_y)/2) * (1/numpy.linalg.norm(self.robot_distance_y)) * self.robot_distance_y + panda_EE_position
        #     world_desired_rotation_y = numpy.array([0.0,desired_velocity.angular.y,0.0])
        #     world_radius_y = panda_current_position_y - center_y
        #     self.world_trajectory_velocity_y = numpy.cross(world_desired_rotation_y,world_radius_y)
        #     self.world_trajectory_velocity = self.world_trajectory_velocity + self.world_trajectory_velocity_y



        # # Object rotation around z axis
        # if desired_velocity.angular.z != 0.0:
        #     panda_current_position_z = numpy.array([
        #         panda_current_position[0],
        #         panda_current_position[1],
        #         0.0,
        #         ])

        #     self.robot_distance_z = numpy.array([
        #         ur16e_gripper_position[0] - panda_EE_position[0],
        #         ur16e_gripper_position[1] - panda_EE_position[1],
        #         0.0,
        #         ])

        #     center_z = (numpy.linalg.norm(self.robot_distance_z)/2) * (1/numpy.linalg.norm(self.robot_distance_z)) * self.robot_distance_z + panda_EE_position
        #     world_desired_rotation_z = numpy.array([0.0,0.0,desired_velocity.angular.z])
        #     world_radius_z = panda_current_position_z - center_z
        #     self.world_trajectory_velocity_z = numpy.cross(world_desired_rotation_z,world_radius_z)
        #     self.world_trajectory_velocity = self.world_trajectory_velocity + self.world_trajectory_velocity_z

        #------------------------------

        # world_cartesian_velocity_trans  = Vector3Stamped()
        # world_cartesian_velocity_rot  = Vector3Stamped()
        #  # Converse cartesian_velocity translation to vector3
        # world_cartesian_velocity_trans.header.frame_id = 'world'
        # world_cartesian_velocity_trans.header.stamp = now
        # world_cartesian_velocity_trans.vector.x = desired_velocity.linear.x + self.world_trajectory_velocity[0]
        # world_cartesian_velocity_trans.vector.y = desired_velocity.linear.y + self.world_trajectory_velocity[1]
        # world_cartesian_velocity_trans.vector.z = desired_velocity.linear.z + self.world_trajectory_velocity[2]

        # Transform cartesian_velocity translation from 'world' frame to 'panda_link0' frame and from 'panda_link0' frame to 'panda_link8
        # base_cartesian_velocity_trans = self.tf_listener.transformVector3('panda_link0',world_cartesian_velocity_trans)

        # # Converse cartesian_velocity rotation to vector3
        # world_cartesian_velocity_rot.header.frame_id = 'world'
        # world_cartesian_velocity_rot.header.stamp = now
        # world_cartesian_velocity_rot.vector.x = desired_velocity.angular.x
        # world_cartesian_velocity_rot.vector.y = desired_velocity.angular.y
        # world_cartesian_velocity_rot.vector.z = desired_velocity.angular.z

        # print("world_cartesian_velocity_rot")
        # print(world_cartesian_velocity_rot)

        # Transform cartesian_velocity rotation from 'world' frame to 'panda_link0' frame and from 'panda_link0' frame to 'panda_link8'
        # base_cartesian_velocity_rot = self.tf_listener.transformVector3('panda_link0',world_cartesian_velocity_rot)

        # print("base_cartesian_velocity_rot")
        # print(base_cartesian_velocity_rot)

        # Converse cartesian_velocity from vector3 to numpy.array
        self.desired_velocity_trans_transformed = [
            desired_velocity.linear.x,
            desired_velocity.linear.y,
            desired_velocity.linear.z
            ]

        self.desired_velocity_rot_transformed = [
            desired_velocity.angular.x,
            desired_velocity.angular.y,
            desired_velocity.angular.z,
            ]
        
        # Set the trajectory velocity for an object rotation to zero
        # self.world_trajectory_velocity = [0.0,0.0,0.0]


    def _on_shutdown(self):
        """
            Shutdown publisher and subscriber when rosnode dies.
        """
        print("Shutdown impedance controller:")
        print("Shutdown publisher joint velocity!")
        self.velocity_command_publisher.publish(self.velocity_command_publisher)
        print("Unregister from cartesian_state_sub!")
        self.cartesian_state_sub.unregister()


if __name__ == "__main__":
    franka_impedance_controller()