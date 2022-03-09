#!/usr/bin/env python

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
    This is a demo showing task-space control on the 
    simulator robot using the ROS topics and messages directly 
    from panda_simulator. The task-space force for the desired
    pose is computed using a simple PD law, and the corresponding
    joint torques are computed and sent to the robot.
    
    After launching the simulator (panda_world.launch),
    run this demo using the command:
        
        roslaunch panda_simulator_examples demo_task_space_control.launch --use_fri:=false

"""

import copy
import rospy
import threading
import quaternion
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from franka_core_msgs.msg import EndPointState, JointCommand, RobotState

# -- add to pythonpath for finding rviz_markers.py 
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
# -------------------------------------------------

from rviz_markers import RvizMarkers

# --------- Modify as required ------------
# Task-space controller parameters
# stiffness gains
P_pos = 50.
P_ori = 25.
# damping gains
D_pos = 10.
D_ori = 1.
# -----------------------------------------
publish_rate = 100

JACOBIAN = None
CARTESIAN_POSE = None
CARTESIAN_VEL = None

destination_marker = RvizMarkers()


def _on_robot_state(msg):
    """
        Callback function for updating jacobian and EE velocity from robot state
    """
    global JACOBIAN, CARTESIAN_VEL
    # Get Jacobian matrix 0_dJac_EE: zero jacobian of end-effector frame. Vectorized 6x7 Jacobian, column-major
    JACOBIAN = np.asarray(msg.O_Jac_EE).reshape(6,7,order = 'F')
    print(JACOBIAN)
    # Get EE velocities from msg.0_dP_EE: EE vel computed as J*dq
    CARTESIAN_VEL = {
                'linear': np.asarray([msg.O_dP_EE[0], msg.O_dP_EE[1], msg.O_dP_EE[2]]),
                'angular': np.asarray([msg.O_dP_EE[3], msg.O_dP_EE[4], msg.O_dP_EE[5]]) }

def _on_endpoint_state(msg):
    """
        Callback function to get current end-point state
    """
    # pose message received is a vectorised column major transformation matrix
    global CARTESIAN_POSE
    # O_T_EE: Measured end effector pose in base frame
    cart_pose_trans_mat = np.asarray(msg.O_T_EE).reshape(4,4,order='F')
    # Transform cart_pose_trans_mat into the dictionary CARTESIAN_POSE
    CARTESIAN_POSE = {
        'position': cart_pose_trans_mat[:3,3],
        'orientation': quaternion.from_rotation_matrix(cart_pose_trans_mat[:3,:3]) }

def quatdiff_in_euler(quat_curr, quat_des):
    """
        Compute difference between quaternions and return 
        Euler angles as difference
    """
    # Transform current orientation to a rotation matrix 
    curr_mat = quaternion.as_rotation_matrix(quat_curr)
    # Transform goal orientation to a rotation matrix
    des_mat = quaternion.as_rotation_matrix(quat_des)
    
    rel_mat = des_mat.T.dot(curr_mat)
    
    rel_quat = quaternion.from_rotation_matrix(rel_mat)
    vec = quaternion.as_float_array(rel_quat)[1:]
    if rel_quat.w < 0.0:
        vec = -vec
        
    return -des_mat.dot(vec)

def control_thread(rate):
    """
        Actual control loop. Uses goal pose from the feedback thread
        and current robot states from the subscribed messages to compute
        task-space force, and then the corresponding joint torques.
    """
    while not rospy.is_shutdown():
        error = 100.
        while error > 0.005:
            # Create a deepcopy of CARTESIAN_POSE
            curr_pose = copy.deepcopy(CARTESIAN_POSE)
            
            # Assgin curr_pose['position'],curr_pose['orientation'] to curr_pos, curr_ori
            curr_pos, curr_ori = curr_pose['position'],curr_pose['orientation']
            
            # Assgin (CARTESIAN_VEL['linear']).reshape([3,1]) to curr_vel
            curr_vel = (CARTESIAN_VEL['linear']).reshape([3,1])
            
            # Assgin CARTESIAN_VEL['angular'].reshape([3,1]) to curr_omg
            curr_omg = CARTESIAN_VEL['angular'].reshape([3,1])
            
            # Calculate position difference
            delta_pos = (goal_pos - curr_pos).reshape([3,1])
            
            # Calculate orientation difference
            delta_ori = quatdiff_in_euler(curr_ori, goal_ori).reshape([3,1])
            
            # Desired task-space force using PD law (Stiffness and Damping)
            F = np.vstack([P_pos*(delta_pos), P_ori*(delta_ori)]) - \
                np.vstack([D_pos*(curr_vel), D_ori*(curr_omg)])
                
            # Calculate the vector norm of delta_pos and delta_ori and sum the results
            error = np.linalg.norm(delta_pos) + np.linalg.norm(delta_ori)
            
            # Create a deepcopy of J
            J = copy.deepcopy(JACOBIAN)
            
            # Calculate the joint torques to be commanded
            tau = np.dot(J.T,F)
            
            # publish joint commands
            command_msg.effort = tau.flatten()
            joint_command_publisher.publish(command_msg)
            rate.sleep()

def process_feedback(feedback):
    """
    InteractiveMarker callback function. Update target pose.
    """
    global goal_pos, goal_ori

    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        p = feedback.pose.position
        q = feedback.pose.orientation
        goal_pos = np.array([p.x,p.y,p.z])
        goal_ori = np.quaternion(q.w, q.x,q.y,q.z)

def _on_shutdown():
    """
        Clean shutdown controller thread when rosnode dies.
    """
    global ctrl_thread, cartesian_state_sub, \
        robot_state_sub, joint_command_publisher
    if ctrl_thread.is_alive():
        ctrl_thread.join()

    robot_state_sub.unregister()
    cartesian_state_sub.unregister()
    joint_command_publisher.unregister()
    
if __name__ == "__main__":
    # global goal_pos, goal_ori, ctrl_thread

    rospy.init_node("ts_control_sim_only")

    # if not using franka_ros_interface, you have to subscribe to the right topics
    # to obtain the current end-effector state and robot jacobian for computing 
    # commands
    cartesian_state_sub = rospy.Subscriber(
        'panda_simulator/custom_franka_state_controller/tip_state',
        EndPointState,
        _on_endpoint_state,
        queue_size=1,
        tcp_nodelay=True)

    robot_state_sub = rospy.Subscriber(
        'panda_simulator/custom_franka_state_controller/robot_state',
        RobotState,
        _on_robot_state,
        queue_size=1,
        tcp_nodelay=True)
    
    # create joint command message and fix its type to joint torque mode
    command_msg = JointCommand()
    command_msg.names = ['panda_joint1','panda_joint2','panda_joint3',\
        'panda_joint4','panda_joint5','panda_joint6','panda_joint7']
    command_msg.mode = JointCommand.TORQUE_MODE
    
    # Also create a publisher to publish joint commands
    joint_command_publisher = rospy.Publisher(
            'panda_simulator/motion_controller/arm/joint_commands',
            JointCommand,
            tcp_nodelay=True,
            queue_size=1)

    # wait for messages to be populated before proceeding
    rospy.loginfo("Subscribing to robot state topics...")
    while (True):
        if not (JACOBIAN is None or CARTESIAN_POSE is None):
            break
    rospy.loginfo("Recieved messages; Starting Demo.")


    pose = copy.deepcopy(CARTESIAN_POSE)
    start_pos, start_ori = pose['position'],pose['orientation']
    goal_pos, goal_ori = start_pos, start_ori # set goal pose a starting pose in the beginning

    # start controller thread
    rospy.on_shutdown(_on_shutdown)
    rate = rospy.Rate(publish_rate)
    ctrl_thread = threading.Thread(target=control_thread, args = [rate])
    ctrl_thread.start()

    # ------------------------------------------------------------------------------------
    server = InteractiveMarkerServer("basic_control")

    position = Point( start_pos[0], start_pos[1], start_pos[2])
    marker = destination_marker.makeMarker( False, InteractiveMarkerControl.MOVE_ROTATE_3D, \
                                        position, quaternion.as_float_array(start_ori), True)
    server.insert(marker, process_feedback)
    
    server.applyChanges()

    rospy.spin()    
    # ------------------------------------------------------------------------------------