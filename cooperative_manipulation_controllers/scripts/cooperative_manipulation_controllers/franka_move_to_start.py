#!/usr/bin/env python3

# /***************************************************************************

# 
# @package: franka_interface
# @metapackage: franka_ros_interface
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
:info: 
   commands robot to move to start pose

"""

import rospy
from franka_interface import ArmInterface

class franka_move_to_start:
    
    def __init__(self):
        rospy.init_node("franka_move_to_start_node")
        
        # * Get joint poses from launch file
        self.panda_joint1 = float(rospy.get_param("~panda_joint1"))
        self.panda_joint2 = float(rospy.get_param("~panda_joint2"))
        self.panda_joint3 = float(rospy.get_param("~panda_joint3"))
        self.panda_joint4 = float(rospy.get_param("~panda_joint4"))
        self.panda_joint5 = float(rospy.get_param("~panda_joint5"))
        self.panda_joint6 = float(rospy.get_param("~panda_joint6"))
        self.panda_joint7 = float(rospy.get_param("~panda_joint7"))

        self.panda_start_joint_dict = {
            "panda_joint1": self.panda_joint1,
            "panda_joint2": self.panda_joint2,  
            "panda_joint3": self.panda_joint3, 
            "panda_joint4": self.panda_joint4,
            "panda_joint5": self.panda_joint5, 
            "panda_joint6": self.panda_joint6, 
            "panda_joint7": self.panda_joint7
        }

        self.r = ArmInterface()
        self.move_to_start(self.r,self.panda_start_joint_dict,timeout=20.0,speed=0.05)
            
    def move_to_start(self, arm_interface_object,joint_start_pose_dict, timeout=15.0, speed=0.15):
            """
            :type timeout: float
            :param timeout: seconds to wait for move to finish [15]
            :type speed: float
            :param speed: ratio of maximum joint speed for execution
            default= 0.15; range= [0.0-1.0]
            """
            arm_interface_object.set_joint_position_speed(speed)
            arm_interface_object.move_to_joint_positions(joint_start_pose_dict, timeout)
            
if __name__ == '__main__':
    franka_move_to_start()