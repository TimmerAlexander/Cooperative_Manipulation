// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>


#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

namespace cooperative_manipulation_controllers {

class FrankaImpedanceController : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaModelInterface,
                                            franka_hw::FrankaVelocityCartesianInterface,
                                            hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void velocityCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& vel_cmd);

 private:
  ros::Subscriber sub_velocity_command_;

  std::array<double, 6>  vel_current_ = {0.0,0.0,0.0,0.0,0.0,0.0};
  std::array<double, 6> velocity_command = {0.0,0.0,0.0,0.0,0.0,0.0};

  // Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  static constexpr double kDeltaTauMax{1.0};
  double vel_acc_{0.0001};
  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  double coriolis_factor_{1.0};
  std::array<double, 7> dq_filtered_;
  std::array<double, 16> initial_pose_;

  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};
};

}  // namespace cooperative_manipulation_controllers
