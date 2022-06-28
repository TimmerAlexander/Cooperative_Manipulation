// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cooperative_manipulation_controllers/franka_joint_torque_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>


namespace franka_example_controllers {

bool FrankaJointTorqueController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("FrankaJointTorqueController: Could not read parameter arm_id");
    return false;
  }
  // if (!node_handle.getParam("radius", radius_)) {
  //   ROS_INFO_STREAM(
  //       "FrankaJointTorqueController: No parameter radius, defaulting to: " << radius_);
  // }
  // if (std::fabs(radius_) < 0.005) {
  //   ROS_INFO_STREAM("FrankaJointTorqueController: Set radius to small, defaulting to: " << 0.1);
  //   radius_ = 0.1;
  // }

  // if (!node_handle.getParam("vel_max", vel_max_)) {
  //   ROS_INFO_STREAM(
  //       "FrankaJointTorqueController: No parameter vel_max, defaulting to: " << vel_max_);
  // }
  // if (!node_handle.getParam("acceleration_time", acceleration_time_)) {
  //   ROS_INFO_STREAM(
  //       "FrankaJointTorqueController: No parameter acceleration_time, defaulting to: "
  //       << acceleration_time_);
  // }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "FrankaJointTorqueController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  // if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
  //   ROS_ERROR(
  //       "FrankaJointTorqueController:  Invalid or no k_gain parameters provided, aborting "
  //       "controller init!");
  //   return false;
  // }

  // if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
  //   ROS_ERROR(
  //       "FrankaJointTorqueController:  Invalid or no d_gain parameters provided, aborting "
  //       "controller init!");
  //   return false;
  // }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("FrankaJointTorqueController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("FrankaJointTorqueController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "FrankaJointTorqueController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "FrankaJointTorqueController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* cartesian_pose_interface = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface == nullptr) {
    ROS_ERROR_STREAM(
        "FrankaJointTorqueController: Error getting cartesian pose interface from hardware");
    return false;
  }
  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "FrankaJointTorqueController: Exception getting cartesian pose handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "FrankaJointTorqueController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "FrankaJointTorqueController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
//-------------------------------------------------------------------------------------------------------
  sub_tau_command_ = node_handle.subscribe("tau_command", 1, &FrankaJointTorqueController::chatterCallback,this,ros::TransportHints().reliable().tcpNoDelay());

//-------------------------------------------------------------------------------------------------------

  torques_publisher_.init(node_handle, "torque_comparison", 1);

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  return true;
}

void FrankaJointTorqueController::starting(const ros::Time& /*time*/) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
}

//------------------------------------------------------------------------------------------------
void FrankaJointTorqueController::chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& tau_cmd)
{
  if (sizeof(tau_cmd) != 7) {
    ROS_ERROR_STREAM(
          "EffortJointTorqueController: Published Commands are not of size 7");
  }
  else{
    for (size_t i = 0; i < 7; ++i) {
    tau_command[i] = tau_cmd->data[i];
    ROS_INFO("I heard: [%f]", tau_cmd->data[i]);
    }
  }

}


void FrankaJointTorqueController::update(const ros::Time& /*time*/, const ros::Duration& period) {
                                              
  // if (vel_current_ < vel_max_) {
  //   vel_current_ += period.toSec() * std::fabs(vel_max_ / acceleration_time_);
  // }

  // angle_ += period.toSec() * vel_current_ / std::fabs(radius_);
  // if (angle_ > 2 * M_PI) {
  //   angle_ -= 2 * M_PI;
  // }

  // double delta_y = radius_ * (1 - std::cos(angle_));
  // double delta_z = radius_ * std::sin(angle_);

  // std::array<double, 16> pose_desired = initial_pose_; //current endeffector pose
  // pose_desired[13] += delta_y;
  // pose_desired[14] += delta_z;
  // cartesian_pose_handle_->setCommand(pose_desired);

  float test = period.toSec();
  // ROS_INFO_STREAM(test);

  franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  // double alpha = 0.99;
  // for (size_t i = 0; i < 7; i++) {
  //   dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  // }

  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; ++i) {
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i]; //+ double tau_command[i];
    
    
    // +
    //                       k_gains_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
    //                       d_gains_[i] * (robot_state.dq_d[i] - dq_filtered_[i]);
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  // Get tau diff
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }

  if (rate_trigger_() && torques_publisher_.trylock()) {

    std::array<double, 7> tau_j = robot_state.tau_J;
    std::array<double, 7> tau_error;
    double error_rms(0.0);

    for (size_t i = 0; i < 7; ++i) {
      tau_error[i] = last_tau_d_[i] - tau_j[i];
      error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
    }
    torques_publisher_.msg_.root_mean_square_error = error_rms;
    for (size_t i = 0; i < 7; ++i) {
      torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
      torques_publisher_.msg_.tau_error[i] = tau_error[i];
      torques_publisher_.msg_.tau_measured[i] = tau_j[i];
    }
    torques_publisher_.unlockAndPublish();
  }

  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
  }
}


//------------------------------------------------------------------------------------------------
std::array<double, 7> FrankaJointTorqueController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::FrankaJointTorqueController,
                       controller_interface::ControllerBase)
