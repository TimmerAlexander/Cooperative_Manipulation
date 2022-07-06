// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cooperative_manipulation_controllers/franka_impedance_control_hardware.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace cooperative_manipulation_controllers {

bool FrankaImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("FrankaImpedanceController: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "FrankaImpedanceController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "FrankaImpedanceController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "FrankaImpedanceController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("FrankaImpedanceController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("FrankaImpedanceController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "FrankaImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "FrankaImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "FrankaImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "FrankaImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  velocity_cartesian_interface_ =
      robot_hw->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  sub_velocity_command_ = node_handle.subscribe(
    "desired_velocity", 1, &FrankaImpedanceController::velocityCmdCallback,this,ros::TransportHints().reliable().tcpNoDelay());

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  return true;
}

void FrankaImpedanceController::velocityCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& vel_cmd)
{
  if (vel_cmd->data.size() != 6) {
    ROS_ERROR_STREAM(
          "EffortJointTorqueController: Published Commands are not of size 6");
  }
  else{
    for (size_t i = 0; i < 6; ++i) {
    velocity_command[i] = vel_cmd->data[i];
    ROS_INFO_STREAM(velocity_command[i]);
    }
  }
}


void FrankaImpedanceController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
  
  std::array<double, 6> velocity_desired = velocity_cartesian_handle_->getRobotState().O_dP_EE_d;

  for (size_t i = 0; i < 6; i++) {
    if (vel_current_[i] < velocity_command[i]) {
    // Acceleration
    vel_current_[i] += vel_acc_;
    vel_current_[i] = std::fmin(vel_current_[i], velocity_command[i]);
    }
    else if (vel_current_[i] > velocity_command[i]) {
      // Deceleration
      vel_current_[i] -= vel_acc_;
      vel_current_[i] = std::fmax(vel_current_[i], velocity_command[i]);
    }
    else if(velocity_command[i] == 0.0){
      if (vel_current_[i] > 0.0){
        // Stop when vel_current_ > 0.0
        vel_current_[i] -= vel_acc_;
        vel_current_[i] = std::fmax(vel_current_[i], velocity_command[i]);
      }
      else if (vel_current_[i] < 0.0){
        // Stop when vel_current_ < 0.0
        vel_current_[i] += vel_acc_;
        vel_current_[i] = std::fmax(vel_current_[i], velocity_command[i]);
      }
    }
  }

  // Set velocity command
  std::array<double, 6> command = {vel_current_};
  velocity_cartesian_handle_->setCommand(command);
  // Get the robot_state, coriolis and gravity
  franka::RobotState robot_state = velocity_cartesian_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();
  // Filter the joint velocity
  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }
  // Calculate the joint moment with the impedance control law
  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; ++i) {
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                          k_gains_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
                          d_gains_[i] * (robot_state.dq_d[i] - dq_filtered_[i]);
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  // Set the torque for each joint
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }
}

std::array<double, 7> FrankaImpedanceController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace cooperative_manipulation_controllers

PLUGINLIB_EXPORT_CLASS(cooperative_manipulation_controllers::FrankaImpedanceController,
                       controller_interface::ControllerBase)

