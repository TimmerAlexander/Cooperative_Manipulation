// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cooperative_manipulation_controllers/franka_cartesian_impedance_control_hardware.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace cooperative_manipulation_controllers {

bool FrankaCartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw,
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

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 6) {
    ROS_ERROR(
        "FrankaImpedanceController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 6) {
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
        "CartesianVelocityController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  sub_velocity_command_ = node_handle.subscribe(
    "desired_velocity", 1, &FrankaCartesianImpedanceController::velocityCmdCallback,this,ros::TransportHints().reliable().tcpNoDelay());

  // Set dq_filtered matrix
  Eigen::Matrix<double, 7, 1> dq_filtered;
  dq_filtered.setZero();

  // Set stiffness matrix
  Eigen::MatrixXd stiffness(6, 6);
  stiffness.setZero();
  stiffness(0, 0) = k_gains_[0];
  stiffness(1, 1) = k_gains_[1];
  stiffness(2, 2) = k_gains_[2];
  stiffness(3, 3) = k_gains_[3];
  stiffness(4, 4) = k_gains_[4];
  stiffness(5, 5) = k_gains_[5];
  // Set damping matrix
  Eigen::MatrixXd damping(6, 6);
  damping.setZero();
  damping(0, 0) = d_gains_[0];
  damping(1, 1) = d_gains_[1];
  damping(2, 2) = d_gains_[2];
  damping(3, 3) = d_gains_[3];
  damping(4, 4) = d_gains_[4];
  damping(5, 5) = d_gains_[5];


  dq_filtered_ = dq_filtered;                                  
  stiffness_ = stiffness;
  damping_ = damping;


  ROS_INFO_STREAM("Stiffness matrix");
  for (size_t i = 0; i < 6; ++i) {
    ROS_INFO_STREAM(stiffness_.row(i));
  }
  ROS_INFO_STREAM("Damping matrix");
  for (size_t i = 0; i < 6; ++i) {
    ROS_INFO_STREAM(damping_ .row(i));
  }

  return true;
}

void FrankaCartesianImpedanceController::velocityCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& vel_cmd)
{
  if (vel_cmd->data.size() != 6) {
    ROS_ERROR_STREAM(
          "EffortJointTorqueController: Published Commands are not of size 6");
  }
  else{
    for (size_t i = 0; i < 6; ++i) {
    velocity_command[i] = vel_cmd->data[i];
    //ROS_INFO_STREAM(velocity_command[i]);
    }
  }
}

void FrankaCartesianImpedanceController::starting(const ros::Time& /*time*/) {
  // Get equilibrium pose
  franka::RobotState initial_state = velocity_cartesian_handle_->getRobotState();
  // Convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // Set equilibrium point to current state
  desired_position = initial_transform.translation();
  desired_orientation = Eigen::Quaterniond(initial_transform.linear());
}

void FrankaCartesianImpedanceController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
  
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

  // Get the robot_state  
  franka::RobotState robot_state = velocity_cartesian_handle_->getRobotState();
  // Get cuurrent velocity and desired velocity,
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_d(robot_state.dq_d.data());
  // Get coriolis
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  // Get gravity 
  std::array<double, 7> gravity = model_handle_->getGravity(); // ?????
  // Get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  // Get the current end effector pose in base frame. 
  Eigen::Affine3d current_pose(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d current_position(current_pose.translation());
  Eigen::Quaterniond current_orientation(current_pose.linear());
  // Get the last desired end effector pose of motion generation in base frame. 
  Eigen::Affine3d desired_pose(Eigen::Matrix4d::Map(robot_state.O_T_EE_d.data()));
  desired_position = desired_pose.translation();
  desired_orientation = Eigen::Quaterniond(desired_pose.linear());

  // Compute pose_error to desired pose
  Eigen::Matrix<double, 6, 1>  pose_error;
  // Position pose_error
  pose_error.head(3) << current_position - desired_position;

  // Calculate correct orientation
  if (desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0) {
    current_orientation.coeffs() << -current_orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(current_orientation.inverse() * desired_orientation);
  pose_error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  pose_error.tail(3) << -current_pose.linear() * pose_error.tail(3);

  // compute control
  Eigen::VectorXd tau_task(7), tau_d(7);

  // Filter the joint velocity
  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-stiffness_ * pose_error - damping_ * (jacobian * (dq_filtered_ - dq_d)));

  // Desired torque
  tau_d << tau_task + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
















  // Eigen::Quaterniond error_quaternion(current_orientation.inverse() * desired_orientation);
  
  // orientation_error_matrix.head(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // // Transform to base frame
  // orientation_error_matrix.head(3) << -current_pose.linear() * orientation_error_matrix.head(3);
  // Store in array
  // pose_error[3] = 0.0;
  // pose_error[4] = 0.0;
  // pose_error[5] = 0.0;



  // // Compute to desired velocity
  // // Filter the joint velocity
  // double alpha = 0.99;
  // for (size_t i = 0; i < 7; i++) {
  //   dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  // }
  // Eigen::VectorXd velocity_error(6);
  // velocity_error =  (jacobian * (dq_d - dq));

  
  // // Calculate the force with the impedance control law
  // Eigen::Matrix<double, 6, 1> f_d_calculated;
  // for (size_t i = 0; i < 6; ++i) {
  //   f_d_calculated[i] = k_gains_[i] * pose_error[i] + d_gains_[i] * velocity_error[i];
  // }
  // // Transform the force to joint torque
  // Eigen::Matrix<double, 7, 1>tau_d_calculated_matrix;
  // tau_d_calculated_matrix = jacobian.transpose() * f_d_calculated;


  // // Desired torque
  // std::array<double, 7> tau_d_calculated;
  // for (size_t i = 0; i < 7; i++) {
  //   tau_d_calculated[i] = tau_d_calculated_matrix[i] + coriolis[i];
  // }


  // // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // // 1000 * (1 / sampling_time).
  // std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  // // Set the torque for each joint
  // for (size_t i = 0; i < 7; ++i) {
  //   joint_handles_[i].setCommand(tau_d_saturated[i]);
  // }
}

// std::array<double, 7>FrankaCartesianImpedanceController::saturateTorqueRate(
//     const std::array<double, 7>& tau_d_calculated,
//     const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
//   std::array<double, 7> tau_d_saturated{};
//   for (size_t i = 0; i < 7; i++) {
//     double difference = tau_d_calculated[i] - tau_J_d[i];
//     tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
//   }
//   return tau_d_saturated;
// }


Eigen::Matrix<double, 7, 1>FrankaCartesianImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}


}  // namespace cooperative_manipulation_controllers

PLUGINLIB_EXPORT_CLASS(cooperative_manipulation_controllers::FrankaCartesianImpedanceController,
                       controller_interface::ControllerBase)

