////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    cartesian_force_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_force_controller/cartesian_force_controller.h>

#include <cmath>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"

namespace cartesian_force_controller
{
CartesianForceController::CartesianForceController()
: Base::CartesianControllerBase(), m_hand_frame_control(true)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", true);

  // 添加死区和调零参数
  auto_declare<double>("force_deadzone", 1.0);    // 力死区(N)
  auto_declare<double>("torque_deadzone", 0.05);   // 力矩死区(Nm)
  auto_declare<int>("calibration_samples", 100);   // 调零采样数
  auto_declare<double>("auto_calibration_interval", 300.0);  // 自动调零间隔(秒)

  // 添加滤波器参数
  auto_declare<double>("filter_coefficient", 0.05);  // 滤波系数(0-1)

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link = get_node()->get_parameter("ft_sensor_ref_link").as_string();
  if (!Base::robotChainContains(m_ft_sensor_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), m_ft_sensor_ref_link
                                                    << " is not part of the kinematic chain from "
                                                    << Base::m_robot_base_link << " to "
                                                    << Base::m_end_effector_link);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Make sure sensor wrenches are interpreted correctly
  setFtSensorReferenceFrame(Base::m_end_effector_link);

  m_target_wrench_subscriber = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
    get_node()->get_name() + std::string("/target_wrench"), 10,
    std::bind(&CartesianForceController::targetWrenchCallback, this, std::placeholders::_1));

  m_ft_sensor_wrench_subscriber =
    get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_node()->get_name() + std::string("/ft_sensor_wrench"), 10,
      std::bind(&CartesianForceController::ftSensorWrenchCallback, this, std::placeholders::_1));

  m_target_wrench.setZero();
  m_ft_sensor_wrench.setZero();

  m_calib_wrench_publisher = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
    get_node()->get_name() + std::string("/calibration_wrench"), 10);

  // 获取参数
  m_force_deadzone = get_node()->get_parameter("force_deadzone").as_double();
  m_torque_deadzone = get_node()->get_parameter("torque_deadzone").as_double();
  m_calibration_samples = get_node()->get_parameter("calibration_samples").as_int();
  m_auto_calibration_interval = get_node()->get_parameter("auto_calibration_interval").as_double();
  
  // 初始化调零相关变量
  m_force_bias.setZero();
  m_is_calibrating = false;
  m_calibration_data.clear();
  m_last_calibration_time = get_node()->get_clock()->now();

  // 获取滤波器参数
  m_filter_coefficient = get_node()->get_parameter("filter_coefficient").as_double();
  m_filter_initialized = false;
  m_filtered_wrench.setZero();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_deactivate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianForceController::update(const rclcpp::Time & time,
                                                                   const rclcpp::Duration & period)
{
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  // Control the robot motion in such a way that the resulting net force
  // vanishes.  The internal 'simulation time' is deliberately independent of
  // the outer control cycle.
  auto internal_period = rclcpp::Duration::from_seconds(0.02);

  // Compute the net force
  ctrl::Vector6D error = computeForceError();

  // Turn Cartesian error into joint motion
  Base::computeJointControlCmds(error, internal_period);

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianForceController::computeForceError()
{
  ctrl::Vector6D target_wrench;
  m_hand_frame_control = get_node()->get_parameter("hand_frame_control").as_bool();

  if (m_hand_frame_control)  // Assume end-effector frame by convention
  {
    target_wrench = Base::displayInBaseLink(m_target_wrench, Base::m_end_effector_link);
  }
  else  // Default to robot base frame
  {
    target_wrench = m_target_wrench;
  }

  // Superimpose target wrench and sensor wrench in base frame
  return Base::displayInBaseLink(m_ft_sensor_wrench, m_new_ft_sensor_ref) + target_wrench;
}

void CartesianForceController::setFtSensorReferenceFrame(const std::string & new_ref)
{
  // Compute static transform from the force torque sensor to the new reference
  // frame of interest.
  m_new_ft_sensor_ref = new_ref;

  // Joint positions should cancel out, i.e. it doesn't matter as long as they
  // are the same for both transformations.
  KDL::JntArray jnts(Base::m_ik_solver->getPositions());

  KDL::Frame sensor_ref;
  Base::m_forward_kinematics_solver->JntToCart(jnts, sensor_ref, m_ft_sensor_ref_link);

  KDL::Frame new_sensor_ref;
  Base::m_forward_kinematics_solver->JntToCart(jnts, new_sensor_ref, m_new_ft_sensor_ref);

  m_ft_sensor_transform = new_sensor_ref.Inverse() * sensor_ref;
}

void CartesianForceController::targetWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  if (!this->isActive())
  {
    return;
  }

  if (std::isnan(wrench->wrench.force.x) || std::isnan(wrench->wrench.force.y) ||
      std::isnan(wrench->wrench.force.z) || std::isnan(wrench->wrench.torque.x) ||
      std::isnan(wrench->wrench.torque.y) || std::isnan(wrench->wrench.torque.z))
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                "NaN detected in target wrench. Ignoring input.");
    return;
  }

  m_target_wrench[0] = wrench->wrench.force.x;
  m_target_wrench[1] = wrench->wrench.force.y;
  m_target_wrench[2] = wrench->wrench.force.z;
  m_target_wrench[3] = wrench->wrench.torque.x;
  m_target_wrench[4] = wrench->wrench.torque.y;
  m_target_wrench[5] = wrench->wrench.torque.z;
}

void CartesianForceController::ftSensorWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  if (!this->isActive())
  {
    return;
  }

  // 检查是否需要自动调零
  auto current_time = get_node()->get_clock()->now();
  if ((current_time - m_last_calibration_time).seconds() > m_auto_calibration_interval)
  {
    initializeCalibration();
    m_last_calibration_time = current_time;
  }

  if (std::isnan(wrench->wrench.force.x) || std::isnan(wrench->wrench.force.y) ||
      std::isnan(wrench->wrench.force.z) || std::isnan(wrench->wrench.torque.x) ||
      std::isnan(wrench->wrench.torque.y) || std::isnan(wrench->wrench.torque.z))
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                "NaN detected in force-torque sensor wrench. Ignoring input.");
    return;
  }

  // KDL::Wrench tmp;
  // tmp[0] = wrench->wrench.force.x;
  // tmp[1] = wrench->wrench.force.y;
  // tmp[2] = wrench->wrench.force.z;
  // tmp[3] = wrench->wrench.torque.x;
  // tmp[4] = wrench->wrench.torque.y;
  // tmp[5] = wrench->wrench.torque.z;

  // 将力数据转换为向量
  ctrl::Vector6D wrench_vector;
  wrench_vector[0] = wrench->wrench.force.x;
  wrench_vector[1] = wrench->wrench.force.y;
  wrench_vector[2] = wrench->wrench.force.z;
  wrench_vector[3] = wrench->wrench.torque.x;
  wrench_vector[4] = wrench->wrench.torque.y;
  wrench_vector[5] = wrench->wrench.torque.z;

  // 应用滤波器
  ctrl::Vector6D filtered_wrench = lowPassFilter(wrench_vector);

  // 转换为KDL::Wrench
  KDL::Wrench tmp;
  for (int i = 0; i < 6; ++i)
  {
    tmp[i] = filtered_wrench[i];
  }

  // 如果正在调零，更新校准数据
  if (m_is_calibrating)
  {
    updateCalibration(tmp);
    return;
  }

  // 减去零偏置
  for (int i = 0; i < 6; ++i)
  {
    tmp[i] -= m_force_bias[i];
  }

  // 应用死区
  tmp = applyDeadzone(tmp);

  // Compute how the measured wrench appears in the frame of interest.
  tmp = m_ft_sensor_transform * tmp;

  m_ft_sensor_wrench[0] = tmp[0];
  m_ft_sensor_wrench[1] = tmp[1];
  m_ft_sensor_wrench[2] = tmp[2];
  m_ft_sensor_wrench[3] = tmp[3];
  m_ft_sensor_wrench[4] = tmp[4];
  m_ft_sensor_wrench[5] = tmp[5];

  // 发布校准数据
  geometry_msgs::msg::WrenchStamped msg;
  msg.header.stamp = get_node()->now();
  msg.wrench.force.x = tmp[0];
  msg.wrench.force.y = tmp[1];
  msg.wrench.force.z = tmp[2];
  msg.wrench.torque.x = tmp[3];
  msg.wrench.torque.y = tmp[4];
  msg.wrench.torque.z = tmp[5];
  m_calib_wrench_publisher->publish(msg);
  
}

void CartesianForceController::initializeCalibration()
{
  RCLCPP_INFO(get_node()->get_logger(), "Starting force sensor calibration...");
  m_is_calibrating = true;
  m_calibration_data.clear();
}

void CartesianForceController::updateCalibration(const KDL::Wrench& wrench)
{
  // 收集校准数据
  ctrl::Vector6D data;
  for (int i = 0; i < 6; ++i)
  {
    data[i] = wrench[i];
  }
  m_calibration_data.push_back(data);

  // 检查是否收集了足够的样本
  if (m_calibration_data.size() >= m_calibration_samples)
  {
    // 计算平均值作为零偏置
    m_force_bias.setZero();
    for (const auto& data : m_calibration_data)
    {
      m_force_bias += data;
    }
    m_force_bias /= m_calibration_data.size();

    m_is_calibrating = false;
    m_calibration_data.clear();
    RCLCPP_INFO(get_node()->get_logger(), "Force sensor calibration completed");
  }
}

KDL::Wrench CartesianForceController::applyDeadzone(const KDL::Wrench& wrench)
{
  KDL::Wrench result = wrench;
  
  // 应用力的死区
  for (int i = 0; i < 3; ++i)
  {
    if (std::abs(result[i]) < m_force_deadzone){result[i] = 0.0;}
  }
  
  // 应用力矩的死区
  for (int i = 3; i < 6; ++i)
  {
    if (std::abs(result[i]) < m_torque_deadzone){result[i] = 0.0;}
  }
  
  return result;
}

void CartesianForceController::initializeFilter(const ctrl::Vector6D& initial_value)
{
  m_filtered_wrench = initial_value;
  m_filter_initialized = true;
}

ctrl::Vector6D CartesianForceController::lowPassFilter(const ctrl::Vector6D& input)
{
  if (!m_filter_initialized)
  {
    initializeFilter(input);
    return input;
  }

  // 一阶低通滤波
  // y(n) = α * x(n) + (1-α) * y(n-1)
  // 其中α是滤波系数，x(n)是当前输入，y(n-1)是上一次的输出
  for (int i = 0; i < 6; ++i)
  {
    m_filtered_wrench[i] = m_filter_coefficient * input[i] + 
                          (1.0 - m_filter_coefficient) * m_filtered_wrench[i];
  }

  return m_filtered_wrench;
}

}  // namespace cartesian_force_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_force_controller::CartesianForceController,
                       controller_interface::ControllerInterface)
