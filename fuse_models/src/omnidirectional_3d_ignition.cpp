/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Giacomo Franchini
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <Eigen/Dense>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>

#include <exception>
#include <stdexcept>

#include <fuse_constraints/absolute_constraint.hpp>
#include <fuse_constraints/absolute_orientation_3d_stamped_constraint.hpp>
#include <fuse_core/async_sensor_model.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/sensor_model.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_core/util.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_models/omnidirectional_3d_ignition.hpp>
#include <fuse_msgs/srv/set_pose.hpp>
#include <fuse_msgs/srv/set_pose_deprecated.hpp>
#include <fuse_variables/acceleration_linear_3d_stamped.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>
#include <fuse_variables/position_3d_stamped.hpp>
#include <fuse_variables/stamped.hpp>
#include <fuse_variables/velocity_angular_3d_stamped.hpp>
#include <fuse_variables/velocity_linear_3d_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Register this motion model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::Omnidirectional3DIgnition, fuse_core::SensorModel);

namespace fuse_models
{

Omnidirectional3DIgnition::Omnidirectional3DIgnition()
  : fuse_core::AsyncSensorModel(1)
  , started_(false)
  , initial_transaction_sent_(false)
  , device_id_(fuse_core::uuid::NIL)
  , logger_(rclcpp::get_logger("uninitialized"))
{
}

void Omnidirectional3DIgnition::initialize(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces, const std::string& name,
    fuse_core::TransactionCallback transaction_callback)
{
  interfaces_ = interfaces;
  fuse_core::AsyncSensorModel::initialize(interfaces, name, transaction_callback);
}

void Omnidirectional3DIgnition::onInit()
{
  logger_ = interfaces_.get_node_logging_interface()->get_logger();
  clock_ = interfaces_.get_node_clock_interface()->get_clock();

  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(interfaces_);

  params_.loadFromROS(interfaces_, name_);

  // Connect to the reset service
  if (!params_.reset_service.empty())
  {
    reset_client_ = rclcpp::create_client<std_srvs::srv::Empty>(
        interfaces_.get_node_base_interface(), interfaces_.get_node_graph_interface(),
        interfaces_.get_node_services_interface(), params_.reset_service, rclcpp::ServicesQoS(), cb_group_);
  }

  // Advertise
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_;
  sub_ = rclcpp::create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      interfaces_, params_.topic, params_.queue_size,
      std::bind(&Omnidirectional3DIgnition::subscriberCallback, this, std::placeholders::_1), sub_options);

  set_pose_service_ = rclcpp::create_service<fuse_msgs::srv::SetPose>(
      interfaces_.get_node_base_interface(), interfaces_.get_node_services_interface(),
      fuse_core::joinTopicName(interfaces_.get_node_base_interface()->get_name(), params_.set_pose_service),
      std::bind(&Omnidirectional3DIgnition::setPoseServiceCallback, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3),
      rclcpp::ServicesQoS(), cb_group_);
  set_pose_deprecated_service_ = rclcpp::create_service<fuse_msgs::srv::SetPoseDeprecated>(
      interfaces_.get_node_base_interface(), interfaces_.get_node_services_interface(),
      fuse_core::joinTopicName(interfaces_.get_node_base_interface()->get_name(), params_.set_pose_deprecated_service),
      std::bind(&Omnidirectional3DIgnition::setPoseDeprecatedServiceCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3),
      rclcpp::ServicesQoS(), cb_group_);
}

void Omnidirectional3DIgnition::start()
{
  started_ = true;

  // TODO(swilliams) Should this be executed every time optimizer.reset() is called, or only once
  //                 ever? I feel like it should be "only once ever". Send an initial state
  //                 transaction immediately, if requested
  if (params_.publish_on_startup && !initial_transaction_sent_)
  {
    auto pose = geometry_msgs::msg::PoseWithCovarianceStamped();
    tf2::Quaternion q;
    // q.setRPY(params_.initial_state[3], params_.initial_state[4], params_.initial_state[5]);
    q.setEuler(params_.initial_state[5], params_.initial_state[4], params_.initial_state[3]);
    pose.header.stamp = clock_->now();
    pose.pose.pose.position.x = params_.initial_state[0];
    pose.pose.pose.position.y = params_.initial_state[1];
    pose.pose.pose.position.z = params_.initial_state[2];
    pose.pose.pose.orientation.x = q.x();
    pose.pose.pose.orientation.y = q.y();
    pose.pose.pose.orientation.z = q.z();
    pose.pose.pose.orientation.w = q.w();
    for (size_t i = 0; i < 6; i++)
    {
      pose.pose.covariance[i * 7] = params_.initial_sigma[i] * params_.initial_sigma[i];
    }
    sendPrior(pose);
    initial_transaction_sent_ = true;
  }
}

void Omnidirectional3DIgnition::stop()
{
  started_ = false;
}

void Omnidirectional3DIgnition::subscriberCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
  try
  {
    process(msg);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(logger_, e.what() << " Ignoring message.");
  }
}

bool Omnidirectional3DIgnition::setPoseServiceCallback(rclcpp::Service<fuse_msgs::srv::SetPose>::SharedPtr service,
                                                       std::shared_ptr<rmw_request_id_t> request_id,
                                                       const fuse_msgs::srv::SetPose::Request::SharedPtr req)
{
  try
  {
    process(req->pose, [service, request_id]() {
      fuse_msgs::srv::SetPose::Response response;
      response.success = true;
      service->send_response(*request_id, response);
    });
  }
  catch (const std::exception& e)
  {
    fuse_msgs::srv::SetPose::Response response;
    response.success = false;
    response.message = e.what();
    RCLCPP_ERROR_STREAM(logger_, e.what() << " Ignoring request.");
    service->send_response(*request_id, response);
  }
  return true;
}

bool Omnidirectional3DIgnition::setPoseDeprecatedServiceCallback(
    rclcpp::Service<fuse_msgs::srv::SetPoseDeprecated>::SharedPtr service, std::shared_ptr<rmw_request_id_t> request_id,
    const fuse_msgs::srv::SetPoseDeprecated::Request::SharedPtr req)
{
  try
  {
    process(req->pose, [service, request_id]() {
      fuse_msgs::srv::SetPoseDeprecated::Response response;
      service->send_response(*request_id, response);
    });
  }
  catch (const std::exception& e)
  {
    fuse_msgs::srv::SetPoseDeprecated::Response response;
    RCLCPP_ERROR_STREAM(logger_, e.what() << " Ignoring request.");
    service->send_response(*request_id, response);
  }
  return true;
}

void Omnidirectional3DIgnition::process(const geometry_msgs::msg::PoseWithCovarianceStamped& pose,
                                        std::function<void()> post_process)
{
  // Verify we are in the correct state to process set pose requests
  if (!started_)
  {
    throw std::runtime_error("Attempting to set the pose while the sensor is stopped.");
  }
  // Validate the requested pose and covariance before we do anything
  if (!std::isfinite(pose.pose.pose.position.x) || !std::isfinite(pose.pose.pose.position.y) ||
      !std::isfinite(pose.pose.pose.position.z))
  {
    throw std::invalid_argument(
        "Attempting to set the pose to an invalid position (" + std::to_string(pose.pose.pose.position.x) + ", " +
        std::to_string(pose.pose.pose.position.y) + ", " + std::to_string(pose.pose.pose.position.z) + ").");
  }
  auto orientation_norm = std::sqrt(pose.pose.pose.orientation.x * pose.pose.pose.orientation.x +
                                    pose.pose.pose.orientation.y * pose.pose.pose.orientation.y +
                                    pose.pose.pose.orientation.z * pose.pose.pose.orientation.z +
                                    pose.pose.pose.orientation.w * pose.pose.pose.orientation.w);
  if (std::abs(orientation_norm - 1.0) > 1.0e-3)
  {
    throw std::invalid_argument(
        "Attempting to set the pose to an invalid orientation (" + std::to_string(pose.pose.pose.orientation.x) + ", " +
        std::to_string(pose.pose.pose.orientation.y) + ", " + std::to_string(pose.pose.pose.orientation.z) + ", " +
        std::to_string(pose.pose.pose.orientation.w) + ").");
  }
  auto position_cov = fuse_core::Matrix3d();
  // for (size_t i = 0; i < 3; i++) {
  //   for (size_t j = 0; j < 3; j++) {
  //     position_cov(i, j) = pose.pose.covariance[i * 6 + j];
  //   }
  // }
  position_cov << pose.pose.covariance[0], pose.pose.covariance[1], pose.pose.covariance[2], pose.pose.covariance[6],
      pose.pose.covariance[7], pose.pose.covariance[8], pose.pose.covariance[12], pose.pose.covariance[13],
      pose.pose.covariance[14];
  if (!fuse_core::isSymmetric(position_cov))
  {
    throw std::invalid_argument("Attempting to set the pose with a non-symmetric position covariance matrix\n " +
                                fuse_core::to_string(position_cov, Eigen::FullPrecision) + ".");
  }
  if (!fuse_core::isPositiveDefinite(position_cov))
  {
    throw std::invalid_argument("Attempting to set the pose with a non-positive-definite position covariance matrix\n" +
                                fuse_core::to_string(position_cov, Eigen::FullPrecision) + ".");
  }
  auto orientation_cov = fuse_core::Matrix3d();
  // for (size_t i = 0; i < 3; i++) {
  //   for (size_t j = 0; j < 3; j++) {
  //     position_cov(i, j) = pose.pose.covariance[3+i * 6 + j];
  //   }
  // }
  orientation_cov << pose.pose.covariance[21], pose.pose.covariance[22], pose.pose.covariance[23],
      pose.pose.covariance[27], pose.pose.covariance[28], pose.pose.covariance[29], pose.pose.covariance[33],
      pose.pose.covariance[34], pose.pose.covariance[35];
  if (!fuse_core::isSymmetric(orientation_cov))
  {
    throw std::invalid_argument("Attempting to set the pose with a non-symmetric orientation covariance matrix\n " +
                                fuse_core::to_string(orientation_cov, Eigen::FullPrecision) + ".");
  }
  if (!fuse_core::isPositiveDefinite(orientation_cov))
  {
    throw std::invalid_argument(
        "Attempting to set the pose with a non-positive-definite orientation_cov covariance matrix\n" +
        fuse_core::to_string(orientation_cov, Eigen::FullPrecision) + ".");
  }
  // Tell the optimizer to reset before providing the initial state
  if (!params_.reset_service.empty())
  {
    // Wait for the reset service
    while (!reset_client_->wait_for_service(std::chrono::seconds(10)) &&
           interfaces_.get_node_base_interface()->get_context()->is_valid())
    {
      RCLCPP_WARN_STREAM(logger_,
                         "Waiting for '" << reset_client_->get_service_name() << "' service to become avaiable.");
    }

    auto srv = std::make_shared<std_srvs::srv::Empty::Request>();
    // Don't block the executor.
    // It needs to be free to handle the response to this service call.
    // Have a callback do the rest of the work when a response comes.
    auto result_future = reset_client_->async_send_request(
        srv, [this, post_process, pose](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture result) {
          (void)result;
          // Now that the pose has been validated and the optimizer has been reset, actually send the
          // initial state constraints to the optimizer
          sendPrior(pose);
          if (post_process)
          {
            post_process();
          }
        });
  }
  else
  {
    sendPrior(pose);
    if (post_process)
    {
      post_process();
    }
  }
}

void Omnidirectional3DIgnition::sendPrior(const geometry_msgs::msg::PoseWithCovarianceStamped& pose)
{
  const auto& stamp = pose.header.stamp;

  // Create variables for the full state.
  // The initial values of the pose are extracted from the provided PoseWithCovarianceStamped
  // message. The remaining dimensions are provided as parameters to the parameter server.
  auto position = fuse_variables::Position3DStamped::make_shared(stamp, device_id_);
  position->x() = pose.pose.pose.position.x;
  position->y() = pose.pose.pose.position.y;
  position->z() = pose.pose.pose.position.z;
  auto orientation = fuse_variables::Orientation3DStamped::make_shared(stamp, device_id_);
  orientation->w() = pose.pose.pose.orientation.w;
  orientation->x() = pose.pose.pose.orientation.x;
  orientation->y() = pose.pose.pose.orientation.y;
  orientation->z() = pose.pose.pose.orientation.z;
  auto linear_velocity = fuse_variables::VelocityLinear3DStamped::make_shared(stamp, device_id_);
  linear_velocity->x() = params_.initial_state[6];
  linear_velocity->y() = params_.initial_state[7];
  linear_velocity->z() = params_.initial_state[8];
  auto angular_velocity = fuse_variables::VelocityAngular3DStamped::make_shared(stamp, device_id_);
  angular_velocity->roll() = params_.initial_state[9];
  angular_velocity->pitch() = params_.initial_state[10];
  angular_velocity->yaw() = params_.initial_state[11];
  auto linear_acceleration = fuse_variables::AccelerationLinear3DStamped::make_shared(stamp, device_id_);
  linear_acceleration->x() = params_.initial_state[12];
  linear_acceleration->y() = params_.initial_state[13];
  linear_acceleration->z() = params_.initial_state[14];

  // Create the covariances for each variable
  // The pose covariances are extracted from the provided PoseWithCovarianceStamped message.
  // The remaining covariances are provided as parameters to the parameter server.
  auto position_cov = fuse_core::Matrix3d();
  auto orientation_cov = fuse_core::Matrix3d();
  auto linear_velocity_cov = fuse_core::Matrix3d();
  auto angular_velocity_cov = fuse_core::Matrix3d();
  auto linear_acceleration_cov = fuse_core::Matrix3d();

  // for (size_t i = 0; i < 3; i++) {
  //   for (size_t j = 0; j < 3; j++) {
  //     position_cov(i, j) = pose.pose.covariance[i * 6 + j];
  //     orientation_cov(i, j) = pose.pose.covariance[(i + 3) * 6 + j + 3];
  //     if (i == j) {
  //       linear_velocity_cov(i, j) = params_.initial_sigma[6 + i] * params_.initial_sigma[6 + i];
  //       angular_velocity_cov(i, j) = params_.initial_sigma[9 + i] * params_.initial_sigma[9 + i];
  //       linear_acceleration_cov(i, j) = params_.initial_sigma[12 + i] * params_.initial_sigma[12 + i];
  //     }
  //   }
  // }

  position_cov << pose.pose.covariance[0], pose.pose.covariance[1], pose.pose.covariance[2], pose.pose.covariance[6],
      pose.pose.covariance[7], pose.pose.covariance[8], pose.pose.covariance[12], pose.pose.covariance[13],
      pose.pose.covariance[14];

  orientation_cov << pose.pose.covariance[21], pose.pose.covariance[22], pose.pose.covariance[23],
      pose.pose.covariance[27], pose.pose.covariance[28], pose.pose.covariance[29], pose.pose.covariance[33],
      pose.pose.covariance[34], pose.pose.covariance[35];

  linear_velocity_cov << params_.initial_sigma[6] * params_.initial_sigma[6], 0.0, 0.0, 0.0,
      params_.initial_sigma[7] * params_.initial_sigma[7], 0.0, 0.0, 0.0,
      params_.initial_sigma[8] * params_.initial_sigma[8];

  angular_velocity_cov << params_.initial_sigma[9] * params_.initial_sigma[9], 0.0, 0.0, 0.0,
      params_.initial_sigma[10] * params_.initial_sigma[10], 0.0, 0.0, 0.0,
      params_.initial_sigma[11] * params_.initial_sigma[11];

  linear_acceleration_cov << params_.initial_sigma[12] * params_.initial_sigma[12], 0.0, 0.0, 0.0,
      params_.initial_sigma[13] * params_.initial_sigma[13], 0.0, 0.0, 0.0,
      params_.initial_sigma[14] * params_.initial_sigma[14];
  // Create absolute constraints for each variable
  auto position_constraint = fuse_constraints::AbsolutePosition3DStampedConstraint::make_shared(
      name(), *position, fuse_core::Vector3d(position->x(), position->y(), position->z()), position_cov);
  auto orientation_constraint = fuse_constraints::AbsoluteOrientation3DStampedConstraint::make_shared(
      name(), *orientation, Eigen::Quaterniond(orientation->w(), orientation->x(), orientation->y(), orientation->z()),
      orientation_cov);
  auto linear_velocity_constraint = fuse_constraints::AbsoluteVelocityLinear3DStampedConstraint::make_shared(
      name(), *linear_velocity, fuse_core::Vector3d(linear_velocity->x(), linear_velocity->y(), linear_velocity->z()),
      linear_velocity_cov);
  auto angular_velocity_constraint = fuse_constraints::AbsoluteVelocityAngular3DStampedConstraint::make_shared(
      name(), *angular_velocity,
      fuse_core::Vector3d(angular_velocity->roll(), angular_velocity->pitch(), angular_velocity->yaw()),
      angular_velocity_cov);
  auto linear_acceleration_constraint = fuse_constraints::AbsoluteAccelerationLinear3DStampedConstraint::make_shared(
      name(), *linear_acceleration,
      fuse_core::Vector3d(linear_acceleration->x(), linear_acceleration->y(), linear_acceleration->z()),
      linear_acceleration_cov);

  // Create the transaction
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(stamp);
  transaction->addInvolvedStamp(stamp);
  transaction->addVariable(position);
  transaction->addVariable(orientation);
  transaction->addVariable(linear_velocity);
  transaction->addVariable(angular_velocity);
  transaction->addVariable(linear_acceleration);
  transaction->addConstraint(position_constraint);
  transaction->addConstraint(orientation_constraint);
  transaction->addConstraint(linear_velocity_constraint);
  transaction->addConstraint(angular_velocity_constraint);
  transaction->addConstraint(linear_acceleration_constraint);

  // Send the transaction to the optimizer.
  sendTransaction(transaction);

  RCLCPP_INFO_STREAM(logger_, "Received a set_pose request (stamp: "
                                  << rclcpp::Time(stamp).nanoseconds() << ", x: " << position->x() << ", y: "
                                  << position->y() << ", z: " << position->z() << ", roll: " << orientation->roll()
                                  << ", pitch: " << orientation->pitch() << ", yaw: " << orientation->yaw() << ")");
}
}  // namespace fuse_models