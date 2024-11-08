/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, PickNik Robotics
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
#include <tf2_ros/buffer.h>
#include <memory>

#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_models/common/sensor_proc.hpp>
#include <fuse_models/transform_sensor.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::TransformSensor, fuse_core::SensorModel)

namespace fuse_models
{

TransformSensor::TransformSensor()
  : fuse_core::AsyncSensorModel(1)
  , device_id_(fuse_core::uuid::NIL)
  , logger_(rclcpp::get_logger("uninitialized"))
  , throttled_callback_(std::bind(&TransformSensor::process, this, std::placeholders::_1))
{
}

void TransformSensor::initialize(fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
                                 const std::string& name, fuse_core::TransactionCallback transaction_callback)
{
  interfaces_ = interfaces;
  fuse_core::AsyncSensorModel::initialize(interfaces, name, transaction_callback);
}

void TransformSensor::onInit()
{
  logger_ = interfaces_.get_node_logging_interface()->get_logger();
  clock_ = interfaces_.get_node_clock_interface()->get_clock();

  // Read settings from the parameter server
  device_id_ = fuse_variables::loadDeviceId(interfaces_);

  params_.loadFromROS(interfaces_, name_);

  throttled_callback_.setThrottlePeriod(params_.throttle_period);

  if (!params_.throttle_use_wall_time)
  {
    throttled_callback_.setClock(clock_);
  }

  if (params_.position_indices.empty() && params_.orientation_indices.empty())
  {
    throw std::runtime_error("No dimensions specified, so this sensor would not do anything (data from topic " +
                             params_.topic + " would be ignored).");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, interfaces_.get_node_base_interface(),
                                                              interfaces_.get_node_logging_interface(),
                                                              interfaces_.get_node_parameters_interface(),
                                                              interfaces_.get_node_topics_interface());
}

void TransformSensor::onStart()
{
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_;

  sub_ = rclcpp::create_subscription<MessageType>(interfaces_, params_.topic, params_.queue_size,
                                                  std::bind(&AprilTagThrottledCallback::callback<const MessageType&>,
                                                            &throttled_callback_, std::placeholders::_1),
                                                  sub_options);
}

void TransformSensor::onStop()
{
  sub_.reset();
}

void TransformSensor::process(MessageType const& msg)
{
  for (auto const& transform : msg.transforms)
  {
    // Create a transaction object
    auto transaction = fuse_core::Transaction::make_shared();
    transaction->stamp(transform.header.stamp);

    // Create the pose from the transform
    auto pose = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pose->header = transform.header;
    pose->header.frame_id = pose->header.frame_id;
    pose->pose.pose.orientation = transform.transform.rotation;
    pose->pose.pose.position.x = transform.transform.translation.x;
    pose->pose.pose.position.y = transform.transform.translation.y;
    pose->pose.pose.position.z = transform.transform.translation.z;

    // TODO(henrygerardmoore): figure out better method to set the covariance
    for (std::size_t i = 0; i < params_.pose_covariance.size(); ++i)
    {
      pose->pose.covariance[i * 7] = params_.pose_covariance[i];
    }

    const bool validate = !params_.disable_checks;
    common::processAbsolutePose3DWithCovariance(name(), device_id_, *pose, params_.pose_loss, params_.target_frame,
                                                params_.position_indices, params_.orientation_indices, *tf_buffer_,
                                                validate, *transaction, params_.tf_timeout);

    // Send the transaction object to the plugin's parent
    sendTransaction(transaction);
  }
}

}  // namespace fuse_models
