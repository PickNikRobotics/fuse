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
#ifndef FUSE_MODELS__PARAMETERS__APRIL_TAG_POSE_PARAMS_HPP_
#define FUSE_MODELS__PARAMETERS__APRIL_TAG_POSE_PARAMS_HPP_

#include <string>
#include <vector>

#include <fuse_models/parameters/parameter_base.hpp>

#include <fuse_core/loss.hpp>
#include <fuse_core/parameter.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>
#include <fuse_variables/position_3d_stamped.hpp>

namespace fuse_models::parameters
{

/**
 * @brief Defines the set of parameters required by the TransformSensor class
 */
struct TransformSensorParams : public ParameterBase
{
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] interfaces - The node interfaces with which to load parameters
   * @param[in] ns - The parameter namespace to use
   */
  void loadFromROS(
      fuse_core::node_interfaces::NodeInterfaces<fuse_core::node_interfaces::Base, fuse_core::node_interfaces::Logging,
                                                 fuse_core::node_interfaces::Parameters>
          interfaces,
      const std::string& ns)
  {
    position_indices = loadSensorConfig<fuse_variables::Position3DStamped>(
        interfaces, fuse_core::joinParameterName(ns, "position_dimensions"));
    orientation_indices = loadSensorConfig<fuse_variables::Orientation3DStamped>(
        interfaces, fuse_core::joinParameterName(ns, "orientation_dimensions"));

    disable_checks =
        fuse_core::getParam(interfaces, fuse_core::joinParameterName(ns, "disable_checks"), disable_checks);
    queue_size = fuse_core::getParam(interfaces, fuse_core::joinParameterName(ns, "queue_size"), queue_size);
    fuse_core::getPositiveParam(interfaces, fuse_core::joinParameterName(ns, "tf_timeout"), tf_timeout, false);

    fuse_core::getPositiveParam(interfaces, fuse_core::joinParameterName(ns, "throttle_period"), throttle_period, false);
    throttle_use_wall_time = fuse_core::getParam(interfaces, fuse_core::joinParameterName(ns, "throttle_use_wall_time"),
                                                 throttle_use_wall_time);

    fuse_core::getParamRequired(interfaces, fuse_core::joinParameterName(ns, "topic"), topic);

    target_frame = fuse_core::getParam(interfaces, fuse_core::joinParameterName(ns, "target_frame"), target_frame);

    pose_loss = fuse_core::loadLossConfig(interfaces, fuse_core::joinParameterName(ns, "pose_loss"));
    pose_covariance = fuse_core::getParam(interfaces, fuse_core::joinParameterName(ns, "pose_covariance"),
                                          std::vector<double>{ 1, 1, 1, 1, 1, 1 });
  }

  bool disable_checks{ false };
  bool independent{ true };
  fuse_core::Matrix6d minimum_pose_relative_covariance;  //!< Minimum pose relative covariance matrix
  rclcpp::Duration tf_timeout{ 0, 0 };       //!< The maximum time to wait for a transform to become  available
  rclcpp::Duration throttle_period{ 0, 0 };  //!< The throttle period duration in seconds
  bool throttle_use_wall_time{ false };      //!< Whether to throttle using ros::WallTime or not
  std::vector<double> pose_covariance;       //!< The diagonal elements of the tag pose covariance
  int queue_size{ 10 };
  std::string topic;
  std::string target_frame;
  std::vector<size_t> position_indices;
  std::vector<size_t> orientation_indices;
  fuse_core::Loss::SharedPtr pose_loss;
};

}  // namespace fuse_models::parameters

#endif  // FUSE_MODELS__PARAMETERS__APRIL_TAG_POSE_PARAMS_HPP_
