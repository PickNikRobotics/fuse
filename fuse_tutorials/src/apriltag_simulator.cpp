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
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <cmath>
#include <fuse_core/eigen.hpp>
#include <Eigen/Core>
#include <memory>
#include <random>

#include <fuse_core/node_interfaces/node_interfaces.hpp>
#include <fuse_core/util.hpp>
#include <fuse_msgs/srv/set_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace
{
constexpr char baselinkFrame[] = "base_link";      //!< The base_link frame id used when
                                                   //!< publishing sensor data
constexpr char mapFrame[] = "map";                 //!< The map frame id used when publishing ground truth
                                                   //!< data
constexpr double aprilTagPositionSigma = 0.1;      //!< the april tag position std dev
constexpr double aprilTagOrientationSigma = 0.25;  //!< the april tag orientation std dev
constexpr size_t numAprilTags = 8;                 //!< the number of april tags
constexpr double detectionProbability =
    0.5;  //!< the probability that any given april tag is detectable on a given tick of the simulation
}  // namespace

/**
 * @brief The true pose and velocity of the robot
 */
struct Robot
{
  rclcpp::Time stamp;

  double mass{};

  double x = 0;
  double y = 0;
  double z = 0;
  double roll = 0;
  double pitch = 0;
  double yaw = 0;
  double vx = 0;
  double vy = 0;
  double vz = 0;
  double vroll = 0;
  double vpitch = 0;
  double vyaw = 0;
  double ax = 0;
  double ay = 0;
  double az = 0;
};

/**
 * @brief Convert the robot state into a ground truth odometry message
 */
nav_msgs::msg::Odometry robotToOdometry(Robot const& state)
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = state.stamp;
  msg.header.frame_id = mapFrame;
  msg.child_frame_id = baselinkFrame;
  msg.pose.pose.position.x = state.x;
  msg.pose.pose.position.y = state.y;
  msg.pose.pose.position.z = state.z;

  tf2::Quaternion q;
  q.setEuler(state.yaw, state.pitch, state.roll);
  msg.pose.pose.orientation.w = q.w();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.twist.twist.linear.x = state.vx;
  msg.twist.twist.linear.y = state.vy;
  msg.twist.twist.linear.z = state.vz;
  msg.twist.twist.angular.x = state.vroll;
  msg.twist.twist.angular.y = state.vpitch;
  msg.twist.twist.angular.z = state.vyaw;

  // set covariances
  msg.pose.covariance[0] = 0.1;
  msg.pose.covariance[7] = 0.1;
  msg.pose.covariance[14] = 0.1;
  msg.pose.covariance[21] = 0.1;
  msg.pose.covariance[28] = 0.1;
  msg.pose.covariance[35] = 0.1;
  msg.twist.covariance[0] = 0.1;
  msg.twist.covariance[7] = 0.1;
  msg.twist.covariance[14] = 0.1;
  msg.twist.covariance[21] = 0.1;
  msg.twist.covariance[28] = 0.1;
  msg.twist.covariance[35] = 0.1;
  return msg;
}

/**
 * @brief Compute the next robot state given the current robot state and a simulated step time
 */
Robot simulateRobotMotion(Robot const& previous_state, rclcpp::Time const& now, Eigen::Vector3d const& external_force)
{
  auto const dt = (now - previous_state.stamp).seconds();
  auto next_state = Robot();
  next_state.stamp = now;
  next_state.mass = previous_state.mass;

  // just euler integrate to get the next position and velocity
  next_state.x = previous_state.x + previous_state.vx * dt;
  next_state.y = previous_state.y + previous_state.vy * dt;
  next_state.z = previous_state.z + previous_state.vz * dt;
  next_state.vx = previous_state.vx + previous_state.ax * dt;
  next_state.vy = previous_state.vy + previous_state.ay * dt;
  next_state.vz = previous_state.vz + previous_state.az * dt;

  // let's not deal with 3D orientation dynamics for this tutorial
  next_state.roll = 0;
  next_state.pitch = 0;
  next_state.yaw = 0;
  next_state.vroll = 0;
  next_state.vpitch = 0;
  next_state.vyaw = 0;

  // get the current acceleration from the current applied force
  next_state.ax = external_force.x() / next_state.mass;
  next_state.ay = external_force.y() / next_state.mass;
  next_state.az = external_force.z() / next_state.mass;

  return next_state;
}

tf2_msgs::msg::TFMessage aprilTagPoses(Robot const& robot)
{
  tf2_msgs::msg::TFMessage msg;

  // publish the april tag positions relative to the base
  for (std::size_t i = 0; i < numAprilTags; ++i)
  {
    geometry_msgs::msg::TransformStamped april_to_base;
    april_to_base.child_frame_id = "base_link";

    // april tag names start at 1
    april_to_base.header.frame_id = "april_" + std::to_string(i + 1);
    april_to_base.header.stamp = robot.stamp;

    april_to_base.transform.rotation.w = 1;
    april_to_base.transform.rotation.x = 0;
    april_to_base.transform.rotation.y = 0;
    april_to_base.transform.rotation.z = 0;

    // calculate offset of each april tag
    // we start with offset 1, 1, 1 and switch the z, y, then x as if they were binary digits based off of the april tag
    // number see the launch file for a more readable offset for each april tag
    bool const x_positive = ((i >> 2) & 1) == 0u;
    bool const y_positive = ((i >> 1) & 1) == 0u;
    bool const z_positive = ((i >> 0) & 1) == 0u;

    // robot position with offset and noise
    april_to_base.transform.translation.x = x_positive ? 1. : -1.;
    april_to_base.transform.translation.y = y_positive ? 1. : -1.;
    april_to_base.transform.translation.z = z_positive ? 1. : -1.;
    msg.transforms.push_back(april_to_base);
  }
  return msg;
}

tf2_msgs::msg::TFMessage simulateAprilTag(const Robot& robot)
{
  static std::random_device rd{};
  static std::mt19937 generator{ rd() };
  static std::normal_distribution<> position_noise{ 0.0, aprilTagPositionSigma };
  static std::normal_distribution<> orientation_noise{ 0.0, aprilTagOrientationSigma };
  static std::bernoulli_distribution april_tag_detectable(detectionProbability);

  tf2_msgs::msg::TFMessage msg;

  // publish the april tag positions
  for (std::size_t i = 0; i < numAprilTags; ++i)
  {
    geometry_msgs::msg::TransformStamped april_to_world;
    april_to_world.child_frame_id = "odom";
    // april tag names start at 1
    april_to_world.header.frame_id = "april_" + std::to_string(i + 1);
    april_to_world.header.stamp = robot.stamp;
    tf2::Quaternion q;
    // robot orientation with noise
    q.setRPY(robot.roll + orientation_noise(generator), robot.pitch + orientation_noise(generator),
             robot.yaw + orientation_noise(generator));
    april_to_world.transform.rotation.w = q.w();
    april_to_world.transform.rotation.x = q.x();
    april_to_world.transform.rotation.y = q.y();
    april_to_world.transform.rotation.z = q.z();

    // calculate offset of each april tag
    // we start with offset 1, 1, 1 and switch the z, y, then x as if they were binary digits based off of the april tag
    // number see the launch file for a more readable offset for each april tag
    bool const x_positive = ((i >> 2) & 1) == 0u;
    bool const y_positive = ((i >> 1) & 1) == 0u;
    bool const z_positive = ((i >> 0) & 1) == 0u;

    double const x_offset = x_positive ? 1. : -1.;
    double const y_offset = y_positive ? 1. : -1.;
    double const z_offset = z_positive ? 1. : -1.;

    // robot position with offset and noise
    april_to_world.transform.translation.x = robot.x + x_offset + position_noise(generator);
    april_to_world.transform.translation.y = robot.y + y_offset + position_noise(generator);
    april_to_world.transform.translation.z = robot.z + z_offset + position_noise(generator);

    if (april_tag_detectable(generator))
    {
      msg.transforms.push_back(april_to_world);
    }
  }
  return msg;
}

void initializeStateEstimation(fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
                               const Robot& state, const rclcpp::Clock::SharedPtr& clock, const rclcpp::Logger& logger)
{
  // Send the initial localization signal to the state estimator
  auto srv = std::make_shared<fuse_msgs::srv::SetPose::Request>();
  srv->pose.header.frame_id = mapFrame;
  srv->pose.pose.pose.position.x = state.x;
  srv->pose.pose.pose.position.y = state.y;
  srv->pose.pose.pose.position.z = state.z;
  tf2::Quaternion q;
  q.setEuler(state.yaw, state.pitch, state.roll);
  srv->pose.pose.pose.orientation.w = q.w();
  srv->pose.pose.pose.orientation.x = q.x();
  srv->pose.pose.pose.orientation.y = q.y();
  srv->pose.pose.pose.orientation.z = q.z();
  srv->pose.pose.covariance[0] = 1.0;
  srv->pose.pose.covariance[7] = 1.0;
  srv->pose.pose.covariance[14] = 1.0;
  srv->pose.pose.covariance[21] = 1.0;
  srv->pose.pose.covariance[28] = 1.0;
  srv->pose.pose.covariance[35] = 1.0;

  auto const client = rclcpp::create_client<fuse_msgs::srv::SetPose>(
      interfaces.get_node_base_interface(), interfaces.get_node_graph_interface(),
      interfaces.get_node_services_interface(), "/state_estimation/set_pose_service", rclcpp::ServicesQoS());

  while (!client->wait_for_service(std::chrono::seconds(30)) &&
         interfaces.get_node_base_interface()->get_context()->is_valid())
  {
    RCLCPP_WARN_STREAM(logger, "Waiting for '" << client->get_service_name() << "' service to become available.");
  }

  auto success = false;
  while (!success)
  {
    clock->sleep_for(std::chrono::milliseconds(100));
    srv->pose.header.stamp = clock->now();
    auto result_future = client->async_send_request(srv);

    if (rclcpp::spin_until_future_complete(interfaces.get_node_base_interface(), result_future,
                                           std::chrono::seconds(1)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(logger, "set pose service call failed");
      client->remove_pending_request(result_future);
      return;
    }
    success = result_future.get()->success;
  }
}

int main(int argc, char** argv)
{
  // set up our ROS node
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("three_dimensional_simulator");

  // create our sensor publishers
  auto april_tf_publisher = node->create_publisher<tf2_msgs::msg::TFMessage>("april_tf", 1);
  auto tf_publisher = node->create_publisher<tf2_msgs::msg::TFMessage>("tf", 1);

  // create the ground truth publisher
  auto ground_truth_publisher = node->create_publisher<nav_msgs::msg::Odometry>("ground_truth", 1);
  auto predict_time_publisher = node->create_publisher<builtin_interfaces::msg::Time>("predict_time", 1);

  // Initialize the robot state (state variables are zero-initialized)
  auto state = Robot();
  state.stamp = node->now();
  state.mass = 10;  // kg

  // you can modify the rate at which this loop runs to see the different performance of the estimator and the effect of
  // integration inaccuracy on the ground truth
  auto rate = rclcpp::Rate(1000.0);

  // normally we would have to initialize the state estimation, but we included an ignition 'sensor' in our config,
  // which takes care of that.

  // parameters that control the motion pattern of the robot
  double const motion_duration = 5;  // length of time to oscillate on a given axis, in seconds
  double const n_cycles = 2;         // number of oscillations per `motion_duration`

  while (rclcpp::ok())
  {
    // store the first time this runs (since it won't start running exactly at a multiple of `motion_duration`)
    static auto const firstTime = node->now();
    auto const now = node->now();
    builtin_interfaces::msg::Time predict_time(now);

    // predict into the future
    predict_time.sec += 10;
    predict_time_publisher->publish(predict_time);

    // compensate for the original time offset
    double const now_d = (now - firstTime).seconds();
    // store how long it has been (resetting at `motion_duration` seconds)
    double const mod_time = std::fmod(now_d, motion_duration);

    // apply a harmonic force (oscillates `N_cycles` times per `motion_duration`)
    double const force_magnitude = 100 * std::cos(2 * M_PI * n_cycles * mod_time / motion_duration);
    Eigen::Vector3d external_force = { 0, 0, 0 };

    // switch oscillation axes every `motion_duration` seconds (with one 'rest period')
    if (std::fmod(now_d, 4 * motion_duration) < motion_duration)
    {
      external_force.x() = force_magnitude;
    }
    else if (std::fmod(now_d, 4 * motion_duration) < 2 * motion_duration)
    {
      external_force.y() = force_magnitude;
    }
    else if (std::fmod(now_d, 4 * motion_duration) < 3 * motion_duration)
    {
      external_force.z() = force_magnitude;
    }
    else
    {
      // reset the robot's position and velocity, leave the external force as 0
      // we do this so the ground truth doesn't drift (due to inaccuracy from euler integration)
      state.x = 0;
      state.y = 0;
      state.z = 0;
      state.vx = 0;
      state.vy = 0;
      state.vz = 0;
    }

    // Simulate the robot motion
    auto new_state = simulateRobotMotion(state, now, external_force);

    // Publish the new ground truth
    ground_truth_publisher->publish(robotToOdometry(new_state));

    // Generate and publish simulated measurements from the new robot state
    tf_publisher->publish(aprilTagPoses(new_state));

    // Wait for the next time step
    state = new_state;
    rclcpp::spin_some(node);
    rate.sleep();

    // publish simulated position after the static april tag poses since we need them to be in the tf buffer to run
    april_tf_publisher->publish(simulateAprilTag(new_state));
  }

  rclcpp::shutdown();
  return 0;
}
