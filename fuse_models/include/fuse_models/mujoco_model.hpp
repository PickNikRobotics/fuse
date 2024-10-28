#include <mujoco/mujoco.h>
#include <fuse_core/timestamp_manager.hpp>

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
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

#pragma once

#include <fuse_core/async_motion_model.hpp>
#include <fuse_core/node_interfaces/node_interfaces.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fuse_models
{

/**
 * @brief A motion model base class that provides an internal callback queue and executor.
 *
 * A model model plugin is responsible for generating constraints that link together timestamps
 * introduced by other sensors in the system. The MujocoMotionModel class is designed similar to a
 * nodelet, attempting to be as generic and easy to use as a standard ROS node.
 *
 * There are a few notable differences between the MujocoMotionModel class and a standard ROS
 * node. First and most obvious, the MujocoMotionModel class is designed as a plugin, with all of
 * the stipulations and requirements that come with all ROS plugins (must be derived from a known
 * base class, will be default constructed). Second, the MujocoMotionModel class provides an internal
 * node that is hooked to a local callback queue and local executor on init. This makes
 * it act like a full ROS node -- subscriptions trigger message callbacks, callbacks will fire
 * sequentially, etc. However, authors of derived classes should be aware of this fact and avoid
 * creating additional sub-nodes, or at least take care when creating new sub-nodes and
 * additional callback queues. Finally, the interaction of motion model nodes is best compared to
 * a service call -- an external actor will provide a set of timestamps and wait for the motion
 * model to respond with the required set of constraints to link the timestamps together (along
 * with any previously existing timestamps). In lieu of a ROS service callback function, the
 * MujocoMotionModel class requires the applyCallback() function to be implemented. This callback
 * will be executed from the same callback queue as any other subscriptions or service callbacks.
 *
 * Derived classes:
 * - _probably_ need to implement the onInit() method. This method is used to configure the motion
 *    model for operation. This includes things like accessing the parameter server and subscribing
 *    to sensor topics.
 * - _must_ implement the applyCallback() method. This is the communication mechanism between the
 *    parent/optimizer and the derived motion model. This is how the optimizer tells the motion
 *    model what timestamps have been added, and how the motion model sends motion model constraints
 *    to the optimizer.
 * - may _optionally_ implement the onGraphUpdate() method. This should only be done if the derived
 *   motion model needs access to the latest values of the state variables. In many cases, motion
 *   models will simply not need that information. If the motion model does need access the to
 *   graph, the most common implementation will simply be to move the provided pointer into a class
 *   member variable, for use in other callbacks.
 */
class MujocoMotionModel : public fuse_core::AsyncMotionModel
{
public:
  FUSE_SMART_PTR_ALIASES_ONLY(MujocoMotionModel)

  /**
   * @brief Destructor
   */
  ~MujocoMotionModel() = default;
  MujocoMotionModel(MujocoMotionModel const&) = delete;
  MujocoMotionModel(MujocoMotionModel&&) = delete;
  MujocoMotionModel& operator=(MujocoMotionModel&&) = delete;
  MujocoMotionModel& operator=(MujocoMotionModel const&) = delete;

protected:
  /**
   * @brief Constructor
   *
   * Construct a new motion model and create a local callback queue and internal executor.
   *
   * @param[in] model The mujoco model
   */
  explicit MujocoMotionModel(std::unique_ptr<mjModel> model);

  /**
   * @brief Augment a transaction object such that all involved timestamps are connected by motion
   *        model constraints.
   *
   * This is not as straightforward as it would seem. Depending on the history of previously
   * generated constraints, fulfilling the request may require removing previously generated
   * constraints and creating several new constraints, such that all involved timestamps are
   * linked together in a sequential chain. This function is called by the MotionModel::apply()
   * function, but it is done in such a way that *this* function will run inside the derived
   * MujocoMotionModel's local callback queue. This function is roughly analogous to providing a
   * service callback, where the caller makes a request and blocks until the request is
   * completed.
   *
   * @param[in,out] transaction The transaction object that should be augmented with motion model
   *                            constraints
   * @return                    True if the motion models were generated successfully, false
   *                            otherwise
   */
  bool applyCallback(fuse_core::Transaction& transaction) override;

  void generateMotionModel(const rclcpp::Time& beginning_stamp, const rclcpp::Time& ending_stamp,
                           std::vector<fuse_core::Constraint::SharedPtr>& constraints,
                           std::vector<fuse_core::Variable::SharedPtr>& variables);

  /**
   * @brief Callback fired in the local callback queue thread(s) whenever a new Graph is received
   *        from the optimizer
   *
   * Receiving a new Graph object generally means that new variables have been inserted into
   * the Graph, and new optimized values are available. To simplify synchronization between the
   * sensor models and other consumers of Graph data, the provided Graph object will never be
   * updated be updated by anyone. Thus, only read access to the Graph is provided. Information
   * may be accessed or computed, but it cannot be changed. The optimizer provides the sensors
   * with Graph updates by sending a new Graph object, not by modifying the Graph object.
   *
   * If the derived sensor model does not need access to the Graph object, there is not reason
   * to overload this empty implementation.
   *
   * @param[in] graph A read-only pointer to the graph object, allowing queries to be performed
   *                  whenever needed.
   */
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph) override;

  /**
   * @brief Perform any required initialization for the motion model
   *
   * This could include things like reading from the parameter server or subscribing to topics.
   * The class's node will be properly initialized before onInit() is called. Spinning
   * of the callback queue will not begin until after the call to onInit() completes.
   */
  void onInit() override;

  rclcpp::Clock::SharedPtr clock_;                 //!< The sensor model's clock, for timestamping and logging
  rclcpp::Logger logger_;                          //!< The sensor model's logger
  fuse_core::UUID device_id_;                      //!< The UUID of the device to be published
  fuse_core::TimestampManager timestamp_manager_;  //!< Tracks timestamps and previously created
                                                   //!< motion model segments
  std::shared_ptr<fuse_core::Graph const> graph_;
  std::unique_ptr<mjModel> model_;
};

}  // namespace fuse_models
