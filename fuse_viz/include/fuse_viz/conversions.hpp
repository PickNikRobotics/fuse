/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Clearpath Robotics
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

#ifndef FUSE_VIZ__CONVERSIONS_HPP_
#define FUSE_VIZ__CONVERSIONS_HPP_

#include <OgreColourValue.h>
#include <OgreQuaternion.h>
#include <Ogre.h>

#include <Eigen/Dense>

#include <array>
#include <stdexcept>

#include <fuse_core/graph.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_variables/orientation_2d_stamped.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

namespace tf2
{

/**
 * @brief Convert a 3x3 covariance matrix into a 6x6 covariance message.
 * @param[in]  covariance 3x3 covariance matrix.
 * @param[out] msg        6x6 covariance message, which is stored as a plain array with 36 elements.
 */
template <typename Derived>
inline void toMsg(Eigen::MatrixBase<Derived> const& covariance, std::array<double, 36>& msg)
{
  using Scalar = typename Derived::Scalar;
  using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;

  Eigen::Map<Matrix6> C(msg.data());
  C.template topLeftCorner<2, 2>() = covariance.template topLeftCorner<2, 2>();
  C(5, 5) = covariance(2, 2);

  C(0, 5) = covariance(0, 2);
  C(1, 5) = covariance(1, 2);
  C(5, 0) = covariance(2, 0);
  C(5, 1) = covariance(2, 1);
}

}  // namespace tf2

namespace
{

inline tf2::Vector3 toTF(fuse_variables::Position2DStamped const& position)
{
  return { position.x(), position.y(), 0 };
}

inline tf2::Quaternion toTF(fuse_variables::Orientation2DStamped const& orientation)
{
  return { tf2::Vector3{ 0, 0, 1 }, orientation.yaw() };
}

inline tf2::Transform toTF(fuse_variables::Position2DStamped const& position,
                           fuse_variables::Orientation2DStamped const& orientation)
{
  return tf2::Transform(toTF(orientation), toTF(position));
}

inline Ogre::Vector3 toOgre(tf2::Vector3 const& position)
{
  return { static_cast<float>(position.x()), static_cast<float>(position.y()), static_cast<float>(position.z()) };
}

inline Ogre::Quaternion toOgre(tf2::Quaternion const& orientation)
{
  return { static_cast<float>(orientation.w()), static_cast<float>(orientation.x()),  // NOLINT
           static_cast<float>(orientation.y()), static_cast<float>(orientation.z()) };
}

inline Ogre::Vector3 toOgre(fuse_variables::Position2DStamped const& position)
{
  return { static_cast<float>(position.x()), static_cast<float>(position.y()), 0 };
}

inline Ogre::Quaternion toOgre(fuse_variables::Orientation2DStamped const& orientation)
{
  return toOgre(toTF(orientation));
}

}  // namespace

namespace
{

inline tf2::Transform getPose(fuse_variables::Position2DStamped const& position,
                              fuse_variables::Orientation2DStamped const& orientation)
{
  return tf2::Transform(toTF(orientation), toTF(position));
}

inline tf2::Transform getPose(fuse_core::Graph const& graph, fuse_core::UUID const& position_uuid,
                              fuse_core::UUID const& orientation_uuid)
{
  auto const position = dynamic_cast<fuse_variables::Position2DStamped const*>(&graph.getVariable(position_uuid));
  if (!position)
  {
    throw std::runtime_error("Failed to get variable " + fuse_core::uuid::to_string(position_uuid) +
                             " from graph as fuse_variables::Position2DStamped.");
  }

  auto const orientation =
      dynamic_cast<fuse_variables::Orientation2DStamped const*>(&graph.getVariable(orientation_uuid));
  if (!orientation)
  {
    throw std::runtime_error("Failed to get variable " + fuse_core::uuid::to_string(orientation_uuid) +
                             " from graph as fuse_variables::Orientation2DStamped.");
  }

  return getPose(*position, *orientation);
}

}  // namespace
#endif  // FUSE_VIZ__CONVERSIONS_HPP_
