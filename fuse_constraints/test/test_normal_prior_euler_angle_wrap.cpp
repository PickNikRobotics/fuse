/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, PickNik Robotics
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
#include <ceres/autodiff_cost_function.h>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <fuse_constraints/absolute_orientation_3d_stamped_euler_constraint.hpp>
#include <fuse_constraints/absolute_pose_3d_stamped_euler_constraint.hpp>
#include <fuse_constraints/normal_prior_pose_3d_euler_cost_functor.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/eigen_gtest.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>
#include <fuse_variables/position_3d_stamped.hpp>

namespace
{
using fuse_constraints::AbsoluteOrientation3DStampedEulerConstraint;
using fuse_constraints::AbsolutePose3DStampedEulerConstraint;
using fuse_constraints::NormalPriorPose3DEulerCostFunctor;
using fuse_variables::Orientation3DStamped;
using fuse_variables::Position3DStamped;

using AutoDiffNormalPriorPose3DEuler =
    ceres::AutoDiffCostFunction<NormalPriorPose3DEulerCostFunctor, ceres::DYNAMIC, 3, 4>;

fuse_core::Matrix3d makeSqrtInformation(bool correlated)
{
  if (!correlated)
  {
    return fuse_core::Matrix3d::Identity();
  }

  fuse_core::Matrix3d sqrt_information;
  sqrt_information << 2.0, 0.3, -0.4, 0.0, 1.5, 0.6, 0.0, 0.0, 3.0;
  return sqrt_information;
}

fuse_core::Vector6d makePoseMean(fuse_core::Vector3d const& mean_rpy)
{
  fuse_core::Vector6d mean = fuse_core::Vector6d::Zero();
  mean.tail<3>() = mean_rpy;
  return mean;
}

Eigen::Quaterniond makeQuaternion(fuse_core::Vector3d const& rpy)
{
  return Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX());
}

void expectPoseAngularResiduals(fuse_core::Vector3d const& state_rpy, fuse_core::Vector3d const& mean_rpy,
                                fuse_core::Vector3d const& expected_error, bool correlated)
{
  Position3DStamped const position(rclcpp::Time(1, 0));
  Orientation3DStamped const orientation(rclcpp::Time(1, 0));
  fuse_core::Vector6d const mean = makePoseMean(mean_rpy);
  fuse_core::Matrix3d const sqrt_information = makeSqrtInformation(correlated);
  fuse_core::Matrix3d const covariance = (sqrt_information.transpose() * sqrt_information).inverse();
  AbsolutePose3DStampedEulerConstraint const constraint("test", position, orientation, mean, covariance, { 3, 4, 5 });
  std::unique_ptr<ceres::CostFunction> const analytic(constraint.costFunction());
  AutoDiffNormalPriorPose3DEuler const autodiff(
      new NormalPriorPose3DEulerCostFunctor(constraint.sqrtInformation(), mean), 3);
  Eigen::Quaterniond const quaternion = makeQuaternion(state_rpy);
  fuse_core::Vector3d const expected_residual = sqrt_information * expected_error;

  for (double const sign : { 1.0, -1.0 })
  {
    SCOPED_TRACE(sign);
    double const signed_quaternion[] = { sign * quaternion.w(), sign * quaternion.x(), sign * quaternion.y(),
                                         sign * quaternion.z() };
    double const* parameters[] = { position.data(), signed_quaternion };
    fuse_core::Vector3d analytic_residual;
    fuse_core::Vector3d autodiff_residual;
    fuse_core::Matrix<double, 3, 3> analytic_position_jacobian;
    fuse_core::Matrix<double, 3, 3> autodiff_position_jacobian;
    fuse_core::Matrix<double, 3, 4> analytic_orientation_jacobian;
    fuse_core::Matrix<double, 3, 4> autodiff_orientation_jacobian;
    double* analytic_jacobians[] = { analytic_position_jacobian.data(), analytic_orientation_jacobian.data() };
    double* autodiff_jacobians[] = { autodiff_position_jacobian.data(), autodiff_orientation_jacobian.data() };

    ASSERT_TRUE(analytic->Evaluate(parameters, analytic_residual.data(), analytic_jacobians));
    ASSERT_TRUE(autodiff.Evaluate(parameters, autodiff_residual.data(), autodiff_jacobians));
    EXPECT_MATRIX_NEAR(expected_residual, analytic_residual, 1.0e-12);
    EXPECT_MATRIX_NEAR(expected_residual, autodiff_residual, 1.0e-12);
    // All cases avoid the pi-error cut and Euler pitch singularities.
    EXPECT_MATRIX_NEAR(analytic_position_jacobian, autodiff_position_jacobian, 1.0e-12);
    EXPECT_MATRIX_NEAR(analytic_orientation_jacobian, autodiff_orientation_jacobian, 1.0e-12);
  }
}

void expectOrientationYawResidual(double direction)
{
  Orientation3DStamped const orientation(rclcpp::Time(1, 0));
  fuse_core::Matrix3d const sqrt_information = makeSqrtInformation(true);
  fuse_core::Matrix3d const covariance = (sqrt_information.transpose() * sqrt_information).inverse();
  fuse_core::Vector3d const mean(0.2, -0.3, direction * (-M_PI + 0.01));
  AbsoluteOrientation3DStampedEulerConstraint const constraint(
      "test", orientation, mean, covariance,
      { Orientation3DStamped::Euler::ROLL, Orientation3DStamped::Euler::PITCH, Orientation3DStamped::Euler::YAW });
  std::unique_ptr<ceres::CostFunction> const cost_function(constraint.costFunction());
  Eigen::Quaterniond const quaternion = makeQuaternion({ 0.2, -0.3, direction * (M_PI - 0.01) });
  double const parameters[] = { quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z() };
  double const* parameter_blocks[] = { parameters };
  fuse_core::Vector3d residual;

  ASSERT_TRUE(cost_function->Evaluate(parameter_blocks, residual.data(), nullptr));
  EXPECT_MATRIX_NEAR(sqrt_information * fuse_core::Vector3d(0.0, 0.0, direction * -0.02), residual, 1.0e-12);
}
}  // namespace

TEST(NormalPriorEulerAngleWrap, PoseYawResidualAcrossPi)
{
  for (bool const correlated : { false, true })
  {
    for (double const direction : { 1.0, -1.0 })
    {
      SCOPED_TRACE(correlated);
      SCOPED_TRACE(direction);
      expectPoseAngularResiduals({ 0.2, -0.3, direction * (M_PI - 0.01) }, { 0.2, -0.3, direction * (-M_PI + 0.01) },
                                 { 0.0, 0.0, direction * -0.02 }, correlated);
    }
  }
}

TEST(NormalPriorEulerAngleWrap, PoseOrdinaryAngularResiduals)
{
  expectPoseAngularResiduals({ 0.3, -0.2, 0.7 }, { 0.2, -0.3, 0.4 }, { 0.1, 0.1, 0.3 }, true);
}

TEST(NormalPriorEulerAngleWrap, PoseWrapAllAnglesBeforeWeighting)
{
  // Pitch uses an equivalent nonprincipal prior because quaternion pitch is in [-pi/2, pi/2].
  expectPoseAngularResiduals({ M_PI - 0.01, 0.2, M_PI - 0.01 }, { -M_PI + 0.01, 0.3 + 2.0 * M_PI, -M_PI + 0.01 },
                             { -0.02, -0.1, -0.02 }, true);
}

TEST(NormalPriorEulerAngleWrap, OrientationYawResidualAcrossPi)
{
  expectOrientationYawResidual(1.0);
  expectOrientationYawResidual(-1.0);
}
