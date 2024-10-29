/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Clearpath Robotics
 *  Copyright (c) 2024, Giacomo Franchini
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
#include <ceres/cost_function.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <ceres/autodiff_cost_function.h>
#include <fuse_core/util.hpp>
#include <fuse_core/eigen.hpp>

struct Quat2RPY
{
  template <typename T>
  bool operator()(const T* const q, T* rpy) const
  {
    rpy[0] = fuse_core::getRoll(q[0], q[1], q[2], q[3]);
    rpy[1] = fuse_core::getPitch(q[0], q[1], q[2], q[3]);
    rpy[2] = fuse_core::getYaw(q[0], q[1], q[2], q[3]);
    return true;
  }

  static std::unique_ptr<ceres::CostFunction> create()
  {
    return std::unique_ptr<ceres::CostFunction>(new ceres::AutoDiffCostFunction<Quat2RPY, 3, 4>(new Quat2RPY()));
  }
};

struct QuatProd
{
  template <typename T>
  bool operator()(const T* const q1, const T* const q2, T* q_out) const
  {
    ceres::QuaternionProduct(q1, q2, q_out);
    return true;
  }

  static std::unique_ptr<ceres::CostFunction> create()
  {
    return std::unique_ptr<ceres::CostFunction>(new ceres::AutoDiffCostFunction<QuatProd, 4, 4, 4>(new QuatProd()));
  }
};

struct Quat2AngleAxis
{
  template <typename T>
  bool operator()(const T* const q, T* aa) const
  {
    ceres::QuaternionToAngleAxis(q, aa);
    return true;
  }

  static std::unique_ptr<ceres::CostFunction> create()
  {
    return std::unique_ptr<ceres::CostFunction>(
        new ceres::AutoDiffCostFunction<Quat2AngleAxis, 3, 4>(new Quat2AngleAxis()));
  }
};

TEST(Util, wrapAngle2D)
{
  // Wrap angle already in [-Pi, +Pi) range
  {
    const double angle = 0.5;
    EXPECT_EQ(angle, fuse_core::wrapAngle2D(angle));
  }

  // Wrap angle equal to +Pi
  {
    const double angle = M_PI;
    EXPECT_EQ(-angle, fuse_core::wrapAngle2D(angle));
  }

  // Wrap angle equal to -Pi
  {
    const double angle = -M_PI;
    EXPECT_EQ(angle, fuse_core::wrapAngle2D(angle));
  }

  // Wrap angle greater than +Pi
  {
    const double angle = 0.5;
    EXPECT_EQ(angle, fuse_core::wrapAngle2D(angle + 3.0 * 2.0 * M_PI));
  }

  // Wrap angle smaller than -Pi
  {
    const double angle = 0.5;
    EXPECT_EQ(angle, fuse_core::wrapAngle2D(angle - 3.0 * 2.0 * M_PI));
  }

  // Join topic names
  {
    EXPECT_EQ("a/b", fuse_core::joinTopicName("a", "b"));
    EXPECT_EQ("/a/b", fuse_core::joinTopicName("/a", "b"));
    EXPECT_EQ("a/b", fuse_core::joinTopicName("a/", "b"));
    EXPECT_EQ("/b", fuse_core::joinTopicName("a", "/b"));
    EXPECT_EQ("/b", fuse_core::joinTopicName("a/", "/b"));
    EXPECT_EQ("~/b", fuse_core::joinTopicName("a/", "~/b"));
    EXPECT_EQ("~b", fuse_core::joinTopicName("a/", "~b"));
  }
}

TEST(Util, quaternion2rpy)
{
  // Test correct conversion from quaternion to roll-pitch-yaw
  std::array<double, 4> q = { 1.0, 0.0, 0.0, 0.0 };
  std::array<double, 3> rpy{};
  fuse_core::quaternion2rpy(q.data(), rpy.data());
  EXPECT_EQ(0.0, rpy[0]);
  EXPECT_EQ(0.0, rpy[1]);
  EXPECT_EQ(0.0, rpy[2]);

  q[0] = 0.9818562;
  q[1] = 0.0640713;
  q[2] = 0.0911575;
  q[3] = -0.1534393;

  fuse_core::quaternion2rpy(q.data(), rpy.data());
  EXPECT_NEAR(0.1, rpy[0], 1e-6);
  EXPECT_NEAR(0.2, rpy[1], 1e-6);
  EXPECT_NEAR(-0.3, rpy[2], 1e-6);

  // Test correct quaternion to roll-pitch-yaw function jacobian
  const Eigen::Quaterniond q_eigen = Eigen::Quaterniond::UnitRandom();
  std::array<double, 12> j_analytic{};
  std::array<double, 12> j_autodiff{};
  q[0] = q_eigen.w();
  q[1] = q_eigen.x();
  q[2] = q_eigen.y();
  q[3] = q_eigen.z();

  fuse_core::quaternion2rpy(q.data(), rpy.data(), j_analytic.data());

  std::array<double*, 1> jacobians = { j_autodiff.data() };
  std::array<double*, 1> const parameters = { q.data() };

  auto quat2rpy_cf = Quat2RPY::create();
  std::array<double, 3> rpy_autodiff{};
  quat2rpy_cf->Evaluate(parameters.data(), rpy_autodiff.data(), jacobians.data());

  Eigen::Map<fuse_core::Matrix<double, 3, 4>> const j_autodiff_map(jacobians[0]);
  Eigen::Map<fuse_core::Matrix<double, 3, 4>> const j_analytic_map(j_analytic.data());

  EXPECT_TRUE(j_analytic_map.isApprox(j_autodiff_map));
}

TEST(Util, quaternionProduct)
{
  // Test correct quaternion product function jacobian
  const Eigen::Quaterniond q1_eigen = Eigen::Quaterniond::UnitRandom();
  const Eigen::Quaterniond q2_eigen = Eigen::Quaterniond::UnitRandom();
  std::array<double, 4> q_out{};
  std::array<double, 4> q1{ q1_eigen.w(), q1_eigen.x(), q1_eigen.y(), q1_eigen.z() };

  std::array<double, 4> q2{ q2_eigen.w(), q2_eigen.x(), q2_eigen.y(), q2_eigen.z() };

  // Atm only the jacobian wrt the second quaternion is implemented. If the computation will be
  // extended in future, we just have to compare J_analytic_q1 with the other automatic J_autodiff_q1.
  // std::array<double, 16> j_analytic_q1{};
  std::array<double, 16> j_analytic_q2{};  // Analytical Jacobians wrt first and second quaternion
  std::array<double, 16> j_autodiff_q1{};
  std::array<double, 16> j_autodiff_q2{};  // Autodiff Jacobians wrt first and second quaternion

  fuse_core::quaternionProduct(q1.data(), q2.data(), q_out.data(), j_analytic_q2.data());

  fuse_core::quaternionProduct(q1.data(), q2.data(), q_out.data(), j_analytic_q2.data());

  std::array<double*, 2> jacobians{};
  jacobians[0] = j_autodiff_q1.data();
  jacobians[1] = j_autodiff_q2.data();

  std::array<double const*, 2> parameters{};
  parameters[0] = q1.data();
  parameters[1] = q2.data();

  auto quat_prod_cf = QuatProd::create();
  std::array<double, 4> q_out_autodiff{};
  quat_prod_cf->Evaluate(parameters.data(), q_out_autodiff.data(), jacobians.data());

  Eigen::Map<fuse_core::Matrix<double, 4, 4>> const j_autodiff_q1_map(jacobians[0]);
  Eigen::Map<fuse_core::Matrix<double, 4, 4>> const j_autodiff_q2_map(jacobians[1]);

  // Eigen::Map<fuse_core::Matrix<double, 4, 4>> J_analytic_q1_map(J_analytic_q1);
  Eigen::Map<fuse_core::Matrix<double, 4, 4>> const j_analytic_q2_map(j_analytic_q2.data());

  EXPECT_TRUE(j_analytic_q2_map.isApprox(j_autodiff_q2_map));
}

TEST(Util, quaternionToAngleAxis)
{
  // Test correct quaternion to angle-axis function jacobian, for quaternions representing non-zero rotation
  // The implementation of quaternionToAngleAxis changes slightly between ceres 2.1.0 and 2.2.0. We checked for
  // both the versions for the test to pass.
  {
    const Eigen::Quaterniond q_eigen = Eigen::Quaterniond::UnitRandom();
    std::array<double, 3> angle_axis{};
    std::array<double, 4> q{ q_eigen.w(), q_eigen.x(), q_eigen.y(), q_eigen.z() };

    std::array<double, 12> j_analytic{};
    std::array<double, 12> j_autodiff{};

    fuse_core::quaternionToAngleAxis(q.data(), angle_axis.data(), j_analytic.data());

    std::array<double*, 1> jacobians = { j_autodiff.data() };
    std::array<double const*, 1> parameters = { q.data() };

    auto quat2angle_axis_cf = Quat2AngleAxis::create();
    std::array<double, 3> angle_axis_autodiff{};
    quat2angle_axis_cf->Evaluate(parameters.data(), angle_axis_autodiff.data(), jacobians.data());

    Eigen::Map<fuse_core::Matrix<double, 3, 4>> const j_autodiff_map(jacobians[0]);
    Eigen::Map<fuse_core::Matrix<double, 3, 4>> const j_analytic_map(j_analytic.data());

    EXPECT_TRUE(j_analytic_map.isApprox(j_autodiff_map));
  }

  // Test correct quaternion to angle-axis function jacobian, for quaternions representing zero rotation
  {
    std::array<double, 3> angle_axis{};
    std::array<double, 4> q{ 1.0, 0.0, 0.0, 0.0 };

    std::array<double, 12> j_analytic{};
    std::array<double, 12> j_autodiff{};

    fuse_core::quaternionToAngleAxis(q.data(), angle_axis.data(), j_analytic.data());

    std::array<double*, 1> jacobians = { j_autodiff.data() };
    std::array<double const*, 1> parameters = { q.data() };

    auto quat2angle_axis_cf = Quat2AngleAxis::create();
    std::array<double, 3> angle_axis_autodiff{};
    quat2angle_axis_cf->Evaluate(parameters.data(), angle_axis_autodiff.data(), jacobians.data());

    Eigen::Map<fuse_core::Matrix<double, 3, 4>> const j_autodiff_map(jacobians[0]);
    Eigen::Map<fuse_core::Matrix<double, 3, 4>> const j_analytic_map(j_analytic.data());

    EXPECT_TRUE(j_analytic_map.isApprox(j_autodiff_map));
  }

  {
    // Test that approximate conversion and jacobian computation work for very small angles that
    // could potentially cause underflow.

    double const theta = std::pow(std::numeric_limits<double>::min(), 0.75);
    std::array<double, 4> q = { cos(theta / 2.0), sin(theta / 2.0), 0, 0 };
    std::array<double, 3> angle_axis{};
    std::array<double, 3> expected = { theta, 0, 0 };
    std::array<double, 12> j_analytic{};
    std::array<double, 12> j_autodiff{};

    fuse_core::quaternionToAngleAxis(q.data(), angle_axis.data(), j_analytic.data());
    EXPECT_DOUBLE_EQ(angle_axis[0], expected[0]);
    EXPECT_DOUBLE_EQ(angle_axis[1], expected[1]);
    EXPECT_DOUBLE_EQ(angle_axis[2], expected[2]);

    std::array<double*, 1> jacobians = { j_autodiff.data() };
    std::array<double const*, 1> parameters = { q.data() };

    auto quat2angle_axis_cf = Quat2AngleAxis::create();
    std::array<double, 3> angle_axis_autodiff{};
    quat2angle_axis_cf->Evaluate(parameters.data(), angle_axis_autodiff.data(), jacobians.data());

    Eigen::Map<fuse_core::Matrix<double, 3, 4>> const j_autodiff_map(jacobians[0]);
    Eigen::Map<fuse_core::Matrix<double, 3, 4>> const j_analytic_map(j_analytic.data());

    EXPECT_TRUE(j_analytic_map.isApprox(j_autodiff_map));
  }
}
