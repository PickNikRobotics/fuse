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
#include <ceres/covariance.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include <fuse_constraints/absolute_pose_3d_stamped_euler_constraint.hpp>
#include <fuse_core/eigen.hpp>
#include <fuse_core/eigen_gtest.hpp>
#include <fuse_core/serialization.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_variables/orientation_3d_stamped.hpp>
#include <fuse_variables/position_3d_stamped.hpp>

using fuse_constraints::AbsolutePose3DStampedEulerConstraint;
using fuse_variables::Orientation3DStamped;
using fuse_variables::Position3DStamped;

TEST(AbsolutePose3DStampedEulerConstraint, Constructor)
{
  // Construct a constraint just to make sure it compiles.
  Position3DStamped position_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  Orientation3DStamped orientation_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("walle"));

  fuse_core::Vector6d mean;
  mean << 1.0, 2.0, 3.0, 0.1, 0.1, 1.0;

  // Generated PD matrix using Octave: R = rand(6, 6); A = R * R' (use format long g to get the
  // required precision)
  fuse_core::Matrix6d cov;
  /* *INDENT-OFF* */
  cov << 2.0847236144069, 1.10752598122138, 1.02943174290333, 1.96120532313878, 1.96735470687891,
      1.5153042667951,  // NOLINT
      1.10752598122138, 1.39176289439125, 0.643422499737987, 1.35471905449013, 1.18353784377297,
      1.28979625492894,  // NOLINT
      1.02943174290333, 0.643422499737987, 1.26701658550187, 1.23641771365403, 1.55169301761377,
      1.34706781598061,  // NOLINT
      1.96120532313878, 1.35471905449013, 1.23641771365403, 2.39750866789926, 2.06887486311147,
      2.04350823837035,  // NOLINT
      1.96735470687891, 1.18353784377297, 1.55169301761377, 2.06887486311147, 2.503913946461,
      1.73844731158092,  // NOLINT
      1.5153042667951, 1.28979625492894, 1.34706781598061, 2.04350823837035, 1.73844731158092,
      2.15326088526198;  // NOLINT
  /* *INDENT-ON* */

  EXPECT_NO_THROW(
      AbsolutePose3DStampedEulerConstraint constraint("test", position_variable, orientation_variable, mean, cov));
}

TEST(AbsolutePose3DStampedEulerConstraint, ConstructorPartial)
{
  // Construct a constraint just to make sure it compiles.
  Position3DStamped position_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  Orientation3DStamped orientation_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("walle"));

  std::vector<size_t> variable_indices{ 0, 2, 3, 4, 5 };

  fuse_core::Vector6d mean_partial;
  mean_partial << 1.0, 0.0, 3.0, 0.1, 0.1, 1.0;

  fuse_core::Matrix5d cov_partial;
  /* *INDENT-OFF* */
  cov_partial << 2.0847236144069, 1.02943174290333, 1.96120532313878, 1.96735470687891, 1.5153042667951,  // NOLINT
      1.02943174290333, 1.26701658550187, 1.23641771365403, 1.55169301761377, 1.34706781598061,           // NOLINT
      1.96120532313878, 1.23641771365403, 2.39750866789926, 2.06887486311147, 2.04350823837035,           // NOLINT
      1.96735470687891, 1.55169301761377, 2.06887486311147, 2.503913946461, 1.73844731158092,             // NOLINT
      1.5153042667951, 1.34706781598061, 2.04350823837035, 1.73844731158092, 2.15326088526198;            // NOLINT
  /* *INDENT-ON* */

  EXPECT_NO_THROW(AbsolutePose3DStampedEulerConstraint constraint("test", position_variable, orientation_variable,
                                                                  mean_partial, cov_partial, variable_indices));
}

TEST(AbsolutePose3DStampedEulerConstraint, Covariance)
{
  // Verify the covariance <--> sqrt information conversions are correct
  Position3DStamped position_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("mo"));
  Orientation3DStamped orientation_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("mo"));

  fuse_core::Vector6d mean;
  mean << 1.0, 2.0, 3.0, 0.1, 0.1, 1.0;

  // Generated PD matrix using Octiave: R = rand(6, 6); A = R * R' (use format long g to get the
  // required precision)
  fuse_core::Matrix6d cov;
  /* *INDENT-OFF* */
  cov << 2.0847236144069, 1.10752598122138, 1.02943174290333, 1.96120532313878, 1.96735470687891,
      1.5153042667951,  // NOLINT
      1.10752598122138, 1.39176289439125, 0.643422499737987, 1.35471905449013, 1.18353784377297,
      1.28979625492894,  // NOLINT
      1.02943174290333, 0.643422499737987, 1.26701658550187, 1.23641771365403, 1.55169301761377,
      1.34706781598061,  // NOLINT
      1.96120532313878, 1.35471905449013, 1.23641771365403, 2.39750866789926, 2.06887486311147,
      2.04350823837035,  // NOLINT
      1.96735470687891, 1.18353784377297, 1.55169301761377, 2.06887486311147, 2.503913946461,
      1.73844731158092,  // NOLINT
      1.5153042667951, 1.28979625492894, 1.34706781598061, 2.04350823837035, 1.73844731158092,
      2.15326088526198;  // NOLINT
  /* *INDENT-ON* */

  AbsolutePose3DStampedEulerConstraint constraint("test", position_variable, orientation_variable, mean, cov);

  // Define the expected matrices (used Octave to compute sqrt_info: 'chol(inv(A))')
  fuse_core::Matrix6d expected_sqrt_info;
  /* *INDENT-OFF* */
  expected_sqrt_info << 2.12658752275893, 1.20265444927878, 4.71225672571804, 1.43587520991272, -4.12764062992821,
      -3.19509486240291,                                                                                // NOLINT
      0.0, 2.41958656956248, 5.93151964116945, 3.72535320852517, -4.23326858606213, -5.27776664777548,  // NOLINT
      0.0, 0.0, 3.82674686590005, 2.80341171946161, -2.68168478581452, -2.8894384435255,                // NOLINT
      0.0, 0.0, 0.0, 1.83006791372784, -0.696917410192509, -1.17412835464633,                           // NOLINT
      0.0, 0.0, 0.0, 0.0, 0.953302832761324, -0.769654414882847,                                        // NOLINT
      0.0, 0.0, 0.0, 0.0, 0.0, 0.681477739760948;                                                       // NOLINT
  /* *INDENT-ON* */
  fuse_core::Matrix6d expected_cov = cov;

  // Compare
  EXPECT_MATRIX_NEAR(expected_cov, constraint.covariance(), 1.0e-9);
  EXPECT_MATRIX_NEAR(expected_sqrt_info, constraint.sqrtInformation(), 1.0e-9);
}

TEST(AbsolutePose3DStampedEulerConstraint, CovariancePartial)
{
  // Verify the covariance <--> sqrt information conversions are correct
  Position3DStamped position_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("mo"));
  Orientation3DStamped orientation_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("mo"));

  std::vector<size_t> variable_indices{ 0, 2, 3, 4, 5 };

  fuse_core::Vector6d mean;
  mean << 1.0, 0.0, 3.0, 0.1, 0.1, 1.0;

  // Generated PD matrix using Octiave: R = rand(6, 6); A = R * R' (use format long g to get the
  // required precision)
  fuse_core::Matrix5d cov;
  /* *INDENT-OFF* */
  cov << 2.0847236144069, 1.02943174290333, 1.96120532313878, 1.96735470687891, 1.5153042667951,  // NOLINT
      1.02943174290333, 1.26701658550187, 1.23641771365403, 1.55169301761377, 1.34706781598061,   // NOLINT
      1.96120532313878, 1.23641771365403, 2.39750866789926, 2.06887486311147, 2.04350823837035,   // NOLINT
      1.96735470687891, 1.55169301761377, 2.06887486311147, 2.503913946461, 1.73844731158092,     // NOLINT
      1.5153042667951, 1.34706781598061, 2.04350823837035, 1.73844731158092, 2.15326088526198;    // NOLINT
  /* *INDENT-ON* */

  AbsolutePose3DStampedEulerConstraint constraint("test", position_variable, orientation_variable, mean, cov,
                                                  variable_indices);

  // Define the expected matrices (used Octave to compute sqrt_info: 'chol(inv(A))')
  fuse_core::Matrix<double, 5, 6> expected_sqrt_info;
  /* *INDENT-OFF* */
  expected_sqrt_info << 1.90431982, 0.0, 1.57962714, -0.37235015, -1.81200356, -0.51202133,  // NOLINT
      0., 0.0, 3.82674687, 2.80341172, -2.68168479, -2.88943844,                             // NOLINT
      0., 0.0, 0., 1.83006791, -0.69691741, -1.17412835,                                     // NOLINT
      0., 0.0, 0., 0., 0.95330283, -0.76965441,                                              // NOLINT
      0., 0.0, 0., 0., 0., 0.68147774;                                                       // NOLINT
  /* *INDENT-ON* */

  fuse_core::Matrix6d expected_cov;
  /* *INDENT-OFF* */
  expected_cov << 2.0847236144069, 0.0, 1.02943174290333, 1.96120532313878, 1.96735470687891,
      1.5153042667951,                                                                                // NOLINT
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                                                                   // NOLINT
      1.02943174290333, 0.0, 1.26701658550187, 1.23641771365403, 1.55169301761377, 1.34706781598061,  // NOLINT
      1.96120532313878, 0.0, 1.23641771365403, 2.39750866789926, 2.06887486311147, 2.04350823837035,  // NOLINT
      1.96735470687891, 0.0, 1.55169301761377, 2.06887486311147, 2.503913946461, 1.73844731158092,    // NOLINT
      1.5153042667951, 0.0, 1.34706781598061, 2.04350823837035, 1.73844731158092, 2.15326088526198;   // NOLINT
  /* *INDENT-ON* */

  // Compare
  EXPECT_MATRIX_NEAR(expected_cov, constraint.covariance(), 1.0e-8);
  EXPECT_MATRIX_NEAR(expected_sqrt_info, constraint.sqrtInformation(), 1.0e-8);
}

TEST(AbsolutePose3DStampedEulerConstraint, Optimization)
{
  // Optimize a single pose and single constraint, verify the expected value and covariance are
  // generated. Create the variables
  auto position_variable = Position3DStamped::make_shared(rclcpp::Time(1, 0), fuse_core::uuid::generate("spra"));
  position_variable->x() = 1.5;
  position_variable->y() = -3.0;
  position_variable->z() = 10.0;

  auto orientation_variable = Orientation3DStamped::make_shared(rclcpp::Time(1, 0), fuse_core::uuid::generate("spra"));
  orientation_variable->w() = 0.952;
  orientation_variable->x() = 0.038;
  orientation_variable->y() = -0.189;
  orientation_variable->z() = 0.239;

  // Create an absolute pose constraint
  fuse_core::Vector6d mean;
  mean << 1.0, 2.0, 3.0, 0.0, 0.0, 0.0;

  fuse_core::Matrix6d cov;
  /* *INDENT-OFF* */
  cov << 1.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.1, 2.0, 0.6, 0.5, 0.4, 0.3, 0.2, 0.6, 3.0, 0.2, 0.1, 0.2, 0.3, 0.5, 0.2, 4.0,
      0.3, 0.4, 0.4, 0.4, 0.1, 0.3, 5.0, 0.5, 0.5, 0.3, 0.2, 0.4, 0.5, 6.0;
  /* *INDENT-ON* */

  auto constraint =
      AbsolutePose3DStampedEulerConstraint::make_shared("test", *position_variable, *orientation_variable, mean, cov);

  // Build the problem
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
  ceres::Problem problem(problem_options);
  problem.AddParameterBlock(position_variable->data(), position_variable->size(),
#if !CERES_SUPPORTS_MANIFOLDS
                            position_variable->localParameterization());
#else
                            position_variable->manifold());
#endif

  problem.AddParameterBlock(orientation_variable->data(), orientation_variable->size(),
#if !CERES_SUPPORTS_MANIFOLDS
                            orientation_variable->localParameterization());
#else
                            orientation_variable->manifold());
#endif

  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(position_variable->data());
  parameter_blocks.push_back(orientation_variable->data());

  problem.AddResidualBlock(constraint->costFunction(), constraint->lossFunction(), parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(1.0, position_variable->x(), 1.0e-5);
  EXPECT_NEAR(2.0, position_variable->y(), 1.0e-5);
  EXPECT_NEAR(3.0, position_variable->z(), 1.0e-5);
  EXPECT_NEAR(1.0, orientation_variable->w(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->x(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->y(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->z(), 1.0e-3);

  // Compute the covariance
  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  covariance_blocks.emplace_back(position_variable->data(), position_variable->data());
  covariance_blocks.emplace_back(orientation_variable->data(), orientation_variable->data());
  covariance_blocks.emplace_back(position_variable->data(), orientation_variable->data());

  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);
  covariance.Compute(covariance_blocks, &problem);
  fuse_core::MatrixXd cov_pos_pos(position_variable->size(), position_variable->size());
  covariance.GetCovarianceBlock(position_variable->data(), position_variable->data(), cov_pos_pos.data());

  fuse_core::MatrixXd cov_or_or(orientation_variable->localSize(), orientation_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(orientation_variable->data(), orientation_variable->data(),
                                              cov_or_or.data());

  fuse_core::MatrixXd cov_pos_or(position_variable->localSize(), orientation_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(position_variable->data(), orientation_variable->data(),
                                              cov_pos_or.data());

  // Assemble the full covariance from the covariance blocks
  fuse_core::Matrix6d actual_covariance;
  actual_covariance << cov_pos_pos, cov_pos_or, cov_pos_or.transpose(), cov_or_or;

  // Define the expected covariance
  fuse_core::Matrix6d expected_covariance;
  /* *INDENT-OFF* */
  expected_covariance << 1.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.1, 2.0, 0.6, 0.5, 0.4, 0.3, 0.2, 0.6, 3.0, 0.2, 0.1, 0.2, 0.3,
      0.5, 0.2, 4.0, 0.3, 0.4, 0.4, 0.4, 0.1, 0.3, 5.0, 0.5, 0.5, 0.3, 0.2, 0.4, 0.5, 6.0;
  /* *INDENT-ON* */

  // High tolerance here, but also high values of covariance
  EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-2);
}

TEST(AbsolutePose3DStampedEulerConstraint, OptimizationPartial)
{
  // Optimize a single pose and single constraint, verify the expected value and covariance are
  // generated. Create the variables. Version for partial measurements
  auto position_variable = Position3DStamped::make_shared(rclcpp::Time(1, 0), fuse_core::uuid::generate("spra"));
  position_variable->x() = 1.5;
  position_variable->y() = 1.0;
  position_variable->z() = 10.0;

  auto orientation_variable = Orientation3DStamped::make_shared(rclcpp::Time(1, 0), fuse_core::uuid::generate("spra"));
  orientation_variable->w() = 0.952;
  orientation_variable->x() = 0.038;
  orientation_variable->y() = -0.189;
  orientation_variable->z() = 0.239;

  // Create an absolute pose constraint
  std::vector<size_t> variable_indices{ 0, 2, 3, 4, 5 };

  fuse_core::Vector6d mean;
  mean << 1.0, 0.0, 3.0, 0.0, 0.0, 0.0;

  fuse_core::Matrix5d cov;
  /* *INDENT-OFF* */
  cov << 1.0, 0.2, 0.3, 0.4, 0.5, 0.2, 3.0, 0.2, 0.1, 0.2, 0.3, 0.2, 4.0, 0.3, 0.4, 0.4, 0.1, 0.3, 5.0, 0.5, 0.5, 0.2,
      0.4, 0.5, 6.0;
  /* *INDENT-ON* */

  auto constraint = AbsolutePose3DStampedEulerConstraint::make_shared("test", *position_variable, *orientation_variable,
                                                                      mean, cov, variable_indices);

  // Build the problem
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = fuse_core::Loss::Ownership;
  ceres::Problem problem(problem_options);
  problem.AddParameterBlock(position_variable->data(), position_variable->size(),
#if !CERES_SUPPORTS_MANIFOLDS
                            position_variable->localParameterization());
#else
                            position_variable->manifold());
#endif

  problem.AddParameterBlock(orientation_variable->data(), orientation_variable->size(),
#if !CERES_SUPPORTS_MANIFOLDS
                            orientation_variable->localParameterization());
#else
                            orientation_variable->manifold());
#endif

  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(position_variable->data());
  parameter_blocks.push_back(orientation_variable->data());

  problem.AddResidualBlock(constraint->costFunction(), constraint->lossFunction(), parameter_blocks);

  // Run the solver
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Check
  EXPECT_NEAR(1.0, position_variable->x(), 1.0e-5);
  EXPECT_NEAR(1.0, position_variable->y(), 1.0e-5);  // This is not measured so it will not change
  EXPECT_NEAR(3.0, position_variable->z(), 1.0e-5);
  EXPECT_NEAR(1.0, orientation_variable->w(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->x(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->y(), 1.0e-3);
  EXPECT_NEAR(0.0, orientation_variable->z(), 1.0e-3);

  // Compute the covariance
  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  covariance_blocks.emplace_back(position_variable->data(), position_variable->data());
  covariance_blocks.emplace_back(orientation_variable->data(), orientation_variable->data());
  covariance_blocks.emplace_back(position_variable->data(), orientation_variable->data());

  ceres::Covariance::Options cov_options;
  cov_options.algorithm_type = ceres::DENSE_SVD;
  ceres::Covariance covariance(cov_options);
  covariance.Compute(covariance_blocks, &problem);
  fuse_core::MatrixXd cov_pos_pos(position_variable->size(), position_variable->size());
  covariance.GetCovarianceBlock(position_variable->data(), position_variable->data(), cov_pos_pos.data());

  fuse_core::MatrixXd cov_or_or(orientation_variable->localSize(), orientation_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(orientation_variable->data(), orientation_variable->data(),
                                              cov_or_or.data());

  fuse_core::MatrixXd cov_pos_or(position_variable->localSize(), orientation_variable->localSize());
  covariance.GetCovarianceBlockInTangentSpace(position_variable->data(), orientation_variable->data(),
                                              cov_pos_or.data());

  // Assemble the full covariance from the covariance blocks
  fuse_core::Matrix6d actual_covariance;
  actual_covariance << cov_pos_pos, cov_pos_or, cov_pos_or.transpose(), cov_or_or;

  // Define the expected covariance
  fuse_core::Matrix6d expected_covariance;
  /* *INDENT-OFF* */
  expected_covariance << 1.0, 0.0, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 3.0, 0.2, 0.1, 0.2, 0.3,
      0.0, 0.2, 4.0, 0.3, 0.4, 0.4, 0.0, 0.1, 0.3, 5.0, 0.5, 0.5, 0.0, 0.2, 0.4, 0.5, 6.0;
  /* *INDENT-ON* */

  // High tolerance here, but also high values of covariance
  EXPECT_MATRIX_NEAR(expected_covariance, actual_covariance, 1.0e-2);
}

TEST(AbsolutePose3DStampedEulerConstraint, Serialization)
{
  // Construct a constraint
  Position3DStamped position_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("walle"));
  Orientation3DStamped orientation_variable(rclcpp::Time(1234, 5678), fuse_core::uuid::generate("walle"));

  fuse_core::Vector6d mean;
  mean << 1.0, 2.0, 3.0, 0.1, 0.1, 1.0;

  // Generated PD matrix using Octave: R = rand(6, 6); A = R * R' (use format long g to get the
  // required precision)
  fuse_core::Matrix6d cov;
  /* *INDENT-OFF* */
  cov << 2.0847236144069, 1.10752598122138, 1.02943174290333, 1.96120532313878, 1.96735470687891,
      1.5153042667951,  // NOLINT
      1.10752598122138, 1.39176289439125, 0.643422499737987, 1.35471905449013, 1.18353784377297,
      1.28979625492894,  // NOLINT
      1.02943174290333, 0.643422499737987, 1.26701658550187, 1.23641771365403, 1.55169301761377,
      1.34706781598061,  // NOLINT
      1.96120532313878, 1.35471905449013, 1.23641771365403, 2.39750866789926, 2.06887486311147,
      2.04350823837035,  // NOLINT
      1.96735470687891, 1.18353784377297, 1.55169301761377, 2.06887486311147, 2.503913946461,
      1.73844731158092,  // NOLINT
      1.5153042667951, 1.28979625492894, 1.34706781598061, 2.04350823837035, 1.73844731158092,
      2.15326088526198;  // NOLINT
  /* *INDENT-ON* */

  AbsolutePose3DStampedEulerConstraint expected("test", position_variable, orientation_variable, mean, cov);

  // Serialize the constraint into an archive
  std::stringstream stream;
  {
    fuse_core::TextOutputArchive archive(stream);
    expected.serialize(archive);
  }

  // Deserialize a new constraint from that same stream
  AbsolutePose3DStampedEulerConstraint actual;
  {
    fuse_core::TextInputArchive archive(stream);
    actual.deserialize(archive);
  }

  // Compare
  EXPECT_EQ(expected.uuid(), actual.uuid());
  EXPECT_EQ(expected.variables(), actual.variables());
  EXPECT_MATRIX_EQ(expected.mean(), actual.mean());
  EXPECT_MATRIX_EQ(expected.sqrtInformation(), actual.sqrtInformation());
}
