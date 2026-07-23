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
#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <vector>

#include <fuse_core/eigen_gtest.hpp>
#include <fuse_core/util.hpp>
#include <fuse_models/omnidirectional_3d_predict.hpp>
#include <ceres/jet.h>

TEST(Predict, predictDirectVals)
{
  fuse_core::Vector3d position1(0.0, 0.0, 0.0);
  Eigen::Quaterniond orientation1(1.0, 0.0, 0.0, 0.0);
  fuse_core::Vector3d vel_linear1(1.0, 0.0, 0.0);
  fuse_core::Vector3d vel_angular1(0.0, 0.0, 1.570796327);
  fuse_core::Vector3d acc_linear1(1.0, 0.0, 0.0);
  double dt = 0.1;
  fuse_core::Vector3d position2;
  Eigen::Quaterniond orientation2;
  fuse_core::Vector3d vel_linear2;
  fuse_core::Vector3d vel_angular2;
  fuse_core::Vector3d acc_linear2;

  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(0.1570796327, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

  EXPECT_DOUBLE_EQ(0.105, position2.x());
  EXPECT_DOUBLE_EQ(0.0, position2.y());
  EXPECT_DOUBLE_EQ(0.0, position2.z());
  EXPECT_DOUBLE_EQ(q.w(), orientation2.w());
  EXPECT_DOUBLE_EQ(q.x(), orientation2.x());
  EXPECT_DOUBLE_EQ(q.y(), orientation2.y());
  EXPECT_DOUBLE_EQ(q.z(), orientation2.z());
  EXPECT_DOUBLE_EQ(1.1, vel_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, vel_linear2.y());
  EXPECT_DOUBLE_EQ(0.0, vel_linear2.z());
  EXPECT_DOUBLE_EQ(0.0, vel_angular2.x());
  EXPECT_DOUBLE_EQ(0.0, vel_angular2.y());
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2.z());
  EXPECT_DOUBLE_EQ(1.0, acc_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.y());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.z());

  // // Carry on with the output state from last time - show in-place update support
  fuse_models::predict(position2, orientation2, vel_linear2, vel_angular2, acc_linear2, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  q = Eigen::AngleAxisd(0.3141592654, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

  EXPECT_DOUBLE_EQ(0.21858415916807189, position2.x());
  EXPECT_DOUBLE_EQ(0.017989963481956205, position2.y());
  EXPECT_DOUBLE_EQ(0.0, position2.z());
  EXPECT_DOUBLE_EQ(q.w(), orientation2.w());
  EXPECT_DOUBLE_EQ(q.x(), orientation2.x());
  EXPECT_DOUBLE_EQ(q.y(), orientation2.y());
  EXPECT_DOUBLE_EQ(q.z(), orientation2.z());
  EXPECT_DOUBLE_EQ(1.2, vel_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, vel_linear2.y());
  EXPECT_DOUBLE_EQ(0.0, vel_linear2.z());
  EXPECT_DOUBLE_EQ(0.0, vel_angular2.x());
  EXPECT_DOUBLE_EQ(0.0, vel_angular2.y());
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2.z());
  EXPECT_DOUBLE_EQ(1.0, acc_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.y());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.z());

  // Use non-zero Y values
  vel_linear1.y() = -1.0;
  vel_angular1.z() = -1.570796327;
  acc_linear1.y() = -1.0;

  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  q = Eigen::AngleAxisd(-0.1570796327, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

  EXPECT_DOUBLE_EQ(0.105, position2.x());
  EXPECT_DOUBLE_EQ(-0.105, position2.y());
  EXPECT_DOUBLE_EQ(0.0, position2.z());
  EXPECT_TRUE(q.isApprox(orientation2));
  EXPECT_DOUBLE_EQ(1.1, vel_linear2.x());
  EXPECT_DOUBLE_EQ(-1.1, vel_linear2.y());
  EXPECT_DOUBLE_EQ(0.0, vel_linear2.z());
  EXPECT_DOUBLE_EQ(0.0, vel_angular2.x());
  EXPECT_DOUBLE_EQ(0.0, vel_angular2.y());
  EXPECT_DOUBLE_EQ(-1.570796327, vel_angular2.z());
  EXPECT_DOUBLE_EQ(1.0, acc_linear2.x());
  EXPECT_DOUBLE_EQ(-1.0, acc_linear2.y());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.z());

  // Out of plane motion
  position1 = { 0.0, 0.0, 0.0 };
  orientation1 = { 1.0, 0.0, 0.0, 0.0 };
  vel_linear1 = { 0.0, 0.0, 0.1 };
  vel_angular1 = { 1.570796327, 0.0, 0.0 };
  acc_linear1 = { 0.0, 0.0, 1.0 };
  dt = 0.1;

  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  EXPECT_DOUBLE_EQ(0.0, position2.x());
  EXPECT_DOUBLE_EQ(0.0, position2.y());
  EXPECT_DOUBLE_EQ(0.015, position2.z());
  EXPECT_DOUBLE_EQ(0.99691733373232339, orientation2.w());
  EXPECT_DOUBLE_EQ(0.078459095738068516, orientation2.x());
  EXPECT_DOUBLE_EQ(0.0, orientation2.y());
  EXPECT_DOUBLE_EQ(0.0, orientation2.z());
  EXPECT_DOUBLE_EQ(0.0, vel_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, vel_linear2.y());
  EXPECT_DOUBLE_EQ(0.2, vel_linear2.z());
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2.x());
  EXPECT_DOUBLE_EQ(0.0, vel_angular2.y());
  EXPECT_DOUBLE_EQ(0.0, vel_angular2.z());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.x());
  EXPECT_DOUBLE_EQ(0.0, acc_linear2.y());
  EXPECT_DOUBLE_EQ(1.0, acc_linear2.z());

  // General 3D motion (these value are checked against rl predict() equations)
  position1 = { 0.0, 0.0, 0.0 };
  orientation1 = { 0.110, -0.003, -0.943, 0.314 };  // RPY {-2.490, -0.206, 3.066}
  vel_linear1 = { 0.1, 0.2, 0.1 };
  vel_angular1 = { 1.570796327, 1.570796327, -1.570796327 };
  acc_linear1 = { -0.5, 1.0, 1.0 };
  dt = 0.1;

  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  EXPECT_DOUBLE_EQ(-0.012044123300410431, position2.x());
  EXPECT_DOUBLE_EQ(0.011755776496514461, position2.y());
  EXPECT_DOUBLE_EQ(-0.024959783911094033, position2.z());
  EXPECT_DOUBLE_EQ(0.20388993714859482, orientation2.w());
  EXPECT_DOUBLE_EQ(0.061993007799788086, orientation2.x());
  EXPECT_DOUBLE_EQ(-0.90147820778463239, orientation2.y());
  EXPECT_DOUBLE_EQ(0.3767264277999153, orientation2.z());
  EXPECT_DOUBLE_EQ(0.05, vel_linear2.x());
  EXPECT_DOUBLE_EQ(0.3, vel_linear2.y());
  EXPECT_DOUBLE_EQ(0.2, vel_linear2.z());
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2.x());
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2.y());
  EXPECT_DOUBLE_EQ(-1.570796327, vel_angular2.z());
  EXPECT_DOUBLE_EQ(-0.5, acc_linear2.x());
  EXPECT_DOUBLE_EQ(1.0, acc_linear2.y());
  EXPECT_DOUBLE_EQ(1.0, acc_linear2.z());
}

TEST(Predict, predictFromDoublePointers)
{
  double position1[3]{ 0.0, 0.0, 0.0 };
  double orientation1[3]{ 0.0, 0.0, 0.0 };
  double vel_linear1[3]{ 1.0, 0.0, 0.0 };
  double vel_angular1[3]{ 0.0, 0.0, 1.570796327 };
  double acc_linear1[3]{ 1.0, 0.0, 0.0 };
  double dt = 0.1;
  double position2[3];
  double orientation2[3];
  double vel_linear2[3];
  double vel_angular2[3];
  double acc_linear2[3];

  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  EXPECT_DOUBLE_EQ(0.105, position2[0]);
  EXPECT_DOUBLE_EQ(0.0, position2[1]);
  EXPECT_DOUBLE_EQ(0.0, position2[2]);
  EXPECT_DOUBLE_EQ(0.0, orientation2[0]);
  EXPECT_DOUBLE_EQ(0.0, orientation2[1]);
  EXPECT_DOUBLE_EQ(0.1570796327, orientation2[2]);
  EXPECT_DOUBLE_EQ(1.1, vel_linear2[0]);
  EXPECT_DOUBLE_EQ(0.0, vel_linear2[1]);
  EXPECT_DOUBLE_EQ(0.0, vel_linear2[2]);
  EXPECT_DOUBLE_EQ(0.0, vel_angular2[0]);
  EXPECT_DOUBLE_EQ(0.0, vel_angular2[1]);
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2[2]);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2[0]);
  EXPECT_DOUBLE_EQ(0.0, acc_linear2[1]);
  EXPECT_DOUBLE_EQ(0.0, acc_linear2[2]);

  // Carry on with the output state from last time - show in-place update support
  fuse_models::predict(position2, orientation2, vel_linear2, vel_angular2, acc_linear2, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  EXPECT_DOUBLE_EQ(0.21858415916807189, position2[0]);
  EXPECT_DOUBLE_EQ(0.017989963481956205, position2[1]);
  EXPECT_DOUBLE_EQ(0.0, position2[2]);
  EXPECT_DOUBLE_EQ(0.0, orientation2[0]);
  EXPECT_DOUBLE_EQ(0.0, orientation2[1]);
  EXPECT_DOUBLE_EQ(0.3141592654, orientation2[2]);
  EXPECT_DOUBLE_EQ(1.2, vel_linear2[0]);
  EXPECT_DOUBLE_EQ(0.0, vel_linear2[1]);
  EXPECT_DOUBLE_EQ(0.0, vel_linear2[2]);
  EXPECT_DOUBLE_EQ(0.0, vel_angular2[0]);
  EXPECT_DOUBLE_EQ(0.0, vel_angular2[1]);
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2[2]);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2[0]);
  EXPECT_DOUBLE_EQ(0.0, acc_linear2[1]);
  EXPECT_DOUBLE_EQ(0.0, acc_linear2[2]);

  // Use non-zero Y values
  vel_linear1[1] = -1.0;
  vel_angular1[2] = -1.570796327;
  acc_linear1[1] = -1.0;

  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  EXPECT_DOUBLE_EQ(0.105, position2[0]);
  EXPECT_DOUBLE_EQ(-0.105, position2[1]);
  EXPECT_DOUBLE_EQ(0.0, position2[2]);
  EXPECT_DOUBLE_EQ(0.0, orientation2[0]);
  EXPECT_DOUBLE_EQ(0.0, orientation2[1]);
  EXPECT_DOUBLE_EQ(-0.1570796327, orientation2[2]);
  EXPECT_DOUBLE_EQ(1.1, vel_linear2[0]);
  EXPECT_DOUBLE_EQ(-1.1, vel_linear2[1]);
  EXPECT_DOUBLE_EQ(0.0, vel_linear2[2]);
  EXPECT_DOUBLE_EQ(0.0, vel_angular2[0]);
  EXPECT_DOUBLE_EQ(0.0, vel_angular2[1]);
  EXPECT_DOUBLE_EQ(-1.570796327, vel_angular2[2]);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2[0]);
  EXPECT_DOUBLE_EQ(-1.0, acc_linear2[1]);
  EXPECT_DOUBLE_EQ(0.0, acc_linear2[2]);

  // Out of plane motion
  position1[0] = 0.0;
  position1[1] = 0.0;
  position1[2] = 0.0;
  orientation1[0] = 0.0;
  orientation1[1] = 0.0;
  orientation1[2] = 0.0;
  vel_linear1[0] = 0.0;
  vel_linear1[1] = 0.0;
  vel_linear1[2] = 0.1;
  vel_angular1[0] = 1.570796327;
  vel_angular1[1] = 0.0;
  vel_angular1[2] = 0.0;
  acc_linear1[0] = 0.0;
  acc_linear1[1] = 0.0;
  acc_linear1[2] = 1.0;
  dt = 0.1;

  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  EXPECT_DOUBLE_EQ(0.0, position2[0]);
  EXPECT_DOUBLE_EQ(0.0, position2[1]);
  EXPECT_DOUBLE_EQ(0.015, position2[2]);
  EXPECT_DOUBLE_EQ(0.15707963270000003, orientation2[0]);
  EXPECT_DOUBLE_EQ(0.0, orientation2[1]);
  EXPECT_DOUBLE_EQ(0.0, orientation2[2]);
  EXPECT_DOUBLE_EQ(0.0, vel_linear2[0]);
  EXPECT_DOUBLE_EQ(0.0, vel_linear2[1]);
  EXPECT_DOUBLE_EQ(0.2, vel_linear2[2]);
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2[0]);
  EXPECT_DOUBLE_EQ(0.0, vel_angular2[1]);
  EXPECT_DOUBLE_EQ(0.0, vel_angular2[2]);
  EXPECT_DOUBLE_EQ(0.0, acc_linear2[0]);
  EXPECT_DOUBLE_EQ(0.0, acc_linear2[1]);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2[2]);

  // General 3D motion (these value are checked against rl predict() equations)
  position1[0] = 0.0;
  position1[1] = 0.0;
  position1[2] = 0.0;
  orientation1[0] = -2.490;
  orientation1[1] = -0.206;
  orientation1[2] = 3.066;
  vel_linear1[0] = 0.1;
  vel_linear1[1] = 0.2;
  vel_linear1[2] = 0.1;
  vel_angular1[0] = 1.570796327;
  vel_angular1[1] = 1.570796327;
  vel_angular1[2] = -1.570796327;
  acc_linear1[0] = -0.5;
  acc_linear1[1] = 1.0;
  acc_linear1[2] = 1.0;
  dt = 0.1;

  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  EXPECT_DOUBLE_EQ(-0.012031207341885572, position2[0]);
  EXPECT_DOUBLE_EQ(0.011723254405731805, position2[1]);
  EXPECT_DOUBLE_EQ(-0.024981300126995967, position2[2]);
  EXPECT_DOUBLE_EQ(-2.3391131265098766, orientation2[0]);
  EXPECT_DOUBLE_EQ(-0.4261584872792554, orientation2[1]);
  EXPECT_DOUBLE_EQ(3.0962756133525855, orientation2[2]);
  EXPECT_DOUBLE_EQ(0.05, vel_linear2[0]);
  EXPECT_DOUBLE_EQ(0.3, vel_linear2[1]);
  EXPECT_DOUBLE_EQ(0.2, vel_linear2[2]);
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2[0]);
  EXPECT_DOUBLE_EQ(1.570796327, vel_angular2[1]);
  EXPECT_DOUBLE_EQ(-1.570796327, vel_angular2[2]);
  EXPECT_DOUBLE_EQ(-0.5, acc_linear2[0]);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2[1]);
  EXPECT_DOUBLE_EQ(1.0, acc_linear2[2]);
}

TEST(Predict, predictFromJetPointers)
{
  using Jet = ceres::Jet<double, 32>;

  Jet position1[3] = { Jet(0.0), Jet(0.0), Jet(0.0) };
  Jet orientation1[3] = { Jet(0.0), Jet(0.0), Jet(0.0) };
  Jet vel_linear1[3] = { Jet(1.0), Jet(0.0), Jet(0.0) };
  Jet vel_angular1[3] = { Jet(0.0), Jet(0.0), Jet(1.570796327) };
  Jet acc_linear1[3] = { Jet(1.0), Jet(0.0), Jet(0.0) };
  Jet dt = Jet(0.1);
  Jet position2[3];
  Jet orientation2[3];
  Jet vel_linear2[3];
  Jet vel_angular2[3];
  Jet acc_linear2[3];

  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  EXPECT_DOUBLE_EQ(Jet(0.105).a, position2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, position2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, position2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, orientation2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, orientation2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.1570796327).a, orientation2[2].a);
  EXPECT_DOUBLE_EQ(Jet(1.1).a, vel_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_linear2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_angular2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_angular2[1].a);
  EXPECT_DOUBLE_EQ(Jet(1.570796327).a, vel_angular2[2].a);
  EXPECT_DOUBLE_EQ(Jet(1.0).a, acc_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[2].a);

  // Carry on with the output state from last time - show in-place update support
  fuse_models::predict(position2, orientation2, vel_linear2, vel_angular2, acc_linear2, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  EXPECT_DOUBLE_EQ(Jet(0.21858415916807189).a, position2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.017989963481956205).a, position2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, position2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, orientation2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, orientation2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.3141592654).a, orientation2[2].a);
  EXPECT_DOUBLE_EQ(Jet(1.2).a, vel_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_linear2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_angular2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_angular2[1].a);
  EXPECT_DOUBLE_EQ(Jet(1.570796327).a, vel_angular2[2].a);
  EXPECT_DOUBLE_EQ(Jet(1.0).a, acc_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[2].a);

  // // Use non-zero Y values
  vel_linear1[1] = Jet(-1.0);
  vel_angular1[2] = Jet(-1.570796327);
  acc_linear1[1] = Jet(-1.0);

  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  EXPECT_DOUBLE_EQ(Jet(0.105).a, position2[0].a);
  EXPECT_DOUBLE_EQ(Jet(-0.105).a, position2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, position2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, orientation2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, orientation2[1].a);
  EXPECT_DOUBLE_EQ(Jet(-0.1570796327).a, orientation2[2].a);
  EXPECT_DOUBLE_EQ(Jet(1.1).a, vel_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(-1.1).a, vel_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_linear2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_angular2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_angular2[1].a);
  EXPECT_DOUBLE_EQ(Jet(-1.570796327).a, vel_angular2[2].a);
  EXPECT_DOUBLE_EQ(Jet(1.0).a, acc_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(-1.0).a, acc_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[2].a);

  // Out of plane motion
  position1[0] = Jet(0.0);
  position1[1] = Jet(0.0);
  position1[2] = Jet(0.0);
  orientation1[0] = Jet(0.0);
  orientation1[1] = Jet(0.0);
  orientation1[2] = Jet(0.0);
  vel_linear1[0] = Jet(0.0);
  vel_linear1[1] = Jet(0.0);
  vel_linear1[2] = Jet(0.1);
  vel_angular1[0] = Jet(1.570796327);
  vel_angular1[1] = Jet(0.0);
  vel_angular1[2] = Jet(0.0);
  acc_linear1[0] = Jet(0.0);
  acc_linear1[1] = Jet(0.0);
  acc_linear1[2] = Jet(1.0);
  dt = Jet(0.1);

  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  EXPECT_DOUBLE_EQ(Jet(0.0).a, position2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, position2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.015).a, position2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.15707963270000003).a, orientation2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, orientation2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, orientation2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.2).a, vel_linear2[2].a);
  EXPECT_DOUBLE_EQ(Jet(1.570796327).a, vel_angular2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_angular2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, vel_angular2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.0).a, acc_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(1.0).a, acc_linear2[2].a);

  // General 3D motion (these value are checked against rl predict() equations)
  position1[0] = Jet(0.0);
  position1[1] = Jet(0.0);
  position1[2] = Jet(0.0);
  orientation1[0] = Jet(-2.490);
  orientation1[1] = Jet(-0.206);
  orientation1[2] = Jet(3.066);
  vel_linear1[0] = Jet(0.1);
  vel_linear1[1] = Jet(0.2);
  vel_linear1[2] = Jet(0.1);
  vel_angular1[0] = Jet(1.570796327);
  vel_angular1[1] = Jet(1.570796327);
  vel_angular1[2] = Jet(-1.570796327);
  acc_linear1[0] = Jet(-0.5);
  acc_linear1[1] = Jet(1.0);
  acc_linear1[2] = Jet(1.0);
  dt = Jet(0.1);

  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2);

  EXPECT_DOUBLE_EQ(Jet(-0.012031207341885572).a, position2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.011723254405731805).a, position2[1].a);
  EXPECT_DOUBLE_EQ(Jet(-0.024981300126995967).a, position2[2].a);
  EXPECT_DOUBLE_EQ(Jet(-2.3391131265098766).a, orientation2[0].a);
  EXPECT_DOUBLE_EQ(Jet(-0.4261584872792554).a, orientation2[1].a);
  EXPECT_DOUBLE_EQ(Jet(3.0962756133525855).a, orientation2[2].a);
  EXPECT_DOUBLE_EQ(Jet(0.05).a, vel_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(0.3).a, vel_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(0.2).a, vel_linear2[2].a);
  EXPECT_DOUBLE_EQ(Jet(1.570796327).a, vel_angular2[0].a);
  EXPECT_DOUBLE_EQ(Jet(1.570796327).a, vel_angular2[1].a);
  EXPECT_DOUBLE_EQ(Jet(-1.570796327).a, vel_angular2[2].a);
  EXPECT_DOUBLE_EQ(Jet(-0.5).a, acc_linear2[0].a);
  EXPECT_DOUBLE_EQ(Jet(1.0).a, acc_linear2[1].a);
  EXPECT_DOUBLE_EQ(Jet(1.0).a, acc_linear2[2].a);
}

TEST(Predict, VelocityDecayZeroIsNoOp)
{
  // GIVEN a state with known non-zero velocity
  fuse_core::Vector3d const position1(1.0, 2.0, 0.0);
  Eigen::Quaterniond const orientation1(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
  fuse_core::Vector3d const vel_linear1(0.5, 0.3, 0.0);
  fuse_core::Vector3d const vel_angular1(0.0, 0.0, 0.1);
  fuse_core::Vector3d const acc_linear1(0.0, 0.0, 0.0);
  double const dt = 0.05;

  fuse_core::Vector3d position2;
  Eigen::Quaterniond orientation2;
  fuse_core::Vector3d vel_linear2_default;
  fuse_core::Vector3d vel_angular2_default;
  fuse_core::Vector3d acc_linear2;
  fuse_core::Vector3d vel_linear2_zero;
  fuse_core::Vector3d vel_angular2_zero;

  // WHEN predicting with default (no decay arg) and explicit velocity_decay = 0.0
  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2_default, vel_angular2_default, acc_linear2);
  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2_zero, vel_angular2_zero, acc_linear2, 0.0);

  // THEN results are identical — velocity_decay = 0.0 is a no-op
  EXPECT_DOUBLE_EQ(vel_linear2_default.x(), vel_linear2_zero.x());
  EXPECT_DOUBLE_EQ(vel_linear2_default.y(), vel_linear2_zero.y());
  EXPECT_DOUBLE_EQ(vel_angular2_default.z(), vel_angular2_zero.z());
}

TEST(Predict, VelocityDecayReducesVelocity)
{
  // GIVEN a robot with residual velocity and no acceleration
  fuse_core::Vector3d const position1(0.0, 0.0, 0.0);
  Eigen::Quaterniond const orientation1(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
  fuse_core::Vector3d const vel_linear1(0.2, 0.15, 0.0);
  fuse_core::Vector3d const vel_angular1(0.0, 0.0, 0.1);
  fuse_core::Vector3d const acc_linear1(0.0, 0.0, 0.0);
  double const dt = 0.05;
  double const k = 1.0;
  double const expected_decay_factor = std::exp(-k * dt);

  fuse_core::Vector3d position2;
  Eigen::Quaterniond orientation2;
  fuse_core::Vector3d acc_linear2;
  fuse_core::Vector3d vel_linear2;
  fuse_core::Vector3d vel_angular2;
  fuse_core::Vector3d vel_linear2_no_decay;
  fuse_core::Vector3d vel_angular2_no_decay;

  // WHEN predicting with decay enabled
  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2, k);

  // THEN velocity is multiplied by exp(-k * dt)
  EXPECT_NEAR(vel_linear2.x(), vel_linear1.x() * expected_decay_factor, 1e-9);
  EXPECT_NEAR(vel_linear2.y(), vel_linear1.y() * expected_decay_factor, 1e-9);
  EXPECT_NEAR(vel_linear2.z(), vel_linear1.z() * expected_decay_factor, 1e-9);
  EXPECT_NEAR(vel_angular2.z(), vel_angular1.z() * expected_decay_factor, 1e-9);

  // WHEN predicting with no decay
  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2_no_decay, vel_angular2_no_decay, acc_linear2, 0.0);

  // THEN undecayed velocity is strictly larger
  EXPECT_GT(std::abs(vel_linear2_no_decay.x()), std::abs(vel_linear2.x()));
  EXPECT_GT(std::abs(vel_linear2_no_decay.y()), std::abs(vel_linear2.y()));
}

TEST(Predict, VelocityDecayJacobiansMatchDecayFactor)
{
  // GIVEN a state and a known decay rate
  double const k = 1.0;
  double const dt = 0.05;
  double const expected_decay_factor = std::exp(-k * dt);

  double position1[3] = { 0.0, 0.0, 0.0 };
  double orientation1[3] = { 0.0, 0.0, 0.0 };
  double vel_linear1[3] = { 0.2, 0.15, 0.0 };
  double vel_angular1[3] = { 0.0, 0.0, 0.1 };
  double acc_linear1[3] = { 0.0, 0.0, 0.0 };
  double position2[3] = {};
  double orientation2[3] = {};
  double vel_linear2[3] = {};
  double vel_angular2[3] = {};
  double acc_linear2[3] = {};
  // Parameter block sizes: position=3, orientation=4, vel_linear=3, vel_angular=3, acc_linear=3
  std::array<double, 15UL * 3UL> j0{};
  std::array<double, 15UL * 4UL> j1{};  // orientation block is 15x4
  std::array<double, 15UL * 3UL> j2{};
  std::array<double, 15UL * 3UL> j3{};
  std::array<double, 15UL * 3UL> j4{};
  std::array<double*, 5> jacobians = { j0.data(), j1.data(), j2.data(), j3.data(), j4.data() };
  double j_quat2rpy[12] = {};

  // WHEN computing analytical Jacobians with decay enabled
  fuse_models::predict(position1[0], position1[1], position1[2], orientation1[0], orientation1[1], orientation1[2],
                       vel_linear1[0], vel_linear1[1], vel_linear1[2], vel_angular1[0], vel_angular1[1],
                       vel_angular1[2], acc_linear1[0], acc_linear1[1], acc_linear1[2], dt, position2[0], position2[1],
                       position2[2], orientation2[0], orientation2[1], orientation2[2], vel_linear2[0], vel_linear2[1],
                       vel_linear2[2], vel_angular2[0], vel_angular2[1], vel_angular2[2], acc_linear2[0],
                       acc_linear2[1], acc_linear2[2], jacobians.data(), j_quat2rpy, k);

  // THEN d(vel_linear2_i)/d(vel_linear1_i) = decay_factor (not 1.0)
  Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> const j_vel_linear(j2.data());
  EXPECT_NEAR(j_vel_linear(6, 0), expected_decay_factor, 1e-9);
  EXPECT_NEAR(j_vel_linear(7, 1), expected_decay_factor, 1e-9);
  EXPECT_NEAR(j_vel_linear(8, 2), expected_decay_factor, 1e-9);

  // AND d(vel_angular2_i)/d(vel_angular1_i) = decay_factor (not 1.0)
  Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> const j_vel_angular(j3.data());
  EXPECT_NEAR(j_vel_angular(9, 0), expected_decay_factor, 1e-9);
  EXPECT_NEAR(j_vel_angular(10, 1), expected_decay_factor, 1e-9);
  EXPECT_NEAR(j_vel_angular(11, 2), expected_decay_factor, 1e-9);
}

TEST(Predict, VelocityDecayAppliesToFullPrediction)
{
  // GIVEN a robot with both non-zero velocity and non-zero acceleration.
  // Pins: vel2 = (vel1 + acc*dt) * decay_factor, not vel1*decay_factor + acc*dt.
  fuse_core::Vector3d const position1(0.0, 0.0, 0.0);
  Eigen::Quaterniond const orientation1(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
  fuse_core::Vector3d const vel_linear1(0.2, 0.0, 0.0);
  fuse_core::Vector3d const vel_angular1(0.0, 0.0, 0.0);
  fuse_core::Vector3d const acc_linear1(1.0, 0.0, 0.0);
  double const dt = 0.05;
  double const k = 1.0;
  double const expected_decay_factor = std::exp(-k * dt);

  fuse_core::Vector3d position2;
  Eigen::Quaterniond orientation2;
  fuse_core::Vector3d vel_linear2;
  fuse_core::Vector3d vel_angular2;
  fuse_core::Vector3d acc_linear2;

  // WHEN predicting with decay enabled
  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2, k);

  // THEN vel2.x = (vel1.x + acc.x*dt) * decay_factor
  double const expected_vx = (vel_linear1.x() + acc_linear1.x() * dt) * expected_decay_factor;
  EXPECT_NEAR(vel_linear2.x(), expected_vx, 1e-9);

  // AND the old formula vel1*decay + acc*dt gives a different result, confirming acc is decayed too
  double const old_formula_vx = vel_linear1.x() * expected_decay_factor + acc_linear1.x() * dt;
  EXPECT_NE(vel_linear2.x(), old_formula_vx);
}

TEST(Predict, VelocityDecayAccelerationJacobianIsScaledByDecayFactor)
{
  // GIVEN a state and a known decay rate
  double const k = 1.0;
  double const dt = 0.05;
  double const expected_decay_factor = std::exp(-k * dt);

  double position1[3] = { 0.0, 0.0, 0.0 };
  double orientation1[3] = { 0.0, 0.0, 0.0 };
  double vel_linear1[3] = { 0.2, 0.15, 0.0 };
  double vel_angular1[3] = { 0.0, 0.0, 0.1 };
  double acc_linear1[3] = { 0.0, 0.0, 0.0 };
  double position2[3] = {};
  double orientation2[3] = {};
  double vel_linear2[3] = {};
  double vel_angular2[3] = {};
  double acc_linear2[3] = {};
  // Parameter block sizes: position=3, orientation=4, vel_linear=3, vel_angular=3, acc_linear=3
  std::array<double, 15UL * 3UL> j0{};
  std::array<double, 15UL * 4UL> j1{};  // orientation block is 15x4
  std::array<double, 15UL * 3UL> j2{};
  std::array<double, 15UL * 3UL> j3{};
  std::array<double, 15UL * 3UL> j4{};
  std::array<double*, 5> jacobians = { j0.data(), j1.data(), j2.data(), j3.data(), j4.data() };
  double j_quat2rpy[12] = {};

  // WHEN computing analytical Jacobians with decay enabled
  fuse_models::predict(position1[0], position1[1], position1[2], orientation1[0], orientation1[1], orientation1[2],
                       vel_linear1[0], vel_linear1[1], vel_linear1[2], vel_angular1[0], vel_angular1[1],
                       vel_angular1[2], acc_linear1[0], acc_linear1[1], acc_linear1[2], dt, position2[0], position2[1],
                       position2[2], orientation2[0], orientation2[1], orientation2[2], vel_linear2[0], vel_linear2[1],
                       vel_linear2[2], vel_angular2[0], vel_angular2[1], vel_angular2[2], acc_linear2[0],
                       acc_linear2[1], acc_linear2[2], jacobians.data(), j_quat2rpy, k);

  // THEN d(vel_linear2_i)/d(acc_linear1_i) = dt * decay_factor (not plain dt)
  Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> const j_acc(j4.data());
  EXPECT_NEAR(j_acc(6, 0), dt * expected_decay_factor, 1e-9);
  EXPECT_NEAR(j_acc(7, 1), dt * expected_decay_factor, 1e-9);
  EXPECT_NEAR(j_acc(8, 2), dt * expected_decay_factor, 1e-9);
}

TEST(Predict, VelocityDecaysGeometricallyOverMultipleSteps)
{
  // GIVEN a robot with residual velocity and no acceleration (simulates odom going silent)
  fuse_core::Vector3d const position1(0.0, 0.0, 0.0);
  Eigen::Quaterniond const orientation1(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
  fuse_core::Vector3d vel_linear(0.5, 0.0, 0.0);
  fuse_core::Vector3d vel_angular(0.0, 0.0, 0.0);
  fuse_core::Vector3d const acc_linear(0.0, 0.0, 0.0);
  double const dt = 0.05;
  double const k = 1.0;
  double const decay_factor = std::exp(-k * dt);

  // WHEN predicting 20 steps (1 second) with no sensor corrections
  fuse_core::Vector3d position;
  Eigen::Quaterniond orientation;
  fuse_core::Vector3d acc_out;
  for (int i = 0; i < 20; ++i)
  {
    fuse_core::Vector3d vel_next;
    fuse_core::Vector3d vel_angular_next;
    fuse_models::predict(position1, orientation1, vel_linear, vel_angular, acc_linear, dt, position, orientation,
                         vel_next, vel_angular_next, acc_out, k);
    // THEN each step velocity is multiplied by decay_factor
    EXPECT_NEAR(vel_next.x(), vel_linear.x() * decay_factor, 1e-9);
    vel_linear = vel_next;
    vel_angular = vel_angular_next;
  }

  // AND after 1 second velocity has decayed to vel0 * exp(-k * 1.0)
  EXPECT_NEAR(vel_linear.x(), 0.5 * std::exp(-k * 1.0), 1e-6);
}

TEST(Predict, predictJacobians)
{
  // GIVEN a state away from gimbal lock (small angles keep the RPY parameterization well conditioned)
  double const dt = 0.1;
  const fuse_core::Vector3d position1(0.1, -0.2, 0.3);
  const fuse_core::Vector3d vel_linear1(1.0, 0.2, -0.1);
  const fuse_core::Vector3d vel_angular1(0.3, -0.2, 0.5);
  const fuse_core::Vector3d acc_linear1(0.5, -0.4, 0.2);
  const Eigen::Quaterniond orientation1 = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) *
                                          Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX());

  // Extract RPY with the same convention predict() uses so the autodiff input matches the analytic input.
  double const quat[4] = { orientation1.w(), orientation1.x(), orientation1.y(), orientation1.z() };
  double rpy[3];
  fuse_core::quaternion2rpy(quat, rpy);

  // WHEN computing the analytic 15x15 state Jacobian (orientation columns in RPY space)
  fuse_core::Vector3d position2;
  fuse_core::Vector3d vel_linear2;
  fuse_core::Vector3d vel_angular2;
  fuse_core::Vector3d acc_linear2;
  Eigen::Quaterniond orientation2;
  fuse_core::Matrix15d jacobian_analytic;
  fuse_models::predict(position1, orientation1, vel_linear1, vel_angular1, acc_linear1, dt, position2, orientation2,
                       vel_linear2, vel_angular2, acc_linear2, jacobian_analytic);

  // AND the same Jacobian by autodiff through the templated (RPY-in, RPY-out) overload
  using Jet = ceres::Jet<double, 15>;
  const std::array<Jet, 15> x{
    Jet(position1.x(), 0),    Jet(position1.y(), 1),    Jet(position1.z(), 2),     Jet(rpy[0], 3),
    Jet(rpy[1], 4),           Jet(rpy[2], 5),           Jet(vel_linear1.x(), 6),   Jet(vel_linear1.y(), 7),
    Jet(vel_linear1.z(), 8),  Jet(vel_angular1.x(), 9), Jet(vel_angular1.y(), 10), Jet(vel_angular1.z(), 11),
    Jet(acc_linear1.x(), 12), Jet(acc_linear1.y(), 13), Jet(acc_linear1.z(), 14)
  };
  std::array<Jet, 3> position2_jet;
  std::array<Jet, 3> orientation2_jet;
  std::array<Jet, 3> vel_linear2_jet;
  std::array<Jet, 3> vel_angular2_jet;
  std::array<Jet, 3> acc_linear2_jet;
  fuse_models::predict(x.data(), x.data() + 3, x.data() + 6, x.data() + 9, x.data() + 12, Jet(dt), position2_jet.data(),
                       orientation2_jet.data(), vel_linear2_jet.data(), vel_angular2_jet.data(),
                       acc_linear2_jet.data());

  fuse_core::Matrix15d jacobian_autodiff;
  const std::array<std::array<Jet, 3> const*, 5> outputs{ &position2_jet, &orientation2_jet, &vel_linear2_jet,
                                                          &vel_angular2_jet, &acc_linear2_jet };
  for (int block = 0; block < 5; ++block)
  {
    for (int row = 0; row < 3; ++row)
    {
      jacobian_autodiff.row(block * 3 + row) = (*outputs[block])[row].v.transpose();
    }
  }

  // THEN the analytic Jacobian matches autodiff. This guards the quaternion->RPY conversion of the
  // orientation columns: the prior truncated assembly would fail this check.
  EXPECT_MATRIX_NEAR(jacobian_autodiff, jacobian_analytic, 1e-9);
}
