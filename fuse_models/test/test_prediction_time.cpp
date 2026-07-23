/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, PickNik Robotics, Inc.
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

#include <fuse_models/detail/prediction_time.hpp>

using fuse_models::detail::forwardPredictionDt;

// Regression test for the predict_to_current_time latency bug.
//
// The odometry publishers extrapolate the latest optimized state forward to the current time.
// A prior change computed this delta as std::min(to - stamp, 0.0), which is always <= 0 in normal
// forward operation, forcing dt == 0 and silently disabling forward prediction: the publisher then
// emitted a stale pose stamped as "now". The correct behavior clamps the delta to be non-negative
// (std::max(..., 0.0)) so forward prediction is preserved while backward prediction is prevented.

TEST(PredictionTime, ForwardIntervalIsPreserved)
{
  // The whole point of predict_to_current_time: a future target must yield a positive dt so the
  // state is actually extrapolated forward. The std::min bug returned 0.0 here.
  // (EXPECT_NEAR, not EXPECT_DOUBLE_EQ: 10.1 - 10.0 is not exactly 0.1 in floating point.)
  EXPECT_NEAR(0.1, forwardPredictionDt(10.1, 10.0), 1e-12);
  EXPECT_DOUBLE_EQ(0.5, forwardPredictionDt(100.5, 100.0));  // 0.5 is exactly representable
  EXPECT_GT(forwardPredictionDt(10.1, 10.0), 0.0);
}

TEST(PredictionTime, BackwardIntervalIsClampedToZero)
{
  // If the optimized state is momentarily ahead of the publish clock, do not predict backwards.
  EXPECT_DOUBLE_EQ(0.0, forwardPredictionDt(9.9, 10.0));
  EXPECT_DOUBLE_EQ(0.0, forwardPredictionDt(10.0, 10.0));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
