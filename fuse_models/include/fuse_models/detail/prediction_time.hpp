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
#ifndef FUSE_MODELS__DETAIL__PREDICTION_TIME_HPP_
#define FUSE_MODELS__DETAIL__PREDICTION_TIME_HPP_

#include <algorithm>

namespace fuse_models::detail
{

/**
 * @brief Compute the forward time delta used to extrapolate the latest optimized state up to the
 *        current time in the odometry publishers (predict_to_current_time).
 *
 * The publisher predicts from the latest optimized state (at @p stamp_sec) forward to the wall-clock
 * time it is publishing for (@p to_predict_to_sec). That delta must be clamped to be non-negative:
 * we never want to predict *backwards* if the optimized state is momentarily ahead of the publish
 * clock. It must NOT be clamped to be non-positive -- doing so forces dt == 0 and silently disables
 * forward prediction, so the publisher emits a stale pose stamped as "now" (a latency bug; see the
 * regression test). Hence std::max(..., 0.0), never std::min(..., 0.0).
 *
 * @param[in] to_predict_to_sec Time to predict the state to (seconds), typically wall-clock now.
 * @param[in] stamp_sec         Timestamp of the latest optimized state (seconds).
 * @return The non-negative forward prediction interval in seconds.
 */
inline double forwardPredictionDt(double const to_predict_to_sec, double const stamp_sec)
{
  return std::max(to_predict_to_sec - stamp_sec, 0.0);
}

}  // namespace fuse_models::detail

#endif  // FUSE_MODELS__DETAIL__PREDICTION_TIME_HPP_
