/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
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

#include <algorithm>
#include <vector>

#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_optimizers/variable_stamp_index.hpp>
#include <fuse_variables/stamped.hpp>
#include <rclcpp/time.hpp>

namespace fuse_optimizers
{
rclcpp::Time VariableStampIndex::currentStamp() const
{
  auto compare_stamps = [](StampedMap::value_type const& lhs, StampedMap::value_type const& rhs) {
    return lhs.second < rhs.second;
  };
  auto iter = std::max_element(stamped_index_.begin(), stamped_index_.end(), compare_stamps);
  if (iter != stamped_index_.end())
  {
    return iter->second;
  }
  return { 0, 0, RCL_ROS_TIME };
}

void VariableStampIndex::addNewTransaction(fuse_core::Transaction const& transaction)
{
  applyAddedVariables(transaction);
  applyAddedConstraints(transaction);
  applyRemovedConstraints(transaction);
  applyRemovedVariables(transaction);
}

void VariableStampIndex::addMarginalTransaction(fuse_core::Transaction const& transaction)
{
  // Only the removed variables and removed constraints should be applied to the VariableStampIndex
  // No variables will be added by a marginal transaction, and the added constraints add variable
  // links that we *do not* want to track. These links are merely an artifact of the marginalization
  // process.
  applyRemovedConstraints(transaction);
  applyRemovedVariables(transaction);
}

void VariableStampIndex::applyAddedConstraints(fuse_core::Transaction const& transaction)
{
  for (auto const& constraint : transaction.addedConstraints())
  {
    constraints_[constraint.uuid()].insert(constraint.variables().begin(), constraint.variables().end());
    for (auto const& variable_uuid : constraint.variables())
    {
      variables_[variable_uuid].insert(constraint.uuid());
    }
  }
}

void VariableStampIndex::applyAddedVariables(fuse_core::Transaction const& transaction)
{
  for (auto const& variable : transaction.addedVariables())
  {
    auto const* stamped_variable = dynamic_cast<fuse_variables::Stamped const*>(&variable);
    if (stamped_variable != nullptr)
    {
      stamped_index_[variable.uuid()] = stamped_variable->stamp();
    }
    variables_[variable.uuid()];  // Add an empty set of constraints
  }
}

void VariableStampIndex::applyRemovedConstraints(fuse_core::Transaction const& transaction)
{
  for (auto const& constraint_uuid : transaction.removedConstraints())
  {
    for (auto const& variable_uuid : constraints_[constraint_uuid])
    {
      variables_[variable_uuid].erase(constraint_uuid);
    }
    constraints_.erase(constraint_uuid);
  }
}

void VariableStampIndex::applyRemovedVariables(fuse_core::Transaction const& transaction)
{
  for (auto const& variable_uuid : transaction.removedVariables())
  {
    stamped_index_.erase(variable_uuid);
    variables_.erase(variable_uuid);
  }
}

}  // namespace fuse_optimizers
