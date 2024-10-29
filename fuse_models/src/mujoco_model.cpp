#include <fuse_models/mujoco_model.hpp>

namespace fuse_models
{
MujocoMotionModel::MujocoMotionModel()
  : fuse_core::AsyncMotionModel(1)
  , logger_(rclcpp::get_logger("mujoco_model"))
  , device_id_(fuse_core::uuid::NIL)
  , timestamp_manager_(&MujocoMotionModel::generateMotionModel, this, rclcpp::Duration::max())
{
}

void MujocoMotionModel::generateMotionModel(const rclcpp::Time& beginning_stamp, const rclcpp::Time& ending_stamp,
                                            std::vector<fuse_core::Constraint::SharedPtr>& constraints,
                                            std::vector<fuse_core::Variable::SharedPtr>& variables)
{
  // TODO(henrygerardmoore): implement this
}

void MujocoMotionModel::onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph)
{
  // TODO(henrygerardmoore): see if we need to emulate unicycle_2d for this
  this->graph_ = std::move(graph);
}

void MujocoMotionModel::onInit()
{
  // TODO(henrygerardmoore): implement this
  // model_ = fuse::getParam...
  // data_ = ...
}

bool MujocoMotionModel::applyCallback(fuse_core::Transaction& transaction)
{
  // Use the timestamp manager to generate just the required motion model segments. The timestamp
  // manager, in turn, makes calls to the generateMotionModel() function.
  try
  {
    // Now actually generate the motion model segments
    timestamp_manager_.query(transaction, true);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 10.0 * 1000,
                                 "An error occurred while completing the motion model query. Error: " << e.what());
    return false;
  }
  return true;
}
}  // namespace fuse_models
