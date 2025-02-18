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

#ifndef FUSE_VIZ__RELATIVE_POSE_2D_STAMPED_CONSTRAINT_PROPERTY_HPP_
#define FUSE_VIZ__RELATIVE_POSE_2D_STAMPED_CONSTRAINT_PROPERTY_HPP_

#include <OgreColourValue.h>

#include <memory>
#include <string>
#include <unordered_map>

#include <fuse_core/uuid.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/property.hpp>

namespace Ogre
{

class SceneManager;
class SceneNode;

}  // namespace Ogre

namespace fuse_core
{

class Graph;

}  // namespace fuse_core

namespace fuse_constraints
{

class RelativePose2DStampedConstraint;

}  // namespace fuse_constraints

namespace fuse_viz
{

class Pose2DStampedVisual;
class RelativePose2DStampedConstraintVisual;

using rviz_common::properties::BoolProperty;
using rviz_common::properties::ColorProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::Property;

class MappedCovarianceProperty;

class RelativePose2DStampedConstraintProperty : public BoolProperty
{
  Q_OBJECT

public:
  using Visual = RelativePose2DStampedConstraintVisual;
  using VisualPtr = std::shared_ptr<Visual>;

  RelativePose2DStampedConstraintProperty(QString const& name = "RelativePose2DStampedConstraint",
                                          bool default_value = true, QString const& description = QString(),
                                          Property* parent = NULL, char const* changed_slot = NULL,
                                          QObject* receiver = NULL);

  ~RelativePose2DStampedConstraintProperty() override = default;

  VisualPtr createAndInsertVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
                                  fuse_constraints::RelativePose2DStampedConstraint const& constraint,
                                  fuse_core::Graph const& graph);
  void eraseVisual(fuse_core::UUID const& uuid);
  void clearVisual();

  void setColor(QColor const& color);

public Q_SLOTS:
  void updateVisibility();

private Q_SLOTS:
  void updateColor();
  void updateErrorLineAlpha();
  void updateErrorLineWidth();
  void updateLossMinBrightness();
  void updateRelativePoseAxesAlpha();
  void updateRelativePoseAxesScale();
  void updateRelativePoseLineAlpha();
  void updateRelativePoseLineWidth();
  void updateShowText();
  void updateTextScale();

private:
  void updateColor(VisualPtr const& constraint);
  void updateErrorLineAlpha(VisualPtr const& constraint);
  void updateErrorLineWidth(VisualPtr const& constraint);
  void updateLossMinBrightness(VisualPtr const& constraint);
  void updateRelativePoseAxesAlpha(VisualPtr const& constraint);
  void updateRelativePoseAxesScale(VisualPtr const& constraint);
  void updateRelativePoseLineAlpha(VisualPtr const& constraint);
  void updateRelativePoseLineWidth(VisualPtr const& constraint);
  void updateShowText(VisualPtr const& constraint);
  void updateTextScale(VisualPtr const& constraint);
  void updateVisibility(VisualPtr const& constraint);

  std::unordered_map<fuse_core::UUID, VisualPtr, fuse_core::uuid::hash> constraints_;

  ColorProperty* color_property_;
  BoolProperty* show_text_property_;
  FloatProperty* text_scale_property_;
  FloatProperty* relative_pose_axes_alpha_property_;
  FloatProperty* relative_pose_axes_scale_property_;
  FloatProperty* relative_pose_line_alpha_property_;
  FloatProperty* relative_pose_line_width_property_;
  FloatProperty* error_line_alpha_property_;
  FloatProperty* error_line_width_property_;
  FloatProperty* loss_min_brightness_property_;
  MappedCovarianceProperty* covariance_property_;
};

}  // namespace fuse_viz

#endif  // FUSE_VIZ__RELATIVE_POSE_2D_STAMPED_CONSTRAINT_PROPERTY_HPP_
