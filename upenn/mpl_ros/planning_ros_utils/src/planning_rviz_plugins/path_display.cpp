#include <tf/transform_listener.h>
#include "path_display.h"

namespace planning_rviz_plugins {
  PathDisplay::PathDisplay() {
    line_color_property_ =
      new rviz::ColorProperty("LineColor", QColor(204, 51, 204), "Color to draw the line.",
                              this, SLOT(updateLineColorAndAlpha()));
    node_color_property_ =
      new rviz::ColorProperty("NodeColor", QColor(85, 85, 255), "Color to draw the node.",
                              this, SLOT(updateNodeColorAndAlpha()));
    line_scale_property_ =
      new rviz::FloatProperty("LineScale", 0.1, "0.1 is the default value.",
                              this, SLOT(updateLineScale()));
    node_scale_property_ =
      new rviz::FloatProperty("NodeScale", 0.15, "0.15 is the default value.",
                              this, SLOT(updateNodeScale()));
  }

  void PathDisplay::onInitialize() {
    MFDClass::onInitialize();
  }

  PathDisplay::~PathDisplay() {}

  void PathDisplay::reset() {
    MFDClass::reset();
    visual_ = nullptr;
  }

  void PathDisplay::updateLineColorAndAlpha() {
    Ogre::ColourValue color = line_color_property_->getOgreColor();
    if(visual_)
      visual_->setLineColor(color.r, color.g, color.b, 1);
  }

  void PathDisplay::updateNodeColorAndAlpha() {
    Ogre::ColourValue color = node_color_property_->getOgreColor();
    if(visual_)
      visual_->setNodeColor(color.r, color.g, color.b, 1);
  }

  void PathDisplay::updateLineScale() {
    float s = line_scale_property_->getFloat();
    if(visual_)
      visual_->setLineScale(s);
  }

  void PathDisplay::updateNodeScale() {
    float s = node_scale_property_->getFloat();
    if(visual_)
      visual_->setNodeScale(s);
  }

  void PathDisplay::processMessage(const planning_ros_msgs::Path::ConstPtr &msg) {
    if (!context_->getFrameManager()->getTransform(
                                                   msg->header.frame_id, msg->header.stamp, position_, orientation_)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

    path_ = ros_to_path(*msg);

    visualizeMessage();
  }

  void PathDisplay::visualizeMessage() {
    std::shared_ptr<PathVisual> visual;
    visual.reset(new PathVisual(context_->getSceneManager(), scene_node_));

    visual->setMessage(path_);
    visual->setFramePosition(position_);
    visual->setFrameOrientation(orientation_);

    float line_scale = line_scale_property_->getFloat();
    visual->setLineScale(line_scale);
    float node_scale = node_scale_property_->getFloat();
    visual->setNodeScale(node_scale);

    Ogre::ColourValue line_color = line_color_property_->getOgreColor();
    visual->setLineColor(line_color.r, line_color.g, line_color.b, 1);
    Ogre::ColourValue node_color = node_color_property_->getOgreColor();
    visual->setNodeColor(node_color.r, node_color.g, node_color.b, 1);

    visual_ = visual;
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning_rviz_plugins::PathDisplay, rviz::Display)
