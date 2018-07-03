#include <tf/transform_listener.h>
#include "ellipsoids_display.h"

namespace decomp_rviz_plugins {

EllipsoidsDisplay::EllipsoidsDisplay() {
  color_property_ = new rviz::ColorProperty("Color", QColor(204, 51, 204),
                                            "Color of ellipsoids.",
                                            this, SLOT(updateColorAndAlpha()));
  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 0.5, "0 is fully transparent, 1.0 is fully opaque.", this,
      SLOT(updateColorAndAlpha()));

}

void EllipsoidsDisplay::onInitialize() {
  MFDClass::onInitialize();
}

EllipsoidsDisplay::~EllipsoidsDisplay() {}

void EllipsoidsDisplay::reset() {
  MFDClass::reset();
  visual_ = nullptr;
}

void EllipsoidsDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  if (visual_)
    visual_->setColor(color.r, color.g, color.b, alpha);
}

void EllipsoidsDisplay::processMessage(const decomp_ros_msgs::Ellipsoids::ConstPtr &msg) {
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  std::shared_ptr<EllipsoidsVisual> visual;
  visual.reset(new EllipsoidsVisual(context_->getSceneManager(), scene_node_));

  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor(color.r, color.g, color.b, alpha);

  visual_ = visual;
}

} 

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(decomp_rviz_plugins::EllipsoidsDisplay, rviz::Display)
