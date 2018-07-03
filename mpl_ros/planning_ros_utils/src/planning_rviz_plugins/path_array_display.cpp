#include "path_array_display.h"

namespace planning_rviz_plugins {
  PathArrayDisplay::PathArrayDisplay() {
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
    id_property_ =
      new rviz::EnumProperty("ID", "All", "Visualize individual path using its id, All is the default",
                             this, SLOT(updateID()));
  }

  void PathArrayDisplay::onInitialize() {
    MFDClass::onInitialize();
  }

  PathArrayDisplay::~PathArrayDisplay() {}

  void PathArrayDisplay::reset() {
    MFDClass::reset();
    visual_ = nullptr;
  }

  void PathArrayDisplay::updateLineColorAndAlpha() {
    Ogre::ColourValue color = line_color_property_->getOgreColor();
    if(visual_)
      visual_->setLineColor(color.r, color.g, color.b, 1);
  }

  void PathArrayDisplay::updateNodeColorAndAlpha() {
    Ogre::ColourValue color = node_color_property_->getOgreColor();
    if(visual_)
      visual_->setNodeColor(color.r, color.g, color.b, 1);
  }

  void PathArrayDisplay::updateLineScale() {
    float s = line_scale_property_->getFloat();
    if(visual_)
      visual_->setLineScale(s);
  }

  void PathArrayDisplay::updateNodeScale() {
    float s = node_scale_property_->getFloat();
    if(visual_)
      visual_->setNodeScale(s);
  }

  void PathArrayDisplay::processMessage(const planning_ros_msgs::PathArray::ConstPtr &msg) {
    if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position_, orientation_)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

    paths_ = *msg;

    id_property_->clearOptions();
    id_property_->addOption("All", -1);
    for (int i = 0; i < (int) paths_.paths.size(); i++) {
      if(paths_.paths[i].name.empty())
        id_property_->addOption(QString::number(i), i);
      else
        id_property_->addOption(QString(paths_.paths[i].name.c_str()), i);
    }

    int id = id_property_->getOptionInt();
    visualizeMessage(id);
  }


  void PathArrayDisplay::visualizeMessage(int id) {
    if (id >= (int) paths_.paths.size() || paths_.paths.empty())
      return;

    std::shared_ptr<PathVisual> visual;
    visual.reset(new PathVisual(context_->getSceneManager(), scene_node_));

    bool set = false;
    if(id >= 0) {
      set = true;
      visual->setMessage(ros_to_path(paths_.paths[id]));
      visual->setFramePosition(position_);
      visual->setFrameOrientation(orientation_);
    }
    else {
      for (int i = 0; i < (int)paths_.paths.size(); i++)
      {
        if(i == 0) {
          set = true;
          visual->setMessage(ros_to_path(paths_.paths[i]));
          visual->setFramePosition(position_);
          visual->setFrameOrientation(orientation_);
          continue;
        }
        visual->addMessage(ros_to_path(paths_.paths[i]));
      }
    }

    if(set){
      float line_scale = line_scale_property_->getFloat();
      visual->setLineScale(line_scale);
      float node_scale = node_scale_property_->getFloat();
      visual->setNodeScale(node_scale);

      Ogre::ColourValue line_color = line_color_property_->getOgreColor();
      visual->setLineColor(line_color.r, line_color.g, line_color.b, 1);
      Ogre::ColourValue node_color = node_color_property_->getOgreColor();
      visual->setNodeColor(node_color.r, node_color.g, node_color.b, 1);
    }
    visual_ = visual;
  }

  void PathArrayDisplay::updateID() {
    int id = id_property_->getOptionInt();
    visualizeMessage(id);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning_rviz_plugins::PathArrayDisplay, rviz::Display)
