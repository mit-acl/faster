#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/frame_manager.h>

#include <rviz/load_resource.h>

#include <planning_ros_msgs/PathArray.h>
#include <rviz/message_filter_display.h>

#include "path_visual.h"

namespace planning_rviz_plugins {
  class PathArrayDisplay
    : public rviz::MessageFilterDisplay<planning_ros_msgs::PathArray> {
      Q_OBJECT
      public:
        PathArrayDisplay();
        virtual ~PathArrayDisplay();

      protected:
        virtual void onInitialize();

        virtual void reset();

        private Q_SLOTS:
          void updateLineColorAndAlpha();
        void updateNodeColorAndAlpha();
        void updateLineScale();
        void updateNodeScale();
        void updateID();

      private:
        void processMessage(const planning_ros_msgs::PathArray::ConstPtr &msg);
        void visualizeMessage(int state);

        std::shared_ptr<PathVisual> visual_;

        rviz::ColorProperty *line_color_property_;
        rviz::ColorProperty *node_color_property_;
        rviz::FloatProperty *line_scale_property_;
        rviz::FloatProperty *node_scale_property_;
        rviz::EnumProperty *id_property_;

        Ogre::Vector3 position_;
        Ogre::Quaternion orientation_;

        planning_ros_msgs::PathArray paths_;
    };

}
