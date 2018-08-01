#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/frame_manager.h>

#include <rviz/load_resource.h>

#include <decomp_ros_msgs/Ellipsoids.h>
#include <rviz/message_filter_display.h>
#include "ellipsoids_visual.h"

namespace decomp_rviz_plugins {

  class EllipsoidsVisual;

  class EllipsoidsDisplay
    : public rviz::MessageFilterDisplay<decomp_ros_msgs::Ellipsoids> {
      Q_OBJECT
      public:
        EllipsoidsDisplay();
        ~EllipsoidsDisplay();

      protected:
        void onInitialize();

        void reset();

        private Q_SLOTS:
          void updateColorAndAlpha();

      private:
        void processMessage(const decomp_ros_msgs::Ellipsoids::ConstPtr &msg);

        std::shared_ptr<EllipsoidsVisual> visual_;

        rviz::ColorProperty *color_property_;
        rviz::FloatProperty *alpha_property_;
    };

}
