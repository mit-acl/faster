#ifndef TRAJECTORY_VISUAL_H
#define TRAJECTORY_VISUAL_H

#include <planning_ros_utils/primitive_ros_utils.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/billboard_line.h>

namespace planning_rviz_plugins {
  class TrajectoryVisual {
    public:
      TrajectoryVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);

      virtual ~TrajectoryVisual();

      void setNum(int n);
      void setMessage(const planning_ros_msgs::Trajectory &msg);
      void addMessage(const planning_ros_msgs::Trajectory &msg);

      void setFramePosition(const Ogre::Vector3 &position);
      void setFrameOrientation(const Ogre::Quaternion &orientation);

      void setPosColor(float r, float g, float b, float a);
      void setVelColor(float r, float g, float b, float a);
      void setAccColor(float r, float g, float b, float a);
      void setJrkColor(float r, float g, float b, float a);
      void setPosScale(float s);
      void setVelScale(float s);
      void setAccScale(float s);
      void setJrkScale(float s);
      void setVelVis(bool vis);
      void setAccVis(bool vis);
      void setJrkVis(bool vis);

    private:
      std::vector<std::unique_ptr<rviz::BillboardLine>> poss_;
      std::vector<std::unique_ptr<rviz::BillboardLine>> vels_;
      std::vector<std::unique_ptr<rviz::BillboardLine>> accs_;
      std::vector<std::unique_ptr<rviz::BillboardLine>> jrks_;

      Ogre::SceneNode *frame_node_;
      Ogre::SceneManager *scene_manager_;

      int num_;
      bool vel_vis_;
      bool acc_vis_;
      bool jrk_vis_;
  };
}

#endif
