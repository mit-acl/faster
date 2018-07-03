#include <decomp_ros_msgs/Polyhedra.h>
#include <rviz/message_filter_display.h>

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/load_resource.h>

#include "geometry_utils.h"
#include "mesh_visual.h"
#include "bound_visual.h"
#include "vector_visual.h"

namespace decomp_rviz_plugins {
class PolyhedraDisplay
    : public rviz::MessageFilterDisplay<decomp_ros_msgs::Polyhedra> {
  Q_OBJECT
public:
  PolyhedraDisplay();
  virtual ~PolyhedraDisplay();

protected:
  virtual void onInitialize();

  virtual void reset();

private Q_SLOTS:
  void updateMeshColorAndAlpha();
  void updateBoundColorAndAlpha();
  void updateVsColorAndAlpha();
  void updateState();
  void updateScale();
  void updateVsScale();

private:
  void processMessage(const decomp_ros_msgs::Polyhedra::ConstPtr &msg);
  void visualizeMessage(int state);
  void visualizeMesh();
  void visualizeBound();
  void visualizeVs();

  std::shared_ptr<MeshVisual> visual_mesh_;
  std::shared_ptr<BoundVisual> visual_bound_;
  std::shared_ptr<VectorVisual> visual_vector_;

  rviz::ColorProperty *mesh_color_property_;
  rviz::ColorProperty *bound_color_property_;
  rviz::ColorProperty *vs_color_property_;
  rviz::FloatProperty *alpha_property_;
  rviz::FloatProperty *scale_property_;
  rviz::FloatProperty *vs_scale_property_;
  rviz::EnumProperty *state_property_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  BoundVec3f vertices_;
  std::vector<bool> passes_;
  vec_E<std::pair<Vec3f, Vec3f>> vs_;
};

} 
