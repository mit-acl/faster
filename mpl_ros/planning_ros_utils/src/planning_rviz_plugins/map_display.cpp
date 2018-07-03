#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "map_display.h"

namespace planning_rviz_plugins {

MapDisplay::MapDisplay()
    : point_cloud_common_(new rviz::PointCloudCommon(this)) {
  // PointCloudCommon sets up a callback queue with a thread for each
  // instance.  Use that for processing incoming messages.

  state_property_ = new rviz::EnumProperty(
      "State", "Occupied", "Visualize voxels at three different states: "
      "Occupied, Free and Unknown, this option allows selecting visualizing ",
      this, SLOT(updateState()));
  state_property_->addOption("Occupied", 0);
  state_property_->addOption("Free", 1);
  state_property_->addOption("Unknown", 2);
  state_property_->addOption("Bound", 3);

  bound_scale_property_ = new rviz::FloatProperty(
      "BoundScale", 0.1, "Line width of the bounding box.", this, SLOT(updateBoundScale()));

  mesh_height_property_ = new rviz::FloatProperty(
      "MeshHeight", 0.5, "Select height to visualize mesh.", this, SLOT(updateMeshHeight()));
  mesh_color_property_ =
    new rviz::ColorProperty("MeshColor", QColor(0, 170, 255), "Mesh color.",
        this, SLOT(updateMeshColorAndAlpha()));
  mesh_alpha_property_ = new rviz::FloatProperty("MeshAlpha", 0.2,
      "0 is fully transparent, 1.0 is fully opaque, only affect mesh", this,
      SLOT(updateMeshColorAndAlpha()));

  mesh_height_ = mesh_height_property_->getFloat();
  update_nh_.setCallbackQueue(point_cloud_common_->getCallbackQueue());
  map_util_.reset(new MPL::VoxelMapUtil());
}

MapDisplay::~MapDisplay() { delete point_cloud_common_; }

void MapDisplay::setMap(std::shared_ptr<MPL::VoxelMapUtil>& map_util, const planning_ros_msgs::VoxelMap& msg) {
  Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
  Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
  decimal_t res = msg.resolution;
  std::vector<signed char> map = msg.data;

  map_util->setMap(ori, dim, map, res);
}

void MapDisplay::getMap(std::shared_ptr<MPL::VoxelMapUtil>& map_util, planning_ros_msgs::VoxelMap& map) {
  Vec3f ori = map_util->getOrigin();
  Vec3i dim = map_util->getDim();
  decimal_t res = map_util->getRes();

  map.origin.x = ori(0);
  map.origin.y = ori(1);
  map.origin.z = ori(2);

  map.dim.x = dim(0);
  map.dim.y = dim(1);
  map.dim.z = dim(2);
  map.resolution = res;

  map.data = map_util->getMap();
}



void MapDisplay::onInitialize() {
  MFDClass::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);
}


BoundVec3f MapDisplay::getSpace() {
  Vec3i dim = map_util_->getDim();
  Vec3f ori = map_util_->getOrigin();
  float res = map_util_->getRes();

  float x = dim(0) * res;
  float y = dim(1) * res;
  float z = dim(2) * res;
  float ox = ori(0);
  float oy = ori(1);
  float oz = ori(2);
  vec_Vec3f cs;
  cs.resize(8);
  cs[0](0) = ox + x;
  cs[0](1) = oy;
  cs[0](2) = oz;

  cs[1](0) = ox + x;
  cs[1](1) = oy + y;
  cs[1](2) = oz;

  cs[2](0) = ox;
  cs[2](1) = oy + y;
  cs[2](2) = oz;

  cs[3](0) = ox;
  cs[3](1) = oy;
  cs[3](2) = oz;

  cs[4](0) = ox;
  cs[4](1) = oy;
  cs[4](2) = oz + z;

  cs[5](0) = ox;
  cs[5](1) = oy + y;
  cs[5](2) = oz + z;

  cs[6](0) = ox + x;
  cs[6](1) = oy + y;
  cs[6](2) = oz + z;

  cs[7](0) = ox + x;
  cs[7](1) = oy;
  cs[7](2) = oz + z;

  /*
   *    7------6
   *   /|     /|
   *  / |    / |
   * 8------5  |
   * |  2---|--3
   * | /    | /
   * |/     |/
   * 1------4
   *
   *
   */


  BoundVec3f bs(6);
  vec_Vec3f f1(4), f2(4), f3(4), f4(4), f5(4), f6(4);

  f1[0] = cs[0];
  f1[1] = cs[1];
  f1[2] = cs[2];
  f1[3] = cs[3];

  f2[0] = cs[4];
  f2[1] = cs[5];
  f2[2] = cs[6];
  f2[3] = cs[7];

  f3[0] = cs[0];
  f3[1] = cs[3];
  f3[2] = cs[4];
  f3[3] = cs[7];

  f4[0] = cs[1];
  f4[1] = cs[2];
  f4[2] = cs[5];
  f4[3] = cs[6];

  f5[0] = cs[0];
  f5[1] = cs[1];
  f5[2] = cs[6];
  f5[3] = cs[7];

  f6[0] = cs[2];
  f6[1] = cs[3];
  f6[2] = cs[4];
  f6[3] = cs[5];

  bs[0] = f1;
  bs[1] = f2;
  bs[2] = f3;
  bs[3] = f4;
  bs[4] = f5;
  bs[5] = f6;

  return bs;
}

void MapDisplay::processMessage(const planning_ros_msgs::VoxelMapConstPtr &msg) {
  setMap(map_util_, *msg);

 if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position_, orientation_)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  BoundVec3f bound = getSpace();
  std::shared_ptr<BoundVisual> visual;
    visual.reset(new BoundVisual(context_->getSceneManager(), scene_node_));

  // Now set or update the contents of the chosen visual.
  visual->setMessage(bound);
  visual->setFramePosition(position_);
  visual->setFrameOrientation(orientation_);

  visual->setColor(0.0, 0.0, 0, 1.0);
  float scale = bound_scale_property_->getFloat();
  visual->setScale(scale);

  visual_ = visual;
  header_ptr_ = std::make_shared<std_msgs::Header>(msg->header);

  int state = state_property_->getOptionInt();
  visualizeMessage(state);
}

void MapDisplay::visualizeMesh(const vec_Vec3f& pts, double res) {
  visuals_mesh_.clear();
  vec_E<vec_Vec3f> vss;
  for(const auto& pt: pts) {
    if(std::abs(pt(2) - mesh_height_) > res/2)
      continue;
    Vec3f pt1(pt(0)-res/2, pt(1)-res/2, pt(2));
    Vec3f pt2(pt(0)+res/2, pt(1)-res/2, pt(2));
    Vec3f pt3(pt(0)+res/2, pt(1)+res/2, pt(2));
    Vec3f pt4(pt(0)-res/2, pt(1)+res/2, pt(2));
    vec_Vec3f vertices;
    vertices.push_back(pt1);
    vertices.push_back(pt2);
    vertices.push_back(pt3);
    vertices.push_back(pt4);
    vss.push_back(vertices);
  }
  std::shared_ptr<MeshVisual> visual_mesh;
  visual_mesh.reset(new MeshVisual(context_->getSceneManager(), scene_node_));

  visual_mesh->setMessage(vss);
  visual_mesh->setFramePosition(position_);
  visual_mesh->setFrameOrientation(orientation_);

  float alpha = mesh_alpha_property_->getFloat();
  Ogre::ColourValue color = mesh_color_property_->getOgreColor();
  visual_mesh->setColor(color.r, color.g, color.b, alpha);
  visuals_mesh_.push_back(visual_mesh);
}

void MapDisplay::visualizeMessage(int state) {
  if(header_ptr_ == nullptr)
    return;

  switch (state) {
    case 0: {
              visuals_mesh_.clear();
              sensor_msgs::PointCloud cloud = vec_to_cloud(map_util_->getCloud());
              cloud.header = *header_ptr_;
              point_cloud_common_->addMessage(
                  boost::make_shared<sensor_msgs::PointCloud>(cloud));
              break;
            }
    case 1: {
              sensor_msgs::PointCloud cloud = vec_to_cloud(map_util_->getCloud());
              cloud.header = *header_ptr_;
              point_cloud_common_->addMessage(
                  boost::make_shared<sensor_msgs::PointCloud>(cloud));
 
              vec_Vec3f free_cloud = map_util_->getFreeCloud();
              visualizeMesh(free_cloud, map_util_->getRes());
              break;
            }
    case 2: {
              sensor_msgs::PointCloud cloud = vec_to_cloud(map_util_->getCloud());
              cloud.header = *header_ptr_;
              point_cloud_common_->addMessage(
                  boost::make_shared<sensor_msgs::PointCloud>(cloud));
 
              vec_Vec3f unknown_cloud = map_util_->getUnknownCloud();
              visualizeMesh(unknown_cloud, map_util_->getRes());
              break;
            }
    case 3: {
              point_cloud_common_->reset();
              visuals_mesh_.clear();
              break;
            }
    default:
            std::cout << "Invalid State: " << state << std::endl;
  }
}

void MapDisplay::updateState() {
  int state = state_property_->getOptionInt();
  visualizeMessage(state);
}

void MapDisplay::update(float wall_dt, float ros_dt) {
  point_cloud_common_->update(wall_dt, ros_dt);
}

void MapDisplay::updateBoundScale() {
  float scale = bound_scale_property_->getFloat();
  if (visual_)
    visual_->setScale(scale);
}

void MapDisplay::updateMeshHeight() {
  mesh_height_ = mesh_height_property_->getFloat();
  int state = state_property_->getOptionInt();
  if(visual_)
    visualizeMessage(state);
}

void MapDisplay::updateMeshColorAndAlpha() {
  float alpha = mesh_alpha_property_->getFloat();
  Ogre::ColourValue color = mesh_color_property_->getOgreColor();

  if(visual_) {
    for(auto& it: visuals_mesh_)
      it->setColor(color.r, color.g, color.b, alpha);
  }
}


void MapDisplay::reset() {
  MFDClass::reset();
  point_cloud_common_->reset();
  visual_ = nullptr;
  visuals_mesh_.clear();
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning_rviz_plugins::MapDisplay, rviz::Display)
