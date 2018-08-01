#include "trajectory_visual.h"

namespace planning_rviz_plugins {

TrajectoryVisual::TrajectoryVisual(Ogre::SceneManager *scene_manager,
                       Ogre::SceneNode *parent_node) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
}

TrajectoryVisual::~TrajectoryVisual() { scene_manager_->destroySceneNode(frame_node_); }

void TrajectoryVisual::setMessage(const planning_ros_msgs::Trajectory &msg) {
  poss_.clear();
  vels_.clear();
  accs_.clear();
  jrks_.clear();

  if (msg.primitives.empty() || num_ == 0)
    return;

  poss_.resize(num_);
  if(vel_vis_)
    vels_.resize(num_);
  if(acc_vis_)
    accs_.resize(num_);
  if(jrk_vis_)
    jrks_.resize(num_);
 
  for (int i = 0; i < num_; i++) {
    poss_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
    if(vel_vis_)
      vels_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
    if(acc_vis_)
      accs_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
    if(jrk_vis_)
      jrks_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
  }

  Trajectory3 p = toTrajectory3(msg);
  decimal_t theta = M_PI / 2;
  Mat3f R;
  R << cos(theta), -sin(theta), 0,
    sin(theta), cos(theta), 0,
    0, 0, 1;
  vec_E<Waypoint3> waypoints = p.sample(num_);
  for (int i = 1; i < (int) waypoints.size(); i++) {
    Waypoint3 p1 = waypoints[i-1];
    Waypoint3 p2 = waypoints[i];

    Ogre::Vector3 pos1(p1.pos(0), p1.pos(1), p1.pos(2));
    Ogre::Vector3 pos2(p2.pos(0), p2.pos(1), p2.pos(2));
    poss_[i-1]->addPoint(pos1);
    poss_[i-1]->addPoint(pos2);

    if(vel_vis_){
      Vec3f p3 = p2.pos + R * p2.vel;
      Ogre::Vector3 pos3(p3(0), p3(1), p3(2));
      vels_[i-1]->addPoint(pos2);
      vels_[i-1]->addPoint(pos3);
    }

    if(acc_vis_){
      Vec3f p4 = p2.pos + R * p2.acc;
      Ogre::Vector3 pos4(p4(0), p4(1), p4(2));
      accs_[i-1]->addPoint(pos2);
      accs_[i-1]->addPoint(pos4);
    }

    if(jrk_vis_){
      Vec3f p4 = p2.pos + R * p2.jrk / 10;
      Ogre::Vector3 pos4(p4(0), p4(1), p4(2));
      jrks_[i-1]->addPoint(pos2);
      jrks_[i-1]->addPoint(pos4);
    }

  }
}


void TrajectoryVisual::addMessage(const planning_ros_msgs::Trajectory &msg) {
  int prev_size = poss_.size();
  poss_.resize(num_ + prev_size);
  if(vel_vis_)
    vels_.resize(num_ + prev_size);
  if(acc_vis_)
    accs_.resize(num_ + prev_size);
 if(jrk_vis_)
    jrks_.resize(num_ + prev_size);
  
  for (int i = prev_size; i < (int)poss_.size(); i++) {
    poss_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
    if(vel_vis_)
      vels_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
    if(acc_vis_)
      accs_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
    if(jrk_vis_)
      jrks_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
  }


  Trajectory3 p = toTrajectory3(msg);
  decimal_t theta = M_PI / 2;
  Mat3f R;
  R << cos(theta), -sin(theta), 0,
    sin(theta), cos(theta), 0,
    0, 0, 1;

  vec_E<Waypoint3> waypoints = p.sample(num_);
  for (int i = 1; i < (int) waypoints.size(); i++) {
    Waypoint3 p1 = waypoints[i-1];
    Waypoint3 p2 = waypoints[i];

    Ogre::Vector3 pos1(p1.pos(0), p1.pos(1), p1.pos(2));
    Ogre::Vector3 pos2(p2.pos(0), p2.pos(1), p2.pos(2));
    poss_[i-1 + prev_size]->addPoint(pos1);
    poss_[i-1 + prev_size]->addPoint(pos2);

    if(vel_vis_){
      Vec3f p3 = p2.pos + R * p2.vel;
      Ogre::Vector3 pos3(p3(0), p3(1), p3(2));
      vels_[i-1 + prev_size]->addPoint(pos2);
      vels_[i-1 + prev_size]->addPoint(pos3);
    }

    if(acc_vis_){
      Vec3f p4 = p2.pos + R * p2.acc;
      Ogre::Vector3 pos4(p4(0), p4(1), p4(2));
      accs_[i-1 + prev_size]->addPoint(pos2);
      accs_[i-1 + prev_size]->addPoint(pos4);
    }

    if(jrk_vis_){
      Vec3f p4 = p2.pos + R * p2.jrk / 10;
      Ogre::Vector3 pos4(p4(0), p4(1), p4(2));
      jrks_[i-1]->addPoint(pos2);
      jrks_[i-1]->addPoint(pos4);
    }
  }
}

void TrajectoryVisual::setNum(int n) {
  num_ = n;
}

void TrajectoryVisual::setVelVis(bool vis) {
  vel_vis_ = vis;
}

void TrajectoryVisual::setAccVis(bool vis) {
  acc_vis_ = vis;
}

void TrajectoryVisual::setJrkVis(bool vis) {
  jrk_vis_ = vis;
}

void TrajectoryVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void TrajectoryVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

void TrajectoryVisual::setPosColor(float r, float g, float b, float a) {
  for (auto &it : poss_)
    it->setColor(r, g, b, a);
}

void TrajectoryVisual::setVelColor(float r, float g, float b, float a) {
  for (auto &it : vels_)
    it->setColor(r, g, b, a);
}

void TrajectoryVisual::setAccColor(float r, float g, float b, float a) {
  for (auto &it : accs_)
    it->setColor(r, g, b, a);
}

void TrajectoryVisual::setJrkColor(float r, float g, float b, float a) {
  for (auto &it : jrks_)
    it->setColor(r, g, b, a);
}

void TrajectoryVisual::setPosScale(float s) {
  for (auto &it : poss_)
    it->setLineWidth(s);
}

void TrajectoryVisual::setVelScale(float s) {
  for (auto &it : vels_)
    it->setLineWidth(s);
}

void TrajectoryVisual::setAccScale(float s) {
  for (auto &it : accs_)
    it->setLineWidth(s);
}

void TrajectoryVisual::setJrkScale(float s) {
  for (auto &it : jrks_)
    it->setLineWidth(s);
}

}
