#include "path_visual.h"

namespace planning_rviz_plugins {

PathVisual::PathVisual(Ogre::SceneManager *scene_manager,
                       Ogre::SceneNode *parent_node) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
}

PathVisual::~PathVisual() { scene_manager_->destroySceneNode(frame_node_); }

void PathVisual::setMessage(const vec_Vec3f &path) {
  nodes_.clear();
  lines_.clear();

  if (path.empty())
    return;

  for (const auto& it: path) {
    if (std::isnan(it(0)) ||
        std::isnan(it(1)) ||
        std::isnan(it(2)))
      return;
  }

  nodes_.resize(path.size());
  lines_.resize(path.size() - 1);

  for (auto &it : nodes_)
    it.reset(new rviz::Shape(rviz::Shape::Type::Sphere, scene_manager_,
                             frame_node_));
  for (auto &it : lines_)
    it.reset(new rviz::BillboardLine(scene_manager_, frame_node_));

  for (int i = 0; i < (int) path.size(); i++) {
    Ogre::Vector3 pos(path[i](0), path[i](1), path[i](2));
    if(i < (int) path.size() - 1) {
      Ogre::Vector3 pos2(path[i+1](0), path[i+1](1), path[i+1](2));
      lines_[i]->addPoint(pos);
      lines_[i]->addPoint(pos2);
    }
    nodes_[i]->setPosition(pos);
  }
}

void PathVisual::addMessage(const vec_Vec3f &path) {
  if (path.empty())
    return;

  unsigned int nodes_prev_size = nodes_.size();
  nodes_.resize(nodes_prev_size + path.size());

  for (unsigned int i = nodes_prev_size; i < nodes_.size(); i++)
    nodes_[i].reset(new rviz::Shape(rviz::Shape::Type::Sphere, scene_manager_,
                             frame_node_));

  unsigned int lines_prev_size = lines_.size();
  lines_.resize(lines_prev_size + path.size() - 1);

  for (unsigned int i = lines_prev_size; i < lines_prev_size + path.size() - 1; i++) {
    lines_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
  }

  for (int i = 0; i < (int) path.size(); i++) {
    Ogre::Vector3 pos(path[i](0), path[i](1), path[i](2));
    if(i < (int) path.size() - 1) {
      Ogre::Vector3 pos2(path[i+1](0), path[i+1](1), path[i+1](2));
      lines_[i + lines_prev_size]->addPoint(pos);
      lines_[i + lines_prev_size]->addPoint(pos2);
    }
    nodes_[i + nodes_prev_size]->setPosition(pos);
  }
}

void PathVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void PathVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

void PathVisual::setLineColor(float r, float g, float b, float a) {
  for(auto& it: lines_)
    it->setColor(r, g, b, a);
}

void PathVisual::setNodeColor(float r, float g, float b, float a) {
  for (auto &it : nodes_)
    it->setColor(r, g, b, a);
}

void PathVisual::setLineScale(float s) {
  for (auto &it : lines_)
    it->setLineWidth(s);
}

void PathVisual::setNodeScale(float s) {
  for (auto &it : nodes_)
    it->setScale(Ogre::Vector3(s, s, s));
}
}
