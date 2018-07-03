#include "mesh_visual.h"

namespace planning_rviz_plugins {
  MeshVisual::MeshVisual(Ogre::SceneManager *scene_manager,
                         Ogre::SceneNode *parent_node) {
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();
    obj_.reset(new rviz::MeshShape(scene_manager_, frame_node_));
  }

  MeshVisual::~MeshVisual() { scene_manager_->destroySceneNode(frame_node_); }

  void MeshVisual::setMessage(const vec_E<vec_Vec3f> &vss) {
    if(vss.empty())
      return;
    obj_->clear();

    int cnt = 0;
    obj_->beginTriangles();
    for(const auto& vs: vss) {
      if (vs.size() < 3)
        continue;

      const Vec3f p0(vs[0](0), vs[0](1), vs[0](2));
      const Vec3f p1(vs[1](0), vs[1](1), vs[1](2));
      const Vec3f p2(vs[2](0), vs[2](1), vs[2](2));
      Vec3f n = -(p2-p0).cross(p1-p0);
      n = n.normalized();
      if(std::isnan(n(0)))
        n = Vec3f(0, 0, -1);

      int ref_cnt = cnt;
      const Ogre::Vector3 normal(n(0), n(1), n(2));
      for (unsigned int i = 0; i < vs.size(); i++) {
        obj_->addVertex(Ogre::Vector3(vs[i](0), vs[i](1), vs[i](2)), normal);
        if (i > 1 && i < vs.size())
          obj_->addTriangle(ref_cnt, cnt - 1, cnt);
        cnt ++;
      }
    }

    obj_->endTriangles();
  }

  // Position and orientation are passed through to the SceneNode.
  void MeshVisual::setFramePosition(const Ogre::Vector3 &position) {
    frame_node_->setPosition(position);
  }

  void MeshVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
    frame_node_->setOrientation(orientation);
  }

  void MeshVisual::setColor(float r, float g, float b, float a) {
    obj_->setColor(r, g, b, a);
  }
}
