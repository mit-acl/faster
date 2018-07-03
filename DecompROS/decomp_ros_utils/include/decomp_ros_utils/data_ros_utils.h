#ifndef DECOMP_ROS_UTILS_H
#define DECOMP_ROS_UTILS_H

#include <decomp_util/data_type.h>
#include <sensor_msgs/PointCloud.h>
#include <decomp_ros_msgs/Polyhedra.h>
#include <decomp_ros_msgs/Ellipsoids.h>
#include <nav_msgs/Path.h>

namespace DecompROS {
inline vec_Vec3f path_to_eigen(const nav_msgs::Path &path) {
  vec_Vec3f vs;
  for (auto it : path.poses) {
    Vec3f v(it.pose.position.x, it.pose.position.y, it.pose.position.z);

    vs.push_back(v);
  }

  return vs;
}

inline nav_msgs::Path eigen_to_path(const vec_Vec3f &vs) {
  nav_msgs::Path path;
  for (auto it : vs) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = it(0);
    pose.pose.position.y = it(1);
    pose.pose.position.z = it(2);
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    path.poses.push_back(pose);
  }

  return path;
}


inline sensor_msgs::PointCloud vec_to_cloud(const vec_Vec3f &pts) {
  sensor_msgs::PointCloud cloud;
  cloud.points.resize(pts.size());

  for (unsigned int i = 0; i < pts.size(); i++) {
    cloud.points[i].x = pts[i](0);
    cloud.points[i].y = pts[i](1);
    cloud.points[i].z = pts[i](2);
  }
  return cloud;
}

inline vec_Vec3f cloud_to_vec(const sensor_msgs::PointCloud &cloud) {
  vec_Vec3f pts;
  pts.resize(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    pts[i](0) = cloud.points[i].x;
    pts[i](1) = cloud.points[i].y;
    pts[i](2) = cloud.points[i].z;
  }

  return pts;
}

inline Polyhedra ros_to_polyhedra(const decomp_ros_msgs::Polyhedra& msg){
  Polyhedra polys;
  for(const auto& polyhedron: msg.polyhedra){
    Polyhedron p;
    for(unsigned int i = 0; i < polyhedron.points.size(); i++){
      Vec3f pt(polyhedron.points[i].x,
               polyhedron.points[i].y,
               polyhedron.points[i].z);
      Vec3f n(polyhedron.normals[i].x,
              polyhedron.normals[i].y,
              polyhedron.normals[i].z);
      if(polyhedron.passes.empty())
        p.push_back(Face(pt, n));
      else
        p.push_back(Face(pt, n, polyhedron.passes[i]));
    }
    polys.push_back(p);
  }
  return polys;
}

inline decomp_ros_msgs::Polyhedra polyhedra_to_ros(const Polyhedra& vs){
  decomp_ros_msgs::Polyhedra poly;
  for (const auto &v : vs) {
    decomp_ros_msgs::Polyhedron f;
    for (const auto &p : v) {
      geometry_msgs::Point pt, n;
      pt.x = p.p(0);
      pt.y = p.p(1);
      pt.z = p.p(2);
      n.x = p.n(0);
      n.y = p.n(1);
      n.z = p.n(2);
      f.points.push_back(pt);
      f.normals.push_back(n);
      f.passes.push_back(p.pass);
    }
    poly.polyhedra.push_back(f);
  }

  return poly;
}

inline decomp_ros_msgs::Ellipsoids ellipsoids_to_ros(const vec_Ellipsoid& Es) {
  decomp_ros_msgs::Ellipsoids ellipsoids;
  for (unsigned int i = 0; i < Es.size(); i++) {
    decomp_ros_msgs::Ellipsoid ellipsoid;
    ellipsoid.d[0] = Es[i].second(0);
    ellipsoid.d[1] = Es[i].second(1);
    ellipsoid.d[2] = Es[i].second(2);

    for (int x = 0; x < 3; x++)
      for (int y = 0; y < 3; y++)
        ellipsoid.E[3 * x + y] = Es[i].first(x, y);
    ellipsoids.ellipsoids.push_back(ellipsoid);
  }

  return ellipsoids;
}
}

#endif
