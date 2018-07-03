#ifndef DATA_ROS_UTILS_H
#define DATA_ROS_UTILS_H

#include <motion_primitive_library/common/data_type.h>
#include <sensor_msgs/PointCloud.h>
#include <planning_ros_msgs/Path.h>
#include <planning_ros_msgs/PathArray.h>
#include <planning_ros_msgs/Arrows.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Twist.h>

inline Vec3f pose_to_eigen(const geometry_msgs::Pose &pose) {
  return Vec3f(pose.position.x, pose.position.y, pose.position.z);
}

inline Vec3f twist_to_eigen(const geometry_msgs::Twist &twist) {
  return Vec3f(twist.linear.x, twist.linear.y, twist.linear.z);
}

inline Vec3f vec_to_eigen(const geometry_msgs::Vector3 &v) {
  return Vec3f(v.x, v.y, v.z);
}

inline geometry_msgs::Pose eigen_to_pose(const Vec3f& pose) {
  geometry_msgs::Pose p;
  p.position.x = pose(0);
  p.position.y = pose(1);
  p.position.z = pose(2);
  p.orientation.w = 1.0;
  return p;
}


inline geometry_msgs::Twist eigen_to_twist(const Vec3f& twist) {
  geometry_msgs::Twist t;
  t.linear.x = twist(0);
  t.linear.y = twist(1);
  t.linear.z = twist(2);
  return t;
}


inline sensor_msgs::PointCloud transform_cloud(
    const sensor_msgs::PointCloud &cloud, const Aff3f &TF) {
  sensor_msgs::PointCloud new_cloud = cloud;
  int i = 0;
  for (const auto& it : cloud.points) {
    Vec3f raw(it.x, it.y, it.z);
    raw = TF * raw;
    new_cloud.points[i].x = raw(0);
    new_cloud.points[i].y = raw(1);
    new_cloud.points[i].z = raw(2);
    i++;
  }
  return new_cloud;
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

inline Aff3f toTF(const geometry_msgs::Pose &p)
{
  tf::Pose Ttf;
  tf::poseMsgToTF(p, Ttf);
  Eigen::Affine3d Td;
  tf::poseTFToEigen(Ttf, Td);
  return Td.cast<decimal_t>();
}

inline planning_ros_msgs::Path path_to_ros(const vec_Vec3f& path) {
  planning_ros_msgs::Path msg;
  for (const auto &itt : path) {
    geometry_msgs::Point pt;
    pt.x = itt(0);
    pt.y = itt(1);
    pt.z = itt(2);
    msg.waypoints.push_back(pt);
  }
  return msg;
}

inline vec_Vec3f ros_to_path(const planning_ros_msgs::Path& msg) {
  vec_Vec3f path;
  for (const auto &it : msg.waypoints)
    path.push_back(Vec3f(it.x, it.y, it.z));
  return path;
}

inline planning_ros_msgs::PathArray path_array_to_ros(const vec_E<vec_Vec3f>& paths) {
  planning_ros_msgs::PathArray msg;
  for(const auto& it: paths) {
    planning_ros_msgs::Path path_msg;
    for (const auto &itt : it) {
      geometry_msgs::Point pt;
      pt.x = itt(0);
      pt.y = itt(1);
      pt.z = itt(2);
      path_msg.waypoints.push_back(pt);
    }
    msg.paths.push_back(path_msg);
  }
  return msg;
}


inline planning_ros_msgs::PathArray path_array_to_ros(const std::vector<std::pair<std::string, vec_Vec3f>>& paths) {
  planning_ros_msgs::PathArray msg;
  for(const auto& it: paths) {
    planning_ros_msgs::Path path_msg;
    path_msg.name = it.first;
    for (const auto &itt : it.second) {
      geometry_msgs::Point pt;
      pt.x = itt(0);
      pt.y = itt(1);
      pt.z = itt(2);
      path_msg.waypoints.push_back(pt);
    }
    msg.paths.push_back(path_msg);
  }
  return msg;
}

inline planning_ros_msgs::Arrows pairs_to_arrows(const vec_E<std::pair<Vec3f, Vec3f>>& vs) {
  planning_ros_msgs::Arrows msg;

  for(const auto& v: vs) {
    geometry_msgs::Point pt;
    pt.x = v.first(0);
    pt.y = v.first(1);
    pt.z = v.first(2);
    geometry_msgs::Point dir;
    dir.x = v.second(0);
    dir.y = v.second(1);
    dir.z = v.second(2);
    msg.points.push_back(pt);
    msg.directions.push_back(dir);
  }

  return msg;
}

#endif
