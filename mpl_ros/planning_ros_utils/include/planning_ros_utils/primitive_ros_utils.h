/**
 * @file primitive_ros_utils.h
 * @brief Interface between primitive classes and ROS
 */
#pragma once
#include <planning_ros_msgs/Trajectory.h>
#include <planning_ros_msgs/Primitives.h>
#include <motion_primitive_library/primitive/trajectory.h>

///Primitive2 to primitive ROS message
planning_ros_msgs::Primitive toPrimitiveROSMsg(const Primitive2& pr, double z = 0);
///Primitive3 to primitive ROS message
planning_ros_msgs::Primitive toPrimitiveROSMsg(const Primitive3& pr);
///Multiple Primitive2 to Primitive ROS message
planning_ros_msgs::Primitives toPrimitivesROSMsg(const vec_E<Primitive2>& prs, double z = 0);
///Multiple Primitive3 to Primitive ROS message
planning_ros_msgs::Primitives toPrimitivesROSMsg(const vec_E<Primitive3>& prs);
///Trajectory2 class to trajectory ROS message
planning_ros_msgs::Trajectory toTrajectoryROSMsg(const Trajectory2& traj, double z = 0);
///Trajectory3 class to trajectory ROS message
planning_ros_msgs::Trajectory toTrajectoryROSMsg(const Trajectory3& traj);
///ROS message to Primitive2 class
Primitive2 toPrimitive2(const planning_ros_msgs::Primitive& pr_msg);
///ROS message to Primitive3 class
Primitive3 toPrimitive3(const planning_ros_msgs::Primitive& pr_msg);
///ROS message to Trajectory2 class 
Trajectory2 toTrajectory2(const planning_ros_msgs::Trajectory & traj_msg);
///ROS message to Trajectory3 class 
Trajectory3 toTrajectory3(const planning_ros_msgs::Trajectory & traj_msg);

