#include "bag_reader.hpp"
#include "txt_reader.hpp"
#include <decomp_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <decomp_util/seed_decomp.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/point_cloud_conversion.h>

std_msgs::Header header_;

int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
  ros::Publisher seed_pub = nh.advertise<sensor_msgs::PointCloud>("seed", 1, true);
  ros::Publisher es_pub = nh.advertise<decomp_ros_msgs::Ellipsoids>("ellipsoids", 1, true);
  ros::Publisher poly_pub = nh.advertise<decomp_ros_msgs::Polyhedra>("polyhedra", 1, true);

  header_.frame_id = std::string("map");
  std::string file_name, topic_name, marker_name, seed_file;

  nh.param("seed_file", seed_file, std::string("seed.txt"));
  nh.param("bag_file", file_name, std::string("voxel_map"));
  nh.param("bag_topic", topic_name, std::string("voxel_map"));
  nh.param("bag_marker", marker_name, std::string("voxel_map"));
  //Read the point cloud from bag
  sensor_msgs::PointCloud2 cloud2 = read_bag<sensor_msgs::PointCloud2>(file_name, topic_name);
  //Convert into vector of Eigen
  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud);
  cloud.header = header_;
  map_pub.publish(cloud);

  vec_Vec3f obs = DecompROS::cloud_to_vec(cloud);

  visualization_msgs::MarkerArray markers = read_bag<visualization_msgs::MarkerArray>(file_name, marker_name);
  for(auto & it: markers.markers)
    it.header = header_;
  marker_pub.publish(markers);

  //Read path from txt
  vec_Vec3f seeds;
  if(!read_path(seed_file, seeds))
    ROS_ERROR("Fail to read seeds!");

  sensor_msgs::PointCloud seed_msg = DecompROS::vec_to_cloud(seeds);
  seed_msg.header = header_;
  seed_pub.publish(seed_msg);

  vec_Ellipsoid es;
  Polyhedra polys;

  for(const auto& it: seeds) {
    //Using seed decomposition
    SeedDecomp decomp_util(it);
    decomp_util.set_obstacles(obs);
    decomp_util.dilate(5.0);
    /*
    Aff3f Rf = Trans3f(0, 0, 0) *
      Anglef( 0 * M_PI / 180, Vec3f::UnitX()) *
      Anglef( 10 * M_PI / 180, Vec3f::UnitY()) *
      Anglef(M_PI/3, Vec3f::UnitZ());

    decomp_util.dilate(Vec3f(0.5, 0.6, 0.1), Rf.rotation());
    */
    es.push_back(decomp_util.ellipsoid());
    polys.push_back(decomp_util.polyhedron());
  }

  //Publish visualization msgs
  decomp_ros_msgs::Ellipsoids es_msg = DecompROS::ellipsoids_to_ros(es);
  es_msg.header = header_;
  es_pub.publish(es_msg);

  decomp_ros_msgs::Polyhedra poly_msg = DecompROS::polyhedra_to_ros(polys);
  poly_msg.header = header_;
  poly_pub.publish(poly_msg);


  /*
  vec_LinearConstraint3f cs = decomp_util.get_constraints();
  for(int i = 0; i < cs.size(); i++) {
    MatD3f A = cs[i].first;
    VecDf b = cs[i].second;

    printf("i: %d\n", i);
    std::cout << "start: " << (A*path[i]-b).transpose() << std::endl;
    std::cout << "end: " << (A*path[i+1]-b).transpose() << std::endl;
  }
  */


  ros::spin();

  return 0;
}
