#include "tip.hpp"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
// #include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/median_filter.h>


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "reactive_planner", ros::init_options::NoSigintHandler);

	ros::NodeHandle n("~");

	// TIP tip;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Generate pointcloud data
	cloud->width = 480./4;
	cloud->height = 360./4;
	cloud->points.resize (cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
	}

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree2;

	kdtree.setInputCloud(cloud);
	kdtree2.setInputCloud(cloud);

	pcl::PointXYZ searchPoint;

	searchPoint.x = 0;
	searchPoint.y = 0;
	searchPoint.z = 0;

	std::vector<int> pointIdxNKNSearch(1);
  	std::vector<float> pointNKNSquaredDistance(1);

  	kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
  	std::cout << "distance: " << pointNKNSquaredDistance[0] << std::endl;
	
	std::vector<pcl::KdTreeFLANN<pcl::PointXYZ>> vec_kdtree = {kdtree, kdtree2};
  	vec_kdtree[0].nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
  	std::cout << "vec distance: " << pointNKNSquaredDistance[0] << std::endl;


	// Eigen::Matrix4d X0 = Eigen::Matrix4d::Zero();

	// std::vector<double> t_x{std::vector<double>(3,0)};
	// std::vector<double> x0{std::vector<double>(4,0)};
	// std::vector<double> vx0{std::vector<double>(4,0)};
	// std::vector<double> ax0{std::vector<double>(4,0)};
	// std::vector<double> jx{std::vector<double>(4,0)};

	// Eigen::Vector4d x = Eigen::Vector4d::Zero();

	// Eigen::Vector3d pose = Eigen::Vector3d::Zero();

	// double j = 30;
	// double vf = 1;
	// double t0 = 0;

	// // Test if we're in decel phase
	// // x[1] = vf/2;
	// // x[2] = 5.477;

	// Eigen::ArrayXd times_;
	// times_ = Eigen::ArrayXd::Zero(100000);
	// double then;

	// for (int i=0; i<100000; i++){
	// 	then = ros::WallTime::now().toSec();
	// 	tip.find_times(x,vf,t_x,X0,false);
	// 	tip.find_times(x,vf,t_x,X0,false);
	// 	tip.find_times(x,vf,t_x,X0,false);
	// 	times_(i) = ros::WallTime::now().toSec()-then;
	// }

	// double mu = times_.mean();
	// double sig = pow(((times_-mu)*(times_-mu)).mean(),0.5);

	// std::cout << "Trajectory gen time [ms]: " << 1000*times_.mean() << std::endl;
	// std::cout << "STD trajectory gen time [ms]: " << 1000*sig << std::endl;

	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// // Generate pointcloud data
	// cloud->width = 480./4;
	// cloud->height = 360./4;
	// cloud->points.resize (cloud->width * cloud->height);

	// for (size_t i = 0; i < cloud->points.size (); ++i)
	// {
	// 	cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
	// 	cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
	// 	cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
	// }


	// times_ = Eigen::ArrayXd::Zero(1);
	// for (int i=0; i<1; i++){
	// 	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	// 	then = ros::WallTime::now().toSec();
	// 	kdtree.setInputCloud (cloud);
	// 	times_(i) = ros::WallTime::now().toSec()-then;
	// }

	// pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	// kdtree.setInputCloud (cloud);


	// mu = times_.mean();
	// sig = pow(((times_-mu)*(times_-mu)).mean(),0.5);

	// std::cout << "pcl k-d tree gen time [ms]: " << 1000*times_.mean() << std::endl;
	// std::cout << "pcl STD k-d treee gen time [ms]: " << 1000*sig << std::endl;

	
	// pcl::PointXYZ searchPoint;

	// searchPoint.x = 0;
	// searchPoint.y = 0;
	// searchPoint.z = 0;

 //  	int K = 100;	

	// std::vector<int> pointIdxNKNSearch(K);
 //  	std::vector<float> pointNKNSquaredDistance(K);

 //  	times_ = Eigen::ArrayXd::Zero(10000);
	// for (int i=0; i<10000; i++){
	// 	then = ros::WallTime::now().toSec();
	// 	kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
	// 	times_(i) = ros::WallTime::now().toSec()-then;
	// }

	// mu = times_.mean();
	// sig = pow(((times_-mu)*(times_-mu)).mean(),0.5);

	// std::cout << "pcl k-d tree point queury time [ms]: " << 1000*times_.mean() << std::endl;
	// std::cout << "pcl STD k-d tree point queury time [ms]: " << 1000*sig << std::endl;

	// float radius = 3;
	
	// std::vector<int> pointIdxRadiusSearch;
	// std::vector<float> pointRadiusSquaredDistance;

	// times_ = Eigen::ArrayXd::Zero(10000);
	// for (int i=0; i<10000; i++){
	// 	then = ros::WallTime::now().toSec();
	// 	kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	// 	times_(i) = ros::WallTime::now().toSec()-then;
	// }

	// mu = times_.mean();
	// sig = pow(((times_-mu)*(times_-mu)).mean(),0.5);

	// std::cout << "pcl k-d tree radius queury time [ms]: " << 1000*times_.mean() << std::endl;
	// std::cout << "pcl STD k-d tree radius queury time [ms]: " << 1000*sig << std::endl;

	// kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

	
	// // Fill in the cloud data
	// cloud->width = 480./4;
	// cloud->height = 360./4;
	// cloud->points.resize (cloud->width * cloud->height);

	// for (size_t i = 0; i < cloud->points.size (); ++i)
	// {
	// 	cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
	// 	cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
	// 	cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
	// }
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// times_ = Eigen::ArrayXd::Zero(100);
	// for (int i=0; i<100; i++){
	//     // build the filter
	// 	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	// 	sor.setInputCloud (cloud);
	// 	sor.setMeanK (10);
	// 	sor.setStddevMulThresh (1.0);
	// 	sor.filter (*cloud_filtered);
	// 	times_(i) = ros::WallTime::now().toSec()-then;
	// }

	// mu = times_.mean();
	// sig = pow(((times_-mu)*(times_-mu)).mean(),0.5);

	// std::cout << "pcl filter time [ms]: " << 1000*times_.mean() << std::endl;
	// std::cout << "pcl STD filter time [ms]: " << 1000*sig << std::endl;


	// if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	// {
	// for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
	// 	std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
	//         << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
	//         << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
	//         << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	// }

	// KDTree<double> kd_tree;

	// for (int i=0; i<100000; i++){
	// 	then = ros::WallTime::now().toSec();
	// 	kd_tree.Initialize(cloud);
	// 	times_(i) = ros::WallTime::now().toSec()-then;
	// }

	// mu = times_.mean();
	// sig = pow(((times_-mu)*(times_-mu)).mean(),0.5);

	// std::cout << "nano k-d tree gen time [ms]: " << 1000*times_.mean() << std::endl;
	// std::cout << "nano STD k-d treee gen time [ms]: " << 1000*sig << std::endl;




	return 0;
}