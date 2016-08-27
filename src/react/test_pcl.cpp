#include "rp.hpp"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <nanoflann.hpp>
#include <pcl/point_cloud.h>
#include "kd_tree.h"


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "reactive_planner", ros::init_options::NoSigintHandler);

	ros::NodeHandle n("~");

	REACT rp;

	Eigen::Matrix4d X0 = Eigen::Matrix4d::Zero();


	double a = 10;
	std::cout << "Original val: " << a << std::endl;
	rp.saturate(a,-1,5);
	std::cout << "Saturate val: " << a << std::endl << std::endl;

	std::vector<double> t_x{std::vector<double>(3,0)};
	std::vector<double> x0{std::vector<double>(4,0)};
	std::vector<double> vx0{std::vector<double>(4,0)};
	std::vector<double> ax0{std::vector<double>(4,0)};
	std::vector<double> jx{std::vector<double>(4,0)};

	Eigen::Vector3d x = Eigen::Vector3d::Zero();

	Eigen::Vector3d pose = Eigen::Vector3d::Zero();

	double j = 40;
	double vf = 1;
	double t0 = 0;

	// Test if we're in decel phase
	// x[1] = vf/2;
	// x[2] = 5.477;

	double diff;
	double  then;

	Eigen::ArrayXd times_;
	times_ = Eigen::ArrayXd::Zero(100000);

	for (int i=0; i<100000; i++){
		then = ros::WallTime::now().toSec();
		rp.find_times(x,vf,t_x,X0,false);
		rp.find_times(x,vf,t_x,X0,false);
		times_(i) = ros::WallTime::now().toSec()-then;
	}

	double mu = times_.mean();
	double sig = pow(((times_-mu)*(times_-mu)).mean(),0.5);

	std::cout << "Trajectory gen time [ms]: " << 1000*times_.mean() << std::endl;
	std::cout << "STD trajectory gen time [ms]: " << 1000*sig << std::endl;


	return 0;
}