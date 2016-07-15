#include "rp.hpp"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "reactive_planner", ros::init_options::NoSigintHandler);

	ros::NodeHandle n("~");

	REACT rp;

	Eigen::Matrix4d X0 = Eigen::Matrix4d::Zero();

	// std::vector<float> v {std::vector<float>(3,0)};
	// std::cout << Eigen::Map<Eigen::MatrixXd, 0, Eigen::InnerStride<1>>(v) << std::endl;

	// m(10,10) = 0;

	// std::cout << m.rows() << " " << m.cols() << std::endl;	


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

	double j = 30;
	double vf = 1;
	double t0 = 0;

	// Test if we're in decel phase
	// x[1] = vf/2;
	// x[2] = 5.477;

	time_t now;
	time_t  then;
	double diff;
	then = clock();
	for (int i=0; i<500; i++){
		rp.find_times(t_x,X0,x,vf);
	}
	now = clock();

	diff = 1000*((float)(now-then))/CLOCKS_PER_SEC/500;


	std::cout << "Trajectory gen time [ms]: " << diff << std::endl << std::endl;

	// std::cout << X0 << std::endl;

	// std::cout << "Switching times 1: " << t_x[0] << std::endl;
	// std::cout << "Switching times 2: " << t_x[1] << std::endl;
	// std::cout << "Switching times 3: " << t_x[2] << std::endl<< std::endl;

	// std::cout << "Switching pose 1: " << x0[0] << std::endl;
	// std::cout << "Switching pose 2: " << x0[1] << std::endl;
	// std::cout << "Switching pose 3: " << x0[2] << std::endl<< std::endl;

	// std::cout << "Switching vel 1: " << vx0[0] << std::endl;
	// std::cout << "Switching vel 2: " << vx0[1] << std::endl;
	// std::cout << "Switching vel 3: " << vx0[2] << std::endl<< std::endl;

	// std::cout << "Switching accel 1: " << ax0[0] << std::endl;
	// std::cout << "Switching accel 2: " << ax0[1] << std::endl;
	// std::cout << "Switching accel 3: " << ax0[2] << std::endl << std::endl;

	// std::cout << "Jerk 1: " << jx[0] << std::endl;
	// std::cout << "Jerk 2: " << jx[1] << std::endl;
	// std::cout << "Jerk 3: " << jx[2] << std::endl << std::endl;


	acl_system::QuadGoal quad_goal;
	Eigen::MatrixXd Xc(3,2);
	double t = 0.1;

	then = clock();
	for (int i=0; i<500; i++){
	rp.eval_trajectory(Xc,X0,X0,t_x,t_x,t);
	}
	now = clock();

	double diff2 = 1000*((float)(now-then))/CLOCKS_PER_SEC/500;

	std::cout << "Function eval time [ms]: " << diff2 << std::endl << std::endl;

	std::cout << "x goal: " << Xc(0,0) << " y goal: " << Xc(0,1) << std::endl;
	std::cout << "vx goal: " << Xc(1,0) << " vy goal: " << Xc(1,1) << std::endl;
	std::cout << "ax goal: " << Xc(2,0) << " ay goal: " << Xc(2,1) << std::endl << std::endl;

	


	Eigen::MatrixXd scan;

	sensor_msgs::LaserScan test_scan;

	// Standard angle increment
	test_scan.angle_increment = 0.00628;
	test_scan.angle_max = 2;
	test_scan.angle_min = -2;

	for (int i=0;i<636;i++){
		if (i > 636/2){
			test_scan.ranges.push_back(4);
		}
		// else if (i==636/2){
		// 	test_scan.ranges.push_back(1);
		// }
		// else if (i==636){
		// 	test_scan.ranges.push_back(.2);
		// }
		else
			{
			test_scan.ranges.push_back(5);
		}
	}

	std::vector<double> scan2;

	then = clock();
	rp.convert_scan(test_scan,scan,scan2);
	now = clock();

	double diff3 = 1000*((float)(now-then))/CLOCKS_PER_SEC;

	std::cout << "Function eval time [ms]: " << diff3 << std::endl << std::endl;

	Eigen::MatrixXd X = Eigen::MatrixXd::Zero(3,2);
	Eigen::Vector3d goal ;
	goal << 3, 0, 0.5;
	double buff = 0.5;
	double v = 2;
	bool can_reach_goal;

	then = clock();
	for (int i=0; i<500; i++){
		rp.collision_check(X,scan,goal,buff,v,can_reach_goal);
	}	
	now = clock();

	double diff4 = 1000*((float)(now-then))/CLOCKS_PER_SEC/500;

	std::cout << "Collision check 1 eval time [ms]: " << diff4 << std::endl << std::endl;

	then = clock();
	for (int i=0; i<500; i++){
		rp.collision_check2(X,scan2,goal,buff,v,can_reach_goal);
	}	
	now = clock();

	diff4 = 1000*((float)(now-then))/CLOCKS_PER_SEC/500;

	std::cout << "Collision check 2 eval time [ms]: " << diff4 << std::endl << std::endl;

	std::cout << "Can reach goal: " << can_reach_goal << std::endl;


	nav_msgs::Path path;
	path.header.frame_id = "vicon";


	std::vector<double> t_x_{std::vector<double>(3,0)};
	std::vector<double> t_y_{std::vector<double>(3,0)};

	Eigen::Vector3d x_;
	Eigen::Vector3d y_;

	v = 2;
	double vfx_ = v*cos(3.14159/4);
	double vfy_ = v*sin(3.14159/4);

	x_ << X.col(0);
	y_ << X.col(1);

	x_(1) = 1;

	Eigen::Matrix4d X0_ = Eigen::Matrix4d::Zero();
	Eigen::Matrix4d Y0_ = Eigen::Matrix4d::Zero();

	Eigen::MatrixXd Xc_(3,2);

	Xc_(0,0) = 0;
	Xc_(1,0) = 0;
	Xc_(2,0) = 0;
	Xc_(0,1) = 0;
	Xc_(1,1) = 0;
	Xc_(2,1) = 0;

	rp.find_times(t_x_, X0_, x_, vfx_);
	rp.find_times(t_y_, Y0_, y_, vfy_);

	// std::cout << "Switching times 1: " << t_x_[0] << std::endl;
	// std::cout << "Switching times 2: " << t_x_[1] << std::endl;
	// std::cout << "Switching times 3: " << t_x_[2] << std::endl<< std::endl;

	// std::cout << "X0_: " << X0_ << std::endl;
	// std::cout << "Y0_: " << Y0_ << std::endl;

	double T = std::max(t_x_[0] + t_x_[1] + t_x_[2],t_y_[0] + t_y_[1] + t_y_[2]);
	double dt = 0.01;
	int num = (int) ceil(T/dt);

	double t_ = 0;
	geometry_msgs::PoseStamped temp;

	for(int i=0; i<num; i++){
		rp.eval_trajectory(Xc_,X0_,Y0_,t_x_,t_y_,t_);
		temp.pose.position.x = Xc_(0,0);
		temp.pose.position.y = Xc_(0,1);
		t_+=dt;
		path.poses.push_back(temp);
	}



	Eigen::MatrixXd Goals ;
	int part = 0;

	then = clock();
	for (int i=0;i<500;i++){
		rp.partition_scan(scan,Goals,part,pose);
	}
	now = clock();

	double diff5 = 1000*((float)(now-then))/CLOCKS_PER_SEC/500;

	std::cout << "Partition scan eval time [ms]: " << diff5 << std::endl << std::endl;

	std::cout << "Num of partitions: " << part << std::endl << std::endl;

	std::cout << "Cluster: " << Goals << std::endl;

	  ros::Publisher chatter_pub = n.advertise<nav_msgs::Path>("traj", 1);

	  ros::Rate loop_rate(10);


	  while (ros::ok())
	  {

	    chatter_pub.publish(path);

	    ros::spinOnce();

	    loop_rate.sleep();
	  }



	// std::cout << "Total latency [ms]: " << diff3 + diff2 + diff << std::endl << std::endl;

	return 0;
}