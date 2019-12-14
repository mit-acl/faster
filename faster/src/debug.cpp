#include "ros/ros.h"
#include "solver.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "test", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh_p("~");

	initialize_optimizer();	

	double dt = 0.1;
	double x0[6] = {1,0,1,0,0,0}; 
	double xf[6] = {0,0,1,0,0,0}; 

	load_default_data(dt, x0, xf);

	double then = ros::Time::now().toSec();
	int r = optimize();

	printf("Time: %0.6f \n",ros::Time::now().toSec()-then);

	printf("Result: %d \n",r);

	double ** x; double ** u;
	x = get_state();
	u = get_control();

	for (int i=1;i<20;i++){
		ROS_INFO("%0.2f %0.2f %0.2f",x[i][0], x[i][3], u[i][0]);
	}

	// ros::spin();

	return 0;
}
