#include "rp.hpp"


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "reactive_planner", ros::init_options::NoSigintHandler);

	ros::NodeHandle n("~");

	REACT rp;

	double a = 10;
	std::cout << "Original val: " << a << std::endl;
	rp.saturate(a,-1,5);
	std::cout << "Saturate val: " << a << std::endl << std::endl;

	std::vector<double> t_x{std::vector<double>(3,0)};
	std::vector<double> x0{std::vector<double>(4,0)};
	std::vector<double> vx0{std::vector<double>(4,0)};
	std::vector<double> ax0{std::vector<double>(4,0)};
	std::vector<double> x{std::vector<double>(4,0)};
	std::vector<double> jx{std::vector<double>(4,0)};


	double j = 30;
	double vf = 1;
	double t0 = 0;

	// Test if we're in decel phase
	// x[1] = vf/2;
	// x[2] = 5.477;

	double now;
	double then = ros::Time::now().toSec();

	rp.find_times(t_x,x0,vx0,ax0,jx,t0,x,vf);
	now = ros::Time::now().toSec();

	std::cout << "Function eval time [ms]: " << 1000*(now - then) << std::endl << std::endl;

	std::cout << "Switching times 1: " << t_x[0] << std::endl;
	std::cout << "Switching times 2: " << t_x[1] << std::endl;
	std::cout << "Switching times 3: " << t_x[2] << std::endl<< std::endl;

	std::cout << "Switching pose 1: " << x0[0] << std::endl;
	std::cout << "Switching pose 2: " << x0[1] << std::endl;
	std::cout << "Switching pose 3: " << x0[2] << std::endl<< std::endl;

	std::cout << "Switching vel 1: " << vx0[0] << std::endl;
	std::cout << "Switching vel 2: " << vx0[1] << std::endl;
	std::cout << "Switching vel 3: " << vx0[2] << std::endl<< std::endl;

	std::cout << "Switching accel 1: " << ax0[0] << std::endl;
	std::cout << "Switching accel 2: " << ax0[1] << std::endl;
	std::cout << "Switching accel 3: " << ax0[2] << std::endl << std::endl;

	std::cout << "Jerk 1: " << jx[0] << std::endl;
	std::cout << "Jerk 2: " << jx[1] << std::endl;
	std::cout << "Jerk 3: " << jx[2] << std::endl << std::endl;


	acl_system::QuadGoal goal;
	double t = 1;

	then = ros::Time::now().toSec();
	rp.eval_trajectory(goal,t_x,t_x,t);
	now = ros::Time::now().toSec();

	std::cout << "Function eval time [ms]: " << 1000*(now - then) << std::endl << std::endl;

	std::cout << "x goal: " << goal.pos.x << " y goal: " << goal.pos.y << std::endl;
	std::cout << "vx goal: " << goal.vel.x << " vy goal: " << goal.vel.y << std::endl;
	std::cout << "ax goal: " << goal.accel.x << " ay goal: " << goal.accel.y << std::endl << std::endl;




	return 0;
}