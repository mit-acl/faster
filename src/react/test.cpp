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
	std::vector<double> x0{std::vector<double>(3,0)};
	std::vector<double> vx0{std::vector<double>(3,0)};
	std::vector<double> ax0{std::vector<double>(3,0)};
	std::vector<double> x{std::vector<double>(3,0)};

	double j = 30;
	double vf = -1;

	rp.find_times(t_x,x0,vx0,ax0,j,x,vf);

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

	std::cout << "j: " << j << std::endl << std::endl;



	return 0;
}