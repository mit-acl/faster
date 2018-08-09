#include "cvx.hpp"

int main(int argc, char **argv)
{
  // Initializes ROS, and sets up a node
  ros::init(argc, argv, "cvx");
  ros::NodeHandle nh("~");
  CVX cvx(nh);
  /*  ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();*/
  ros::spin();
  return 0;
}