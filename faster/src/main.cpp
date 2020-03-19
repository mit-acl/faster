/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "faster_ros.hpp"

int main(int argc, char **argv)
{
  // Initializes ROS, and sets up a node
  ros::init(argc, argv, "faster");
  ros::NodeHandle nh("~");
  ros::CallbackQueue custom_queue1;
  ros::CallbackQueue custom_queue2;
  nh.setCallbackQueue(&custom_queue1);
  nh.setCallbackQueue(&custom_queue2);

  FasterRos FasterRos(nh);

  ros::AsyncSpinner spinner1(0, &custom_queue1);
  ros::AsyncSpinner spinner2(0, &custom_queue2);
  spinner1.start();  // start spinner of the custom queue 1
  spinner2.start();  // start spinner of the custom queue 2

  ros::spin();  // spin the normal queue

  // ros::AsyncSpinner spinner(0);  // 0 means # of threads= # of CPU cores

  ros::waitForShutdown();
  return 0;
}