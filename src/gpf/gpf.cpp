#include "gpf.hpp"

FilterGP::FilterGP(){

	ang_min = -PI/2;
	ang_max = PI/2;
	max_range = 4;
	points_removed = 0;

	inf = std::numeric_limits<double>::max();

	q.setRPY(0.0, 0.0, 0.0);

	ros::param::get("~ground_range",ground_range);
	ros::param::get("~filter_thresh",filter_thresh);

	if(ground_range==-1){
		ground_range = inf;
	}

	// listener.waitForTransform("/vicon", "/laser", ros::Time::now(), ros::Duration(1));
 //    listener.lookupTransform("/vicon", "/laser", ros::Time::now(), trans);
 //        //Do something

 //    pose = trans.getOrigin();
 //    q = trans.getRotation();

	ROS_INFO("Ground plane filter initialized.");


}

// Get POSE messages from estimator
void FilterGP::stateCB(const acl_system::ViconState& msg)
{
	// TODO time check.
	if (msg.has_pose) {
		geometry_msgs::Vector3 position;
		position.x = msg.pose.position.x;
		position.y = msg.pose.position.y;
		position.z = msg.pose.position.z;
		tf::vector3MsgToTF(position,pose);	
		tf::quaternionMsgToTF(msg.pose.orientation,q);
	} 
}

void FilterGP::scanCB(const sensor_msgs::LaserScan& msg)
 {

 	// std::cout << ros::Time() << std::endl;
 	// std::cout << ros::Duration(1) << std::endl;
 	// double then = ros::Time::now().toSec();
  //   listener.waitForTransform("/vicon", "/laser", ros::Time::now(), ros::Duration(1));
  //   listener.lookupTransform("/vicon", "/laser", ros::Time::now(), trans);

  //   std::cout << "Tranform latency: " << ros::Time::now().toSec() - then << std::endl;
  //       //Do something

  //   pose = trans.getOrigin();
  //   q = trans.getRotation();


 	sensor_msgs::LaserScan msg_filtered;

	msg_filtered = msg;

	increment = msg.angle_increment;

	num_samples = (ang_max-ang_min) / increment;

	ang_min = msg.angle_min;
	ang_max = msg.angle_max;
	max_range = msg.range_max;

	points_removed = 0;

	scan_points.header.stamp = ros::Time::now();
	scan_points.header.frame_id = "vicon";

	scan_points.poses.clear();

	for (int i=0; i < num_samples; i++){
		if (!isinf(msg.ranges[i]) && !isnan(msg.ranges[i])){
			
			geometry_msgs::PoseStamped temp;

			x = msg.ranges[i]*std::cos(ang_min + increment*i);
			y = msg.ranges[i]*std::sin(ang_min + increment*i);
			z = 0.1;
			tf::Quaternion n_body_q = tf::Quaternion(x, y, z, 0.0);
			tf::Quaternion n_wolrd_q = q*n_body_q*q.inverse();

			tf::Vector3 n_world =  tf::Vector3(n_wolrd_q.getX()+pose.getX(),n_wolrd_q.getY()+pose.getY(),n_wolrd_q.getZ()+pose.getZ());

			temp.pose.position.x = n_world.getX();
			temp.pose.position.y = n_world.getY();
			temp.pose.position.z = n_world.getZ();

			scan_points.poses.push_back(temp);

			if (n_world.getZ() < filter_thresh){
	    		msg_filtered.ranges[i] = ground_range;
	    		points_removed++;
	    	}
    	}
	}

 	// screenPrint();
	
    filtered_scan_pub.publish(msg_filtered);
    point_array_pub.publish(scan_points);
 }


void FilterGP::screenPrint()
{
	if (not errorMsg.str().empty())
		ROS_ERROR_STREAM_THROTTLE(SCREEN_PRINT_RATE, errorMsg.str());

	if (not warnMsg.str().empty())
		ROS_WARN_STREAM_THROTTLE(SCREEN_PRINT_RATE, warnMsg.str());

	std::ostringstream msg;
	msg.setf(std::ios::fixed); // give all the doubles the same precision
	msg.setf(std::ios::showpos); // show +/- signs always
	msg << std::setprecision(4) << std::endl; // set precision to 4 decimal places

	double r, p, y;
	tf::Matrix3x3(q).getRPY(r, p, y);
	msg << "Height:           " << alt << std::endl;
	msg << "Points Removed:   " << points_removed << std::endl;
	msg << "Attitude:	  r: " << r << "  p: " << p << "  y: " << y << std::endl;

	ROS_INFO_STREAM_THROTTLE(SCREEN_PRINT_RATE, msg.str());
	// Print at 1/0.5 Hz = 2 Hz
}
