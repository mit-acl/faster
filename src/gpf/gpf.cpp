#include "gpf.hpp"

FilterGP::FilterGP(){

	ang_min = -PI/2;
	ang_max = PI/2;
	alt = 0; 
	max_range = 4;
	points_removed = 0;
	nx = 0;
	ny = 0;
	nz = 1;
	angle1 = 0;
	angle2 = 0;

	inf = std::numeric_limits<double>::max();

	att.setRPY(0.0, 0.0, 0.0);

	ros::param::get("~ground_range",ground_range);
	if(ground_range==-1){
		ground_range = inf;
	}

}

// Get POSE messages from estimator
void FilterGP::stateCB(const acl_system::ViconState& msg)
{
	// TODO time check.
	if (msg.has_pose) {
		alt = msg.pose.position.z;	
		tf::quaternionMsgToTF(msg.pose.orientation,att);
	} 
	// if (msg.has_twist) velCallback(msg.twist);
}

void FilterGP::scanCB(const sensor_msgs::LaserScan& msg)
 {

 	sensor_msgs::LaserScan msg_filtered;

	msg_filtered = msg;

	increment = msg.angle_increment;

	num_samples = (ang_max-ang_min) / increment;

	ang_min = msg.angle_min;
	ang_max = msg.angle_max;
	max_range = msg.range_max;

 	screenPrint();

 	findAngles(att,min_idx,max_idx);

	points_removed = 0;

    for (int i=min_idx; i < max_idx; i++){
    	double r = rangeCheck(i-min_idx);
    	
    	if (msg.ranges[i] > 0.8*r){
    		msg_filtered.ranges[i] = ground_range;
    		points_removed++;
    	}
    }
	
    filtered_scan_pub.publish(msg_filtered);
 }


double FilterGP::rangeCheck(int idx)
{
	double r = 0;
	double d ;
	if(ny!=0 and nx!=0){
		double a = -nx/ny;
		double b = -alt/ny;
		double xp = -b/(a+1/a);
		double yp = -xp/a;
		d = sqrt(xp*xp+yp*yp);
	}
	else if(ny==0){
		d = alt/nx;
	}
	else {
		d = alt/ny;
	}
	double zeta;
	if (nx!=0 or ny!=0){
		zeta = atan2(-ny,-nx);
		if (zeta > PI){
			zeta -= PI;
		}
		else if (zeta < -PI){
			zeta += PI;
		}
		double psi = angle1 + idx*increment - zeta;
		double l = d*tan(psi);
		r = sqrt(d*d+l*l);
	}
	else {
		r = max_range;
	}
	
	return r;

}

void FilterGP::findAngles(tf::Quaternion q, double &min_idx, double &max_idx)
{
	tf::Quaternion n = tf::Quaternion(0, 0, 1.0, 0.0);
	tf::Quaternion nb_q = q.inverse()*n*q;

	tf::Vector3 nb =  tf::Vector3(nb_q.getX(),nb_q.getY(),nb_q.getZ());

	nx = nb_q.getX();
	ny = nb_q.getY();
	nz = nb_q.getZ();

	if (std::abs(nx)<0.1){
		nx = 0;
	}

	if (std::abs(ny)<0.1){
		ny = 0;
	}

	if (std::abs(nz)<0.1){
		nz = 0;
	}

	if(nz!=0){
		double alt_B = alt/nz;
		double phi = acos(alt_B/max_range);
		double zeta = atan2(-ny,-nx);	

		if (zeta > PI){
			zeta -= PI;
		}
		else if (zeta < -PI){
			zeta += PI;
		}

		angle1 = std::max(-phi+zeta,ang_min);
		angle2 = std::min(phi+zeta,ang_max);

		min_idx = (angle1-ang_min) / increment;
		max_idx = (angle2-ang_min) / increment;
	}


	min_idx = round(min_idx);
	max_idx = round(max_idx);
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
	tf::Matrix3x3(att).getRPY(r, p, y);
	msg << "Height:           " << alt << std::endl;
	msg << "Points Removed:   " << points_removed << std::endl;
	msg << "Attitude:	  r: " << r << "  p: " << p << "  y: " << y << std::endl;

	ROS_INFO_STREAM_THROTTLE(SCREEN_PRINT_RATE, msg.str());
	// Print at 1/0.5 Hz = 2 Hz
}
