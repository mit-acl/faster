#include "tip.hpp"

void TIP::check_current_prim(Eigen::Matrix4d X0, Eigen::Matrix4d Y0, Eigen::Matrix4d Z0, std::vector<double> t_x, std::vector<double> t_y, std::vector<double> t_z, double t, Eigen::MatrixXd X, bool& clear)
{
	bool temp_clear = false;
	collision_detected_ = false;

	X_prop_ = Eigen::MatrixXd::Zero(4,3);
	X_prop_ << X;

	searchPoint_.x = X(0,0);
	searchPoint_.y = X(0,1);
	searchPoint_.z = X(0,2);

	std::vector<int> pointIdxNKNSearch(K_);
  	std::vector<float> pointNKNSquaredDistance(K_);

  	int iter;
  	if (virgin_) iter = c-1;
  	else iter = trees_.size()-1;
  	mean_distance_ = 100;
  	double dist;
  	for (int i=iter;i>-1;i--){
	  	trees_[i].nearestKSearch(searchPoint_, K_, pointIdxNKNSearch, pointNKNSquaredDistance);	
	    dist = std::sqrt(std::accumulate(pointNKNSquaredDistance.begin(), pointNKNSquaredDistance.end(), 0.0f)/pointIdxNKNSearch.size());
	    if (dist < mean_distance_) {
	    	mean_distance_ = dist; 
	    }
	    if (mean_distance_<buffer_){
    		temp_clear = false;
    		collision_detected_ = true;
    		break;
    	}
	}

    // If the obstacle is farther than safe distance or goal is within mean distance then we're good
    if (mean_distance_ > safe_distance_){
    	temp_clear = true;
    }

    // Something else is closer, need to prop to next time step
	else if (!collision_detected_){
		// evaluate at time required to travel d_min
		t_ = std::max(buffer_/v_max_,mean_distance_/v_max_) + t;

		while (!collision_detected_ && !temp_clear){

			mtx.lock();
			eval_trajectory(X0,Y0,Z0,t_x,t_y,t_z,t_,X_prop_);
			mtx.unlock();

			searchPoint_.x = X_prop_(0,0);
			searchPoint_.y = X_prop_(0,1);
			searchPoint_.z = X_prop_(0,2);

		  	mean_distance_ = 100;
		  	double dist;
		  	for (int i=iter;i>-1;i--){
			  	trees_[i].nearestKSearch(searchPoint_, K_, pointIdxNKNSearch, pointNKNSquaredDistance);	
			    dist = std::sqrt(std::accumulate(pointNKNSquaredDistance.begin(), pointNKNSquaredDistance.end(), 0.0f)/pointIdxNKNSearch.size());
			    if (dist<buffer_){
			    	collision_detected_ = true;
					temp_clear = false;
		    		break;
			    }
			    if (dist < mean_distance_) {
			    	mean_distance_ = dist; 
			    }
			}
    		distance_traveled_ = (X_prop_.row(0)-X.row(0)).norm();

    		// Check if the distance is less than our buffer
			if (mean_distance_ < buffer_){
				collision_detected_ = true;
				temp_clear = false;
			}
			// Check if the min distance is the current goal
			else if (distance_traveled_ > safe_distance_){
					temp_clear = true;
					distance_traveled_ = sensor_distance_;
			}			
			// Neither have happened so propogate again
			else{
				t_ += mean_distance_/v_max_;
			}
		}
	}
	clear = temp_clear;
}

void TIP::collision_check(Eigen::MatrixXd X, double buff, double v, bool& can_reach_goal, Eigen::Vector4d& local_goal_aug)
{
	//Re-intialize
	can_reach_goal = false;
	collision_detected_ = false;
	
	X_prop_ = Eigen::MatrixXd::Zero(4,3);
	X_prop_ << X;

	Eigen::Vector3d local_goal;
	local_goal << local_goal_aug.head(3);
	
	mtx.lock();
	get_traj(X,local_goal,v,t_x_,t_y_,t_z_,X_switch_,Y_switch_,Z_switch_,false);
	mtx.unlock();

	goal_distance_ = (goal_ - X.row(0).transpose()).norm();

	searchPoint_.x = X(0,0);
	searchPoint_.y = X(0,1);
	searchPoint_.z = X(0,2);

	std::vector<int> pointIdxNKNSearch(K_);
  	std::vector<float> pointNKNSquaredDistance(K_);

	int iter;
  	if (virgin_) iter = c-1;
  	else iter = trees_.size()-1;
  	mean_distance_ = 100;
  	double dist;
  	for (int i=iter;i>-1;i--){
	  	trees_[i].nearestKSearch(searchPoint_, K_, pointIdxNKNSearch, pointNKNSquaredDistance);	
	    dist = std::sqrt(std::accumulate(pointNKNSquaredDistance.begin(), pointNKNSquaredDistance.end(), 0.0f)/pointIdxNKNSearch.size());
	    if (dist < mean_distance_) {
	    	mean_distance_ = dist; 
	    }
	    if (dist<buff){
	    	can_reach_goal = false;
    		local_goal_aug(3) = inf;
    		break;
	    }
	}

	// std::cout << "first dist: " << mean_distance_ << std::endl;

    // If the obstacle is farther than safe distance or goal is within mean distance then we're good
    if (mean_distance_ > sensor_distance_ || mean_distance_ > goal_distance_){
    	can_reach_goal = true;
    }
    else if (mean_distance_ < buff){
    	can_reach_goal = false;
    	local_goal_aug(3) = inf;
    }
    // Something else is closer, need to prop to next time step
	else{
		// evaluate at time required to travel d_min
		t_ = std::max(buff/v,mean_distance_/v);

		while (!collision_detected_ && !can_reach_goal){

			mtx.lock();
			eval_trajectory(X_switch_,Y_switch_,Z_switch_,t_x_,t_y_,t_z_,t_,X_prop_);
			mtx.unlock();

			searchPoint_.x = X_prop_(0,0);
			searchPoint_.y = X_prop_(0,1);
			searchPoint_.z = X_prop_(0,2);

		  	mean_distance_ = 100;
		  	double dist;
		  	for (int i=iter;i>-1;i--){
			  	trees_[i].nearestKSearch(searchPoint_, K_, pointIdxNKNSearch, pointNKNSquaredDistance);	
			    dist = std::sqrt(std::accumulate(pointNKNSquaredDistance.begin(), pointNKNSquaredDistance.end(), 0.0f)/pointIdxNKNSearch.size());
			    if (dist < mean_distance_) {
			    	mean_distance_ = dist; 
			    }
			    if (dist<buff){
			    	can_reach_goal = false;
		    		local_goal_aug(3) = inf;
		    		break;
			    }
			}

    		distance_traveled_ = (X_prop_.row(0)-X.row(0)).norm();

    		// Check if the distance is less than our buffer
			if (mean_distance_ < buff){
				collision_detected_ = true;
				can_reach_goal = false;
				// ROS_INFO("Distance traveled: %0.2f", distance_traveled_);
				if (distance_traveled_ < safe_distance_) local_goal_aug(3) = inf;
				else local_goal_aug(3) = 0.05*pow(sensor_distance_-distance_traveled_,2);

			}
			// Check if the min distance is the current goal
			else if (distance_traveled_ > sensor_distance_ || distance_traveled_ > goal_distance_){
				// If traj is not within z bounds then it's not valid
				if (X_prop_(0,2) < z_min_ || X_prop_(0,2) > z_max_){
			    	can_reach_goal = false;
			    	local_goal_aug(3) = inf;
			    	return;
			    }
		    	else{
					can_reach_goal = true;
					distance_traveled_ = sensor_distance_;
				}
			}			
			// Neither have happened so propogate again
			else{
				t_ += mean_distance_/v;
			}
		}
	}
}