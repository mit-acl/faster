#include "ros/ros.h"
#include "solver.h"

#include "visualization_msgs/Marker.h"

#include <Eigen/Dense>

#include <acl_msgs/State.h>
#include <acl_msgs/QuadGoal.h>
#include <acl_msgs/QuadFlightMode.h>
#include <acl_msgs/TermGoal.h>

class CVX
{
public:
  
  CVX( ros::NodeHandle nh );


private: 
  // class methods
  void   pubTraj( double **x );
  void   pubTraj( Eigen::MatrixXd X );
  void   goalCB(const acl_msgs::TermGoal& msg);
  void   stateCB(const acl_msgs::State& msg);
  void   modeCB(const acl_msgs::QuadFlightMode& msg);
  void   pubCB (const ros::TimerEvent& e);

  double callOptimizer(double u_max, double x0[], double xf[]);
  int    checkConvergence(double xf[], double xf_opt[]);
  void   genNewTraj(double u_max, double xf[]);
  void   interpInput(double dt, double xf[], double u0[], double x0[], double ** u, double ** x, Eigen::MatrixXd &U, Eigen::MatrixXd &X);

  visualization_msgs::Marker setpoint_;
  acl_msgs::QuadGoal quadGoal_;
  acl_msgs::QuadFlightMode flight_mode_;

  ros::NodeHandle   nh_;
  ros::Publisher    pub_goal_;
  ros::Publisher    pub_traj_;
  ros::Publisher    pub_setpoint_;
  ros::Subscriber   sub_goal_;
  ros::Subscriber   sub_state_;
  ros::Subscriber   sub_mode_;
  ros::Timer        pubGoalTimer_ ;

  Eigen::MatrixXd U_, X_;
  bool replan_, optimized_, use_ff_;
  double u_min_, u_max_, z_start_, spinup_time_, z_land_;
  int N_ = 20;
};
