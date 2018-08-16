#ifndef UTILS_HPP
#define UTILS_HPP
#include <iostream>
// TODO: This values should be the same as the global_mapper.yaml
#define WDX 20    //[m] world dimension in x
#define WDY 20    //[m] world dimension in y
#define WDZ 4     //[m] world dimension in z
#define RES 0.15  //[m] cell dimension

#define RED 1
#define GREEN 2
#define BLUE 3
#define BLUE_LIGHT 4
#define YELLOW 5
#define ORANGE_TRANS 6

#define DC 0.01           //(seconds) Duration for the interpolation=Value of the timer pubGoal
#define GOAL_RADIUS 0.2   //(m) Drone has arrived to the goal when distance_to_goal<GOAL_RADIUS
#define DRONE_RADIUS 0.3  //(m) Used for collision checking

#define STATE 0
#define INPUT 1

#define V_MAX 20
#define A_MAX 2

#define POS 0
#define VEL 1
#define ACCEL 2
#define JERK 3

// inline is needed to avoid the "multiple definitions" error. Other option is to create the utils.cpp file, and put
// there the function (and here only the prototype)
inline float solvePolyOrder2(Eigen::Vector3f coeff)
{
  // std::cout << "solving\n" << coeff << std::endl;
  float a = coeff[0];
  float b = coeff[1];
  float c = coeff[2];
  float dis = b * b - 4 * a * c;
  if (dis >= 0)
  {
    float x1 = (-b - sqrt(dis)) / (2 * a);  // x1 will always be smaller than x2
    float x2 = (-b + sqrt(dis)) / (2 * a);

    if (x1 >= 0)
    {
      return x1;
    }
    if (x2 >= 0)
    {
      return x2;
    }
  }
  printf("No solution found to the equation\n");
  return std::numeric_limits<float>::max();
}

#endif