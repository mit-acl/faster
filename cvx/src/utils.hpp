#ifndef UTILS_HPP
#define UTILS_HPP
#include <iostream>
#include "ros/ros.h"
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
// TODO: This values should be the same as the global_mapper.yaml
#define WDX 20.0  //[m] world dimension in x
#define WDY 20.0  //[m] world dimension in y
#define WDZ 4.0   //[m] world dimension in z
#define RES 0.15  //[m] cell dimension

#define RED 1
#define RED_TRANS 2
#define RED_TRANS_TRANS 3
#define GREEN 4
#define BLUE 5
#define BLUE_TRANS 6
#define BLUE_TRANS_TRANS 7
#define BLUE_LIGHT 8
#define YELLOW 9
#define ORANGE_TRANS 10

#define DC 0.01           //(seconds) Duration for the interpolation=Value of the timer pubGoal
#define GOAL_RADIUS 0.2   //(m) Drone has arrived to the goal when distance_to_goal<GOAL_RADIUS
#define DRONE_RADIUS 0.3  //(m) Used for collision checking

#define STATE 0
#define INPUT 1

#define V_MAX 2
#define A_MAX 2
#define J_MAX 10

#define POS 0
#define VEL 1
#define ACCEL 2
#define JERK 3

// inline is needed to avoid the "multiple definitions" error. Other option is to create the utils.cpp file, and put
// there the function (and here only the prototype)

inline void saturate(double& var, double min, double max)
{
  if (var < min)
  {
    var = min;
  }
  else if (var > max)
  {
    var = max;
  }
}

inline double angleBetVectors(Eigen::Vector3d a, Eigen::Vector3d b)
{
  double tmp = a.dot(b) / (a.norm() * b.norm());
  saturate(tmp, -1, 1);
  return acos(tmp);
}

// returns the points around B sampled in the sphere with radius r and center center.
inline std::vector<Eigen::Vector3d> samplePointsSphere(Eigen::Vector3d B, double r, Eigen::Vector3d center)
{
  std::vector<Eigen::Vector3d> tmp;

  Eigen::Vector3d dir = B - center;
  double x = dir[0], y = dir[1], z = dir[2];

  double theta0 = acos(z / (sqrt(x * x + y * y + z * z)));
  double phi0 = atan2(y, x);

  Eigen::AngleAxis<double> rot_z(phi0, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxis<double> rot_y(theta0, Eigen::Vector3d::UnitY());

  for (double theta = 0; theta <= 3.14 / 2; theta = theta + 3.14 / 10)
  {
    for (double phi = 0; phi <= 2 * 3.14; phi = phi + 3.14 / 10)
    {
      Eigen::Vector3d p1, p2;
      p1[0] = r * sin(theta) * cos(phi);
      p1[1] = r * sin(theta) * sin(phi);
      p1[2] = r * cos(theta);
      Eigen::Vector3d trans = center;
      p2 = rot_z * rot_y * p1 + trans;

      if (p2[2] > 0)  // If below the ground, discard
      {
        tmp.push_back(p2);
      }
    }
  }

  return tmp;
}

// returns the points around B sampled in the sphere with radius r and center center, and sampled intelligently with
// the given path
// last_index_inside_sphere is the the index of the last point that is inside the sphere (should be provided as a
// parameter to this function)
// B is the first intersection of JPS with the sphere
inline std::vector<Eigen::Vector3d> samplePointsSphereWithJPS(Eigen::Vector3d B, double r, Eigen::Vector3d center,
                                                              vec_Vecf<3> path, int last_index_inside_sphere)
{
  /*  printf("To sample points1\n");
    center = Eigen::Vector3d(1, 2, 3);
    r = 6.57;
    B = Eigen::Vector3d(7.5, 2, 4);
    last_index_inside_sphere = 3;
    std::vector<Eigen::Vector3d> path;
    path.push_back(center);
    path.push_back(Eigen::Vector3d(1.5, -2, 3));
    path.push_back(Eigen::Vector3d(4, 2.5, 2));
    path.push_back(Eigen::Vector3d(3, 6, 3));
    printf("To sample points2\n");

    path.push_back(B);*/

  // printf("last_index_inside_sphere=%d", last_index_inside_sphere);

  path[last_index_inside_sphere + 1] = B;  // path will always have last_index_inside_sphere + 2 elements at least
  // std::vector<Eigen::Vector2d> proj;     // Projection of the waypoints in the sphere, in spherical coordinates
  std::vector<Eigen::Vector3d> samples;  // Points sampled in the sphere
  Eigen::Vector3d dir;
  double x, y, z;

  for (int i = last_index_inside_sphere + 1; i >= 1; i--)
  {
    Eigen::Vector3d point_i = path[i] - center;        // point i expressed with origin=origin sphere
    Eigen::Vector3d point_im1 = path[i - 1] - center;  // point i minus 1
    double angle_max = angleBetVectors(point_i, point_im1);
    Eigen::Vector3d perp = (point_i.cross(point_im1)).normalized();  // perpendicular vector to point_i and point_ip1;

    for (double angle = 0; angle < angle_max; angle = angle + 0.34)
    {
      Eigen::AngleAxis<double> rot(angle, perp);
      dir = rot * point_i;
      double x = dir[0], y = dir[1], z = dir[2];

      double theta = acos(z / (sqrt(x * x + y * y + z * z)));
      double phi = atan2(y, x);

      Eigen::Vector3d sample;
      sample[0] = r * sin(theta) * cos(phi);
      sample[1] = r * sin(theta) * sin(phi);
      sample[2] = r * cos(theta);
      samples.push_back(sample + center);
    }
  }

  if (last_index_inside_sphere >= 1)
  {
    // add the last sample (intersection of pa)
    Eigen::Vector3d last = path[1] - center;
    double x = last[0], y = last[1], z = last[2];
    double theta = acos(z / (sqrt(x * x + y * y + z * z)));
    double phi = atan2(y, x);

    Eigen::Vector3d sample;
    sample[0] = r * sin(theta) * cos(phi);
    sample[1] = r * sin(theta) * sin(phi);
    sample[2] = r * cos(theta);

    samples.push_back(sample + center);
  }

  /*  printf("****SAMPLES OBTAINED*****\n");
    for (int i = 0; i < samples.size() - 1; i++)
    {
      std::cout << "(" << samples[i][0] << "," << samples[i][1] << "," << samples[i][2] << ")" << std::endl;
    }
  */

  // now let's concatenate some uniform samples in case last_index_inside_sphere=0;
  std::vector<Eigen::Vector3d> uniform_samples = samplePointsSphere(B, r, center);

  samples.insert(samples.end(), uniform_samples.begin(),
                 uniform_samples.end());  // concatenate samples and uniform samples

  return samples;
}

inline void angle_wrap(double& diff)
{
  diff = fmod(diff + M_PI, 2 * M_PI);
  if (diff < 0)
    diff += 2 * M_PI;
  diff -= M_PI;
}

inline pcl::PointXYZ eigenPoint2pclPoint(Eigen::Vector3d p)
{
  // std::cout << "solving\n" << coeff << std::endl;
  pcl::PointXYZ tmp(p[0], p[1], p[2]);
  return tmp;
}

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

inline std_msgs::ColorRGBA color(int id)
{
  std_msgs::ColorRGBA red;
  red.r = 1;
  red.g = 0;
  red.b = 0;
  red.a = 1;
  std_msgs::ColorRGBA red_trans;
  red_trans.r = 1;
  red_trans.g = 0;
  red_trans.b = 0;
  red_trans.a = 0.7;
  std_msgs::ColorRGBA red_trans_trans;
  red_trans_trans.r = 1;
  red_trans_trans.g = 0;
  red_trans_trans.b = 0;
  red_trans_trans.a = 0.4;
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1;
  blue.a = 1;
  std_msgs::ColorRGBA blue_trans;
  blue_trans.r = 0;
  blue_trans.g = 0;
  blue_trans.b = 1;
  blue_trans.a = 0.7;
  std_msgs::ColorRGBA blue_trans_trans;
  blue_trans_trans.r = 0;
  blue_trans_trans.g = 0;
  blue_trans_trans.b = 1;
  blue_trans_trans.a = 0.4;
  std_msgs::ColorRGBA blue_light;
  blue_light.r = 0.5;
  blue_light.g = 0.7;
  blue_light.b = 1;
  blue_light.a = 1;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1;
  green.b = 0;
  green.a = 1;
  std_msgs::ColorRGBA yellow;
  yellow.r = 1;
  yellow.g = 1;
  yellow.b = 0;
  yellow.a = 1;
  std_msgs::ColorRGBA orange_trans;  // orange transparent
  orange_trans.r = 1;
  orange_trans.g = 0.5;
  orange_trans.b = 0;
  orange_trans.a = 0.7;
  switch (id)
  {
    case RED:
      return red;
      break;
    case RED_TRANS:
      return red_trans;
      break;
    case RED_TRANS_TRANS:
      return red_trans_trans;
      break;
    case BLUE:
      return blue;
      break;
    case BLUE_TRANS:
      return blue_trans;
      break;
    case BLUE_TRANS_TRANS:
      return blue_trans_trans;
      break;
    case BLUE_LIGHT:
      return blue_light;
      break;
    case GREEN:
      return green;
      break;
    case YELLOW:
      return yellow;
      break;
    case ORANGE_TRANS:
      return orange_trans;
      break;
    default:
      ROS_ERROR("COLOR NOT DEFINED");
  }
}

// coeff is from highest degree to lowest degree. Returns the smallest positive real solution. Returns -1 if a
// root is imaginary or if it's negative

inline geometry_msgs::Point pointOrigin()
{
  geometry_msgs::Point tmp;
  tmp.x = 0;
  tmp.y = 0;
  tmp.z = 0;
  return tmp;
}

inline geometry_msgs::Point eigen2point(Eigen::Vector3d vector)
{
  geometry_msgs::Point tmp;
  tmp.x = vector[0];
  tmp.y = vector[1];
  tmp.z = vector[2];
  return tmp;
}

inline geometry_msgs::Vector3 vectorNull()
{
  geometry_msgs::Vector3 tmp;
  tmp.x = 0;
  tmp.y = 0;
  tmp.z = 0;
  return tmp;
}

inline geometry_msgs::Vector3 vectorUniform(double a)
{
  geometry_msgs::Vector3 tmp;
  tmp.x = a;
  tmp.y = a;
  tmp.z = a;
  return tmp;
}

template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;

template <int N>
using Vecf = Eigen::Matrix<decimal_t, N, 1>;  // Be CAREFUL, because this is with doubles!

template <int N>
using vec_Vecf = vec_E<Vecf<N>>;

// returns 1 if there is an intersection between the segment P1-P2 and the plane given by coeff=[A B C D]
// (Ax+By+Cz+D==0)  returns 0 if there is no intersection.
// The intersection point is saved in "intersection"
inline bool getIntersectionWithPlane(Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector4d coeff,
                                     Eigen::Vector3d* inters)
{
  /*  std::cout << "Coefficients" << std::endl;
    std::cout << coeff.transpose() << std::endl;

    std::cout << "P1" << std::endl;
    std::cout << P1.transpose() << std::endl;

    std::cout << "P2" << std::endl;
    std::cout << P2.transpose() << std::endl;*/

  double A = coeff[0];
  double B = coeff[1];
  double C = coeff[2];
  double D = coeff[3];
  // http://www.ambrsoft.com/TrigoCalc/Plan3D/PlaneLineIntersection_.htm
  double x1 = P1[0];
  double a = (P2[0] - P1[0]);
  double y1 = P1[1];
  double b = (P2[1] - P1[1]);
  double z1 = P1[2];
  double c = (P2[2] - P1[2]);
  double t = -(A * x1 + B * y1 + C * z1 + D) / (A * a + B * b + C * c);
  (*inters)[0] = x1 + a * t;
  (*inters)[1] = y1 + b * t;
  (*inters)[2] = z1 + c * t;
  // printf("t=%f\n", t);
  bool result =
      (t < 0 || t > 1) ? false : true;  // False if the intersection is with the line P1-P2, not with the segment P1-P2
                                        /*  if (result)
                                          {
                                            std::cout << "Intersection\n" << *inters << std::endl;
                                          }*/
  return result;
}

// given 2 points (A inside and B outside the sphere) it computes the intersection of the lines between
// that 2 points and the sphere
inline Eigen::Vector3d getIntersectionWithSphere(Eigen::Vector3d A, Eigen::Vector3d B, double r, Eigen::Vector3d center)
{
  // http://www.ambrsoft.com/TrigoCalc/Sphere/SpherLineIntersection_.htm
  /*  std::cout << "Center=" << std::endl << center << std::endl;
    std::cout << "Radius=" << std::endl << r << std::endl;
    std::cout << "First Point=" << std::endl << A << std::endl;
    std::cout << "Second Point=" << std::endl << B << std::endl;*/
  float x1 = A[0];
  float y1 = A[1];
  float z1 = A[2];

  float x2 = B[0];
  float y2 = B[1];
  float z2 = B[2];

  float x3 = center[0];
  float y3 = center[1];
  float z3 = center[2];

  float a = pow((x2 - x1), 2) + pow((y2 - y1), 2) + pow((z2 - z1), 2);
  float b = 2 * ((x2 - x1) * (x1 - x3) + (y2 - y1) * (y1 - y3) + (z2 - z1) * (z1 - z3));
  float c = x3 * x3 + y3 * y3 + z3 * z3 + x1 * x1 + y1 * y1 + z1 * z1 - 2 * (x3 * x1 + y3 * y1 + z3 * z1) - r * r;

  float discrim = b * b - 4 * a * c;
  if (discrim <= 0)
  {
    printf("The line is tangent or doesn't intersect, returning the first point\n");
    return A;
  }
  else
  {
    float t = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
    float x_int = x1 + (x2 - x1) * t;
    float y_int = y1 + (y2 - y1) * t;
    float z_int = z1 + (z2 - z1) * t;
    Eigen::Vector3d intersection(x_int, y_int, z_int);
    // std::cout << "Intersection=" << std::endl << intersection << std::endl;
    return intersection;
  }
}

// Given a path (starting inside the sphere and finishing outside of it) expressed by a vector of 3D-vectors (points),
// it returns its first intersection with a sphere of radius=r and center=center
// the center is added as the first point of the path to ensure that the first element of the path is inside the sphere
// (to avoid issues with the first point of JPS2)
inline Eigen::Vector3d getFirstIntersectionWithSphere(vec_Vecf<3> path, double r, Eigen::Vector3d center,
                                                      int* last_index_inside_sphere = NULL,
                                                      bool* noPointsOutsideSphere = NULL)
{
  // printf("here\n");
  if (noPointsOutsideSphere != NULL)
  {  // this argument has been provided
    *noPointsOutsideSphere = false;
  }
  // printf("here2\n");
  // path.insert(path.begin(), center);
  int index = -1;
  for (int i = 0; i < path.size(); i++)
  {
    double dist = (path[i] - center).norm();
    if (dist > r)
    {
      index = i;  // This is the first point outside the sphere
      break;
    }
  }

  Eigen::Vector3d A;
  Eigen::Vector3d B;

  switch (index)
  {
    case -1:  // no points are outside the sphere --> find projection center-lastPoint into the sphere
      A = center;
      B = path[path.size() - 1];
      if (last_index_inside_sphere != NULL)
      {
        *last_index_inside_sphere = path.size() - 1;
      }
      if (noPointsOutsideSphere != NULL)
      {  // this argument has been provided
        *noPointsOutsideSphere = true;
      }
      break;
    case 0:  // First element is outside the sphere
      printf("First element is still oustide the sphere, there is sth wrong\n");
      // std::cout << "radius=" << r << std::endl;
      // std::cout << "dist=" << (path[0] - center).norm() << std::endl;
      break;
    default:
      A = path[index - 1];
      B = path[index];
      // printf("index-1=%d\n", index - 1);
      if (last_index_inside_sphere != NULL)
      {
        *last_index_inside_sphere = index - 1;
      }
  }

  bool thereIsIntersec;
  Eigen::Vector3d intersection = getIntersectionWithSphere(A, B, r, center);

  return intersection;
}

// Given a path (starting inside the sphere and finishing outside of it) expressed by a vector of 3D-vectors (points),
// it returns its first intersection with a sphere of radius=r and center=center
inline Eigen::Vector3d getLastIntersectionWithSphere(vec_Vecf<3> path, double r, Eigen::Vector3d center)
{
  // printf("In getLastIntersectionWithSphere\n");
  int index = -1;
  for (int i = path.size() - 1; i >= 0; i--)
  {
    double dist = (path[i] - center).norm();
    if (dist < r)
    {
      index = i;  // This is the first point inside the sphere
      break;
    }
  }

  // printf("Index=%d\n", index);

  if (index == path.size() - 1)
  {
    std::cout << "radius=" << r << std::endl;
    std::cout << "dist=" << (path[index] - center).norm() << std::endl;

    printf("ERROR, the goal is inside the sphere Sb, returning the last point\n");
    return path[path.size() - 1];
  }
  // Note that it's guaranteed that index>=1, since the path[0] is always inside the sphere.
  Eigen::Vector3d A = path[index];
  Eigen::Vector3d B = path[index + 1];

  Eigen::Vector3d intersection = getIntersectionWithSphere(A, B, r, center);
  return intersection;
}

// Same as the previous one, but also returns in Jdist the distance form the last intersection to the goal (following
// the path)
inline Eigen::Vector3d getLastIntersectionWithSphere(vec_Vecf<3> path, double r, Eigen::Vector3d center, double* Jdist)
{
  // printf("********************In getLastIntersectionWithSphere\n");

  /*  printf("Radius=%f\n", r);
    std::cout << "Center\n" << center.transpose() << std::endl;
    printf("Path\n");
    for (int i = 0; i < path.size(); i++)
    {
      std::cout << path[i].transpose() << std::endl;
    }*/

  int index = -1;
  for (int i = path.size() - 1; i >= 0; i--)
  {
    double dist = (path[i] - center).norm();
    if (dist < r)
    {
      index = i;  // This is the first point inside the sphere
      break;
    }
  }

  // printf("Index=%d\n", index);

  if (index == path.size() - 1)
  {
    printf("ERROR, the goal is inside the sphere Sb, returning the last point\n");
    *Jdist = 0;
    return path[path.size() - 1];
  }

  // Note that it's guaranteed that index>=1, since the path[0] is always inside the sphere.
  Eigen::Vector3d A = path[index];      // point inside the sphere
  Eigen::Vector3d B = path[index + 1];  // point outside the sphere

  Eigen::Vector3d intersection = getIntersectionWithSphere(A, B, r, center);

  /*  std::cout << "B\n " << B.transpose() << std::endl;
    std::cout << "intersection\n " << intersection.transpose() << std::endl;*/

  *Jdist = (B - intersection).norm();
  // printf("primer valor es=%f\n", (B - intersection).norm());
  for (int i = index + 1; i < path.size() - 1; i++)
  {
    // printf("otro valor es %f \n", (path[i + 1] - path[i]).norm());
    *Jdist = *Jdist + (path[i + 1] - path[i]).norm();
  }

  // printf("Jist computed is %f\n", *Jdist);

  // printf("********************END OF getLastIntersectionWithSphere\n");
  return intersection;
}

// returns the point placed between two concentric spheres with radii ra, rb, and center=center
// If the path goes out from the 1st sphere, and then enters again, these points are also considered!
// I.e: it returns all the points between the first point that goes out from the sphere and the last point that is
// inside Sb
inline vec_Vecf<3> getPointsBw2Spheres(vec_Vecf<3> path, double ra, double rb, Eigen::Vector3d center)
{
  bool out_first_sphere = false;

  int index_1st_point_outside_Sa;
  int index_last_point_inside_Sb;

  for (int i = 0; i < path.size(); i++)
  {
    float dist = (path[i] - center).norm();
    if (dist > ra)
    {
      index_1st_point_outside_Sa = i;
      break;
    }
  }

  for (int i = path.size() - 1; i >= 0; i--)
  {
    float dist = (path[i] - center).norm();
    if (dist < rb)
    {
      index_last_point_inside_Sb = i;
      break;
    }
  }

  vec_Vecf<3> tmp;
  for (int i = index_1st_point_outside_Sa; i <= index_last_point_inside_Sb; i++)
  {
    tmp.push_back(path[i]);
  }
  return tmp;
}

inline void printElementsOfJPS(vec_Vecf<3> path)
{
  printf("Elements of the path given:\n");
  for (int i = 0; i < path.size(); i++)
  {
    std::cout << path[i].transpose() << std::endl;
  }
}

inline vec_Vecf<3> copyJPS(vec_Vecf<3> path)
{
  vec_Vecf<3> tmp;
  for (int i = 0; i < path.size(); i++)
  {
    tmp.push_back(path[i]);
  }
  return tmp;
}

#endif