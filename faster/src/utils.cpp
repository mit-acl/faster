/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "utils.hpp"

void printStateDeque(std::deque<state>& data)
{
  for (int i = 0; i < data.size(); i++)
  {
    data[i].printHorizontal();
  }
}

void printStateVector(std::vector<state>& data)
{
  for (int i = 0; i < data.size(); i++)
  {
    data[i].printHorizontal();
  }
}

void vectorOfVectors2MarkerArray(vec_Vecf<3> traj, visualization_msgs::MarkerArray* m_array, std_msgs::ColorRGBA color,
                                 int type, std::vector<double> radii)
{
  if (traj.size() == 0)
  {
    return;
  }

  // printf("In vectorOfVectors2MarkerArray\n");
  geometry_msgs::Point p_last = eigen2point(traj[0]);

  bool first_element = true;
  int i = 50000;  // large enough to prevent conflict with other markers
  int j = 0;

  for (const auto& it : traj)
  {
    i++;
    if (first_element and type == visualization_msgs::Marker::ARROW)  // skip the first element
    {
      first_element = false;
      continue;
    }

    visualization_msgs::Marker m;
    m.type = type;
    m.action = visualization_msgs::Marker::ADD;
    m.id = i;
    m.color = color;
    // m.scale.z = 1;

    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    geometry_msgs::Point p = eigen2point(it);
    if (type == visualization_msgs::Marker::ARROW)
    {
      m.scale.x = 0.02;
      m.scale.y = 0.04;
      m.points.push_back(p_last);
      m.points.push_back(p);
      // std::cout << "pushing marker\n" << m << std::endl;
      p_last = p;
    }
    else
    {
      double scale = 0.1;  // Scale is the diameter of the sphere
      if (radii.size() != 0)
      {  // If argument provided
        scale = 2 * radii[j];
      }
      m.scale.x = scale;
      m.scale.y = scale;
      m.scale.z = scale;
      m.pose.position = p;
    }
    (*m_array).markers.push_back(m);
    j = j + 1;
  }
}

std_msgs::ColorRGBA getColorJet(double v, double vmin, double vmax)
{
  std_msgs::ColorRGBA c;
  c.r = 1;
  c.g = 1;
  c.b = 1;
  c.a = 1;
  // white
  double dv;

  if (v < vmin)
    v = vmin;
  if (v > vmax)
    v = vmax;
  dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv))
  {
    c.r = 0;
    c.g = 4 * (v - vmin) / dv;
  }
  else if (v < (vmin + 0.5 * dv))
  {
    c.r = 0;
    c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  }
  else if (v < (vmin + 0.75 * dv))
  {
    c.r = 4 * (v - vmin - 0.5 * dv) / dv;
    c.b = 0;
  }
  else
  {
    c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    c.b = 0;
  }

  return (c);
}

std_msgs::ColorRGBA color(int id)
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

//## From Wikipedia - http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void quaternion2Euler(tf2::Quaternion q, double& roll, double& pitch, double& yaw)
{
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
}

void quaternion2Euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  quaternion2Euler(tf_q, roll, pitch, yaw);
}

void quaternion2Euler(geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  quaternion2Euler(tf_q, roll, pitch, yaw);
}

void saturate(double& var, double min, double max)
{
  // std::cout << "min=" << min << " max=" << max << std::endl;
  if (var < min)
  {
    // std::cout << "Saturating to min" << var << std::endl;
    var = min;
  }
  else if (var > max)
  {
    // std::cout << "Saturating to max" << var << std::endl;
    var = max;
  }
  // std::cout << "Value saturated" << var << std::endl;
}

visualization_msgs::Marker getMarkerSphere(double scale, int my_color)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color = color(my_color);

  return marker;
}

double angleBetVectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  // printf("In angleBetVEctors\n");
  // std::cout << a.transpose() << std::endl;
  // std::cout << b.transpose() << std::endl;

  double tmp = a.dot(b) / (a.norm() * b.norm());
  // printf("tmp=%f\n", tmp);
  saturate(tmp, -1, 1);
  return acos(tmp);
}

// returns the points around B sampled in the sphere with radius r and center center.
std::vector<Eigen::Vector3d> samplePointsSphere(Eigen::Vector3d& B, double r, Eigen::Vector3d& center)
{
  std::vector<Eigen::Vector3d> tmp;

  Eigen::Vector3d dir = B - center;
  double x = dir[0], y = dir[1], z = dir[2];

  double theta0 = acos(z / (sqrt(x * x + y * y + z * z)));
  double phi0 = atan2(y, x);

  Eigen::AngleAxis<double> rot_z(phi0, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxis<double> rot_y(theta0, Eigen::Vector3d::UnitY());

  for (double theta = 0; theta <= 3.14 / 2; theta = theta + 3.14 / 5)
  {
    for (double phi = 0; phi <= 2 * 3.14; phi = phi + 3.14 / 5)
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
      if (theta == 0)
      {  // to avoid including multiple times the pointB
        break;
      }
    }
  }

  return tmp;
}

void printElementsOfJPS(vec_Vecf<3>& path)
{
  // printf("Elements of the path given:\n");
  for (int i = 0; i < path.size(); i++)
  {
    std::cout << path[i].transpose() << std::endl;
  }
}

// returns the points around B sampled in the sphere with radius r and center center, and sampled intelligently with
// the given path
// last_index_inside_sphere is the the index of the last point that is inside the sphere (should be provided as a
// parameter to this function)
// B is the first intersection of JPS with the sphere
std::vector<Eigen::Vector3d> samplePointsSphereWithJPS(Eigen::Vector3d& B, double r, Eigen::Vector3d& center_sent,
                                                       vec_Vecf<3>& path_sent, int last_index_inside_sphere)
{
  printf("In samplePointsSphereWithJPS\n");
  // printElementsOfJPS(path_sent);

  vec_Vecf<3> path;

  for (int i = 0; i < path_sent.size(); i++)
  {
    path.push_back(path_sent[i]);  // Local copy of path_sent
  }

  Eigen::Vector3d center(center_sent[0], center_sent[1], center_sent[2]);

  /*  for (int i = 0; i < path_sent.size(); i++)
    {
      path.push_back(path_sent[i]);  // Local copy of path_sent
    }
  */
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
    Eigen::Vector3d point_i = (path[i] - center);        // point i expressed with origin=origin sphere
    Eigen::Vector3d point_im1 = (path[i - 1] - center);  // point i minus 1

    /*    Eigen::Vector3d& point_i_ref(point_i);      // point i expressed with origin=origin sphere
        Eigen::Vector3d& point_im1_ref(point_im1);  // point i minus 1*/

    std::cout << "i=" << i << "point_i=" << path[i].transpose() << std::endl;
    std::cout << "i=" << i << "point_im1=" << path[i - 1].transpose() << std::endl;

    Eigen::Vector3d a = point_i;
    Eigen::Vector3d b = point_im1;

    double angle_max;
    if (a.norm() != 0 && b.norm() != 0)
    {
      // printf("estoy dentro\n");
      double tmp = a.dot(b) / (a.norm() * b.norm());
      saturate(tmp, -1, 1);
      angle_max = acos(tmp);
      printf("tmp=%f\n", tmp);
      printf("angle_max=%f\n", angle_max);
      if (angle_max < 0.02)
      {
        samples.push_back(B);
        continue;
      }
    }
    else
    {
      samples.push_back(B);
      continue;  // it's weird, but solves the problem when the vectors a and b are the same ones...
    }
    /*    std::cout << "a=" << a.transpose() << std::endl;
        std::cout << "b=" << b.transpose() << std::endl;

        std::cout << "a=" << a.norm() << std::endl;
        std::cout << "b=" << b.norm() << std::endl;

        std::cout << "numerator=" << a.dot(b) << std::endl;
        std::cout << "denominator=" << a.norm() * b.norm() << std::endl;*/

    /*    if (std::isnan(angle_max))
        {
          continue;  // sometimes the path has two same elements. Skip it (if not there will be an error later in
       cvxgen)
        }*/

    Eigen::Vector3d perp = (point_i.cross(point_im1)).normalized();  // perpendicular vector to point_i and point_ip1;
    printf("Perpendicular vector=\n");
    std::cout << perp << std::endl;

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

  // now let's flip the vector
  // std::reverse(samples.begin(), samples.end());

  // now let's concatenate some uniform samples in case last_index_inside_sphere=0;
  // printf("ahora mismo samples vale:\n");
  /*  for (int i = 0; i < samples.size(); i++)
    {
      std::cout << samples[i].transpose() << std::endl;
    }*/
  // printf("despues del bucle:\n");
  std::vector<Eigen::Vector3d> uniform_samples = samplePointsSphere(B, r, center);

  // printf("uniform_samples vale:\n");
  /*  for (int i = 0; i < uniform_samples.size(); i++)
    {
      std::cout << uniform_samples[i].transpose() << std::endl;
    }*/

  samples.insert(samples.end(), uniform_samples.begin(),
                 uniform_samples.end());  // concatenate samples and uniform samples

  printf("**y despues samples vale:\n");
  for (int i = 0; i < samples.size(); i++)
  {
    std::cout << samples[i].transpose() << std::endl;
  }
  /*printf("returning it:\n");*/

  return samples;
}

void angle_wrap(double& diff)
{
  diff = fmod(diff + M_PI, 2 * M_PI);
  if (diff < 0)
    diff += 2 * M_PI;
  diff -= M_PI;
}

pcl::PointXYZ eigenPoint2pclPoint(Eigen::Vector3d& p)
{
  // std::cout << "solving\n" << coeff << std::endl;
  pcl::PointXYZ tmp(p[0], p[1], p[2]);
  return tmp;
}

vec_Vec3f pclptr_to_vec(const pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr_cloud)
{
  vec_Vec3f pts;
  pts.resize(ptr_cloud->points.size());
  for (unsigned int i = 0; i < ptr_cloud->points.size(); i++)
  {
    pts[i](0) = ptr_cloud->points[i].x;
    pts[i](1) = ptr_cloud->points[i].y;
    pts[i](2) = ptr_cloud->points[i].z;
  }

  return pts;
}

vec_Vec3f pclptr_to_vec(const pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr_cloud1,
                        const pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr_cloud2)
{
  vec_Vec3f pts;
  pts.reserve(ptr_cloud1->points.size() + ptr_cloud2->points.size());
  for (unsigned int i = 0; i < ptr_cloud1->points.size(); i++)
  {
    pts[i](0) = ptr_cloud1->points[i].x;
    pts[i](1) = ptr_cloud1->points[i].y;
    pts[i](2) = ptr_cloud1->points[i].z;
  }

  for (unsigned int i = ptr_cloud1->points.size(); i < ptr_cloud2->points.size(); i++)
  {
    pts[i](0) = ptr_cloud2->points[i].x;
    pts[i](1) = ptr_cloud2->points[i].y;
    pts[i](2) = ptr_cloud2->points[i].z;
  }

  return pts;
}

float solvePolyOrder2(Eigen::Vector3f coeff)
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

// coeff is from highest degree to lowest degree. Returns the smallest positive real solution. Returns -1 if a
// root is imaginary or if it's negative

geometry_msgs::Point pointOrigin()
{
  geometry_msgs::Point tmp;
  tmp.x = 0;
  tmp.y = 0;
  tmp.z = 0;
  return tmp;
}

Eigen::Vector3d vec2eigen(geometry_msgs::Vector3 vector)
{
  Eigen::Vector3d tmp;
  tmp << vector.x, vector.y, vector.z;
  return tmp;
}

geometry_msgs::Vector3 eigen2rosvector(Eigen::Vector3d vector)
{
  geometry_msgs::Vector3 tmp;
  tmp.x = vector(0, 0);
  tmp.y = vector(1, 0);
  tmp.z = vector(2, 0);
  return tmp;
}

geometry_msgs::Point eigen2point(Eigen::Vector3d vector)
{
  geometry_msgs::Point tmp;
  tmp.x = vector[0];
  tmp.y = vector[1];
  tmp.z = vector[2];
  return tmp;
}

geometry_msgs::Vector3 vectorNull()
{
  geometry_msgs::Vector3 tmp;
  tmp.x = 0;
  tmp.y = 0;
  tmp.z = 0;
  return tmp;
}

geometry_msgs::Vector3 vectorUniform(double a)
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
bool getIntersectionWithPlane(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2, const Eigen::Vector4d& coeff,
                              Eigen::Vector3d& intersection)
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
  // std::cout << "t vale " << t << std::endl;
  (intersection)[0] = x1 + a * t;
  (intersection)[1] = y1 + b * t;
  (intersection)[2] = z1 + c * t;
  // printf("t=%f\n", t);
  bool result =
      (t < 0 || t > 1) ? false : true;  // False if the intersection is with the line P1-P2, not with the segment P1-P2
  // std::cout << "result vale" << result << std::endl;
  /*  if (result)
    {
      std::cout << "Intersection\n" << intersection(0) << ", " << intersection(1) << ", " << intersection(2) <<
    std::endl;
    }*/
  return result;
}

double normJPS(vec_Vecf<3>& path, int index_start)
{
  double distance = 0;
  for (int i = index_start; i < path.size() - 1; i++)
  {
    distance = distance + (path[i + 1] - path[i]).norm();
  }
  return distance;
}

// Crop the end of a JPS path by a given distance
void reduceJPSbyDistance(vec_Vecf<3>& path, double d)
{
  /*  std::cout<<"In reduceJPSbyDistance, path=\n";
    printElementsOfJPS(path);*/
  double dist_so_far = 0;
  for (int ii = path.size() - 1; ii > 0; ii--)
  {
    Eigen::Vector3d v = path[ii] - path[ii - 1];
    double dist_bet_segments = v.norm();
    dist_so_far = dist_so_far + dist_bet_segments;
    if (dist_so_far > d)
    {
      double dist_wanted = dist_so_far - d;
      path.erase(path.begin() + ii, path.end());
      path.push_back(path[path.size() - 1] + v.normalized() * dist_wanted);
      break;
    }
  }
  /*  std::cout << "Finished reduceJPSbyDistance, path=\n";
    printElementsOfJPS(path);*/
}
// given 2 points (A inside and B outside the sphere) it computes the intersection of the lines between
// that 2 points and the sphere
Eigen::Vector3d getIntersectionWithSphere(Eigen::Vector3d& A, Eigen::Vector3d& B, double r, Eigen::Vector3d& center)
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
    printf("The line is tangent or doesn't intersect, returning the intersection with the center and the first "
           "point\n");

    float x1 = center[0];
    float y1 = center[1];
    float z1 = center[2];

    float x2 = A[0];
    float y2 = A[1];
    float z2 = A[2];

    float x3 = center[0];
    float y3 = center[1];
    float z3 = center[2];

    float a = pow((x2 - x1), 2) + pow((y2 - y1), 2) + pow((z2 - z1), 2);
    float b = 2 * ((x2 - x1) * (x1 - x3) + (y2 - y1) * (y1 - y3) + (z2 - z1) * (z1 - z3));
    float c = x3 * x3 + y3 * y3 + z3 * z3 + x1 * x1 + y1 * y1 + z1 * z1 - 2 * (x3 * x1 + y3 * y1 + z3 * z1) - r * r;

    float t = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
    float x_int = x1 + (x2 - x1) * t;
    float y_int = y1 + (y2 - y1) * t;
    float z_int = z1 + (z2 - z1) * t;
    Eigen::Vector3d intersection(x_int, y_int, z_int);

    return intersection;
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
Eigen::Vector3d getFirstIntersectionWithSphere(vec_Vecf<3>& path, double r, Eigen::Vector3d& center,
                                               int* last_index_inside_sphere, bool* noPointsOutsideSphere)
{
  // printf("Utils: In getFirstIntersectionWithSphere\n");
  // printElementsOfJPS(path);
  // std::cout << "Utils: center=" << center.transpose() << std::endl;
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
    // std::cout << "path[i]=" << path[i].transpose() << std::endl;
    double dist = (path[i] - center).norm();
    // std::cout << "dist=" << dist << std::endl;
    if (dist > r)
    {
      // std::cout << "dist>r!" << std::endl;
      index = i;  // This is the first point outside the sphere
      break;
    }
  }

  Eigen::Vector3d A;
  Eigen::Vector3d B;

  Eigen::Vector3d intersection;
  // std::cout << "Utils: index=" << index << std::endl;
  switch (index)
  {
    case -1:  // no points are outside the sphere --> return last element
      // std::cout << "Utils: no points are outside the sphere!!!" << std::endl;
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
      // std::cout << "Calling intersecion1 with A=" << A.transpose() << "  and B=" << B.transpose() << std::endl;
      intersection = getIntersectionWithSphere(A, B, r, center);

      // intersection = path[path.size() - 1];

      // std::cout << "Utils: Returning intersection=" << intersection.transpose() << std::endl;
      // intersection = path[path.size() - 1];
      if (last_index_inside_sphere != NULL)
      {
        *last_index_inside_sphere = path.size() - 1;
      }
      break;
    case 0:  // First element is outside the sphere
      printf("First element is still oustide the sphere, there is sth wrong, returning the first element\n");
      intersection = path[0];
      // std::cout << "radius=" << r << std::endl;
      // std::cout << "dist=" << (path[0] - center).norm() << std::endl;
      if (last_index_inside_sphere != NULL)
      {
        *last_index_inside_sphere = 1;
      }
      break;
    default:
      A = path[index - 1];
      B = path[index];
      // std::cout << "Utils: calling intersecion2 with A=" << A.transpose() << "  and B=" << B.transpose() <<
      // std::endl;
      intersection = getIntersectionWithSphere(A, B, r, center);
      // printf("index-1=%d\n", index - 1);
      if (last_index_inside_sphere != NULL)
      {
        *last_index_inside_sphere = index - 1;
      }
  }

  // bool thereIsIntersec;
  // std::cout << "Utils: returning intersection= " <<intersection.transpose()<< std::endl;
  return intersection;
}

// Given a path (starting inside the sphere and finishing outside of it) expressed by a vector of 3D-vectors (points),
// it returns its first intersection with a sphere of radius=r and center=center
Eigen::Vector3d getLastIntersectionWithSphere(vec_Vecf<3> path, double r, Eigen::Vector3d center)
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
    // std::cout << "radius=" << r << std::endl;
    // std::cout << "dist=" << (path[index] - center).norm() << std::endl;

    // printf("ERROR, the goal is inside the sphere Sb, returning the last point\n");
    return path[path.size() - 1];
  }
  // Note that it's guaranteed that index>=1, since the path[0] is always inside the sphere.
  Eigen::Vector3d A = path[index];
  Eigen::Vector3d B = path[index + 1];

  Eigen::Vector3d intersection = getIntersectionWithSphere(A, B, r, center);
  // printf("returning intersection=\n");
  // std::cout << intersection.transpose() << std::endl;
  return intersection;
}

double getDistancePath(vec_Vecf<3>& path)
{
  double distance = 0;
  for (int i = 0; i < path.size() - 1; i++)
  {
    distance = distance + (path[i + 1] - path[i]).norm();
  }
  return distance;
}

// Same as the previous one, but also returns dist = the distance form the last intersection to the goal (following
// the path)
Eigen::Vector3d getLastIntersectionWithSphere(vec_Vecf<3> path, double r, Eigen::Vector3d center, double* Jdist)
{
  /*  printf("********************In getLastIntersectionWithSphere\n");

    printf("Radius=%f\n", r);
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

  // std::cout << "B\n " << B.transpose() << std::endl;
  // std::cout << "intersection\n " << intersection.transpose() << std::endl;

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
vec_Vecf<3> getPointsBw2Spheres(vec_Vecf<3> path, double ra, double rb, Eigen::Vector3d center)
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

vec_Vecf<3> copyJPS(vec_Vecf<3> path)
{
  vec_Vecf<3> tmp;
  for (int i = 0; i < path.size(); i++)
  {
    tmp.push_back(path[i]);
  }
  return tmp;
}

visualization_msgs::MarkerArray stateVector2ColoredMarkerArray(const std::vector<state>& data, int type,
                                                               double max_value)
{
  visualization_msgs::MarkerArray marker_array;

  if (data.size() == 0)
  {
    return marker_array;
  }
  geometry_msgs::Point p_last;
  p_last.x = data[0].pos(0);
  p_last.y = data[0].pos(1);
  p_last.z = data[0].pos(2);

  int j = type * 9000;
  for (int i = 0; i < data.size(); i = i + 1)
  {
    j = j + 1;
    double vel = data[i].vel.norm();
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    m.id = j;
    m.color = getColorJet(vel, 0, max_value);  // note that par_.v_max is per axis!
    m.scale.x = 0.15;
    m.scale.y = 0;
    m.scale.z = 0;
    // std::cout << "Mandando bloque" << X.block(i, 0, 1, 3) << std::endl;
    geometry_msgs::Point p;
    p.x = data[i].pos(0);
    p.y = data[i].pos(1);
    p.z = data[i].pos(2);
    m.points.push_back(p_last);
    m.points.push_back(p);
    // std::cout << "pushing marker\n" << m << std::endl;
    p_last = p;
    marker_array.markers.push_back(m);
  }
  return marker_array;
}

// P1-P2 is the direction used for projection. P2 is the goal clicked. wdx, wdy and wdz are the widths of a 3D box
// centered on P1
Eigen::Vector3d projectPointToBox(Eigen::Vector3d& P1, Eigen::Vector3d& P2, double wdx, double wdy, double wdz)
{
  double x_max = P1(0) + wdx / 2;
  double x_min = P1(0) - wdx / 2;
  double y_max = P1(1) + wdy / 2;
  double y_min = P1(1) - wdy / 2;
  double z_max = P1(2) + wdz / 2;
  double z_min = P1(2) - wdz / 2;

  if ((P2(0) < x_max && P2(0) > x_min) && (P2(1) < y_max && P2(1) > y_min) && (P2(2) < z_max && P2(2) > z_min))
  {
    // Clicked goal is inside the map
    return P2;
  }
  Eigen::Vector3d inters;
  std::vector<Eigen::Vector4d> all_planes = {
    Eigen::Vector4d(1, 0, 0, -x_max),  // Plane X right
    Eigen::Vector4d(-1, 0, 0, x_min),  // Plane X left
    Eigen::Vector4d(0, 1, 0, -y_max),  // Plane Y right
    Eigen::Vector4d(0, -1, 0, y_min),  // Plane Y left
    Eigen::Vector4d(0, 0, 1, -z_max),  // Plane Z up
    Eigen::Vector4d(0, 0, -1, z_min)   // Plane Z down
  };

  vec_Vecf<3> intersections;

  int axis;  // 1 is x, 2 is y, 3 is z
  for (int i = 0; i < 6; i++)
  {
    if (getIntersectionWithPlane(P1, P2, all_planes[i], inters) == true)
    {
      intersections.push_back(inters);
    }
  }

  if (intersections.size() == 0)
  {  // There is no intersection
    ROS_ERROR("This is impossible, there should be an intersection");
  }
  std::vector<double> distances;
  // And now take the nearest intersection
  for (size_t i = 0; i < intersections.size(); i++)
  {
    double distance = (intersections[i] - P1).norm();
    distances.push_back(distance);
  }
  int minElementIndex = std::min_element(distances.begin(), distances.end()) - distances.begin();
  inters = intersections[minElementIndex];

  return inters;
}

void deleteVertexes(vec_Vecf<3>& JPS_path, int max_value)
{
  if (JPS_path.size() > max_value + 1)  // If I have more than (max_value + 1) vertexes
  {
    JPS_path.erase(JPS_path.begin() + max_value + 1,
                   JPS_path.end());  // Force JPS to have less than max_value elements
  }
}
