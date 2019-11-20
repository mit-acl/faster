#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <iostream>
struct XYZNormalUnc
{
  PCL_ADD_POINT4D;      // This adds the member point[3] which can also be accessed using the point (which is float[4]
  float cov[3];         // Assuming a diagonal matrix
  PCL_ADD_NORMAL4D;     // This adds the member normal[3] which can also be accessed using the point (which is float[4]
  float cov_normal[3];  // Assuming a diagonal matrix

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                   // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(XYZNormalUnc, (float[3], cov, cov)(float[3], cov_normal, cov_normal))

// Let's overload the << operator now
std::ostream& operator<<(std::ostream& os, const XYZNormalUnc& point)
{
  os << "Point: " << point.x << " (" << point.cov[0] << "), " << point.y << " (" << point.cov[1] << "), " << point.z
     << " (" << point.cov[2] << ")"
     << "\n Normal:" << point.normal_x << " (" << point.cov_normal[0] << "), " << point.normal_y << " ("
     << point.cov_normal[1] << "), " << point.normal_z << " (" << point.cov_normal[2] << ")";
  return os;
}

int main(int argc, char** argv)
{
  pcl::PointCloud<XYZNormalUnc> cloud;
  cloud.points.resize(1);
  cloud.width = 1;
  cloud.height = 1;

  cloud.points[0].x = cloud.points[0].y = cloud.points[0].z = 0;
  cloud.points[0].cov[0] = cloud.points[0].cov[1] = cloud.points[0].cov[2] = 1.5;
  cloud.points[0].normal_x = cloud.points[0].normal_y = cloud.points[0].normal_z = 0.333;
  cloud.points[0].cov_normal[0] = cloud.points[0].cov_normal[1] = cloud.points[0].cov_normal[2] = 1.5;

  std::cout << cloud.points[0] << std::endl;
}