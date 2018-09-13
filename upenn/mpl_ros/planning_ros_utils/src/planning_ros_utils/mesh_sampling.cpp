/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

//***** ROS Wrapper, generate point cloud from stl
//*** sikang@seas.upenn.edu

//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#define PCL_VIS 0

inline double uniform_deviate(int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

inline void randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2,
                                float c3, Eigen::Vector4f &p)
{
  float r1 = static_cast<float>(uniform_deviate(rand()));
  float r2 = static_cast<float>(uniform_deviate(rand()));
  float r1sqr = sqrtf(r1);
  float OneMinR1Sqr = (1 - r1sqr);
  float OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
  p[3] = 0;
}

inline void randPSurface(vtkPolyData *polydata, std::vector<double> *cumulativeAreas, double totalArea,
                         Eigen::Vector4f &p)
{
  float r = static_cast<float>(uniform_deviate(rand()) * totalArea);

  std::vector<double>::iterator low = std::lower_bound(cumulativeAreas->begin(), cumulativeAreas->end(), r);
  vtkIdType el = vtkIdType(low - cumulativeAreas->begin());

  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType *ptIds = NULL;
  polydata->GetCellPoints(el, npts, ptIds);
  polydata->GetPoint(ptIds[0], A);
  polydata->GetPoint(ptIds[1], B);
  polydata->GetPoint(ptIds[2], C);
  randomPointTriangle(float(A[0]), float(A[1]), float(A[2]), float(B[0]), float(B[1]), float(B[2]), float(C[0]),
                      float(C[1]), float(C[2]), p);
}

void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples,
                      pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
  polydata->BuildCells();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();

  double p1[3], p2[3], p3[3], totalArea = 0;
  std::vector<double> cumulativeAreas(cells->GetNumberOfCells(), 0);
  size_t i = 0;
  vtkIdType npts = 0, *ptIds = NULL;
  for (cells->InitTraversal(); cells->GetNextCell(npts, ptIds); i++)
  {
    polydata->GetPoint(ptIds[0], p1);
    polydata->GetPoint(ptIds[1], p2);
    polydata->GetPoint(ptIds[2], p3);
    totalArea += vtkTriangle::TriangleArea(p1, p2, p3);
    cumulativeAreas[i] = totalArea;
  }

  cloud_out.points.resize(n_samples);
  cloud_out.width = static_cast<pcl::uint32_t>(n_samples);
  cloud_out.height = 1;

  for (i = 0; i < n_samples; i++)
  {
    Eigen::Vector4f p;
    randPSurface(polydata, &cumulativeAreas, totalArea, p);
    cloud_out.points[i].x = p[0];
    cloud_out.points[i].y = p[1];
    cloud_out.points[i].z = p[2];
  }
}

sensor_msgs::PointCloud convertToCloud(const pcl::PointCloud<pcl::PointXYZ> &msg)
{
  sensor_msgs::PointCloud2 cloud2;
  pcl::toROSMsg(msg, cloud2);
  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud);

  return cloud;
}

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int default_number_samples = 100000;
float default_leaf_size = 0.01f;

void printHelp(int, char **argv)
{
  print_info("Params are:\n");
  print_info("                     -n_samples X      = number of samples (default: ");
  print_value("%d", default_number_samples);
  print_info(")\n");
  print_info("                     -leaf_size X  = the XYZ leaf size for the "
             "VoxelGrid -- for data reduction (default: ");
  print_value("%f", default_leaf_size);
  print_info(" m)\n");
}

/* ---[ */
int main(int argc, char **argv)
{
  print_info("Convert a CAD model to a point cloud using uniform sampling. For "
             "more information, use: %s -h\n",
             argv[0]);

  printHelp(argc, argv);

  ros::init(argc, argv, "stl_to_cloud");
  ros::NodeHandle n("~");

  // Parse command line arguments
  int SAMPLE_POINTS_ = default_number_samples;
  n.param("n_samples", SAMPLE_POINTS_, default_number_samples);
  float leaf_size = default_leaf_size;
  n.param("leaf_size", leaf_size, default_leaf_size);
  bool save_pcd = false;
  n.param("save_pcd", save_pcd, false);
  printf("n_samples: %d, leaf_size: %.3f, save_pcd: %d\n", SAMPLE_POINTS_, leaf_size, save_pcd);

  // Parse the command line arguments for .ply and PCD files
  std::string stlFileName = "/home/jtorde/Downloads/"
                            "willowgarage_visual2m.stl";
  // n.param("file", stlFileName, std::string(""));

  vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();

  if (stlFileName.compare(stlFileName.size() - 3, 3, "stl") == 0)
  {
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(stlFileName, mesh);
    pcl::io::mesh2vtk(mesh, polydata1);
  }
  else
  {
    ROS_WARN("Fail to open the file %s", stlFileName.c_str());
    return 0;
  }
  // make sure that the polygons are triangles!
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
#if VTK_MAJOR_VERSION < 6
  triangleFilter->SetInput(polydata1);
#else
  triangleFilter->SetInputData(polydata1);
#endif
  triangleFilter->Update();

  vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
  triangleMapper->Update();
  polydata1 = triangleMapper->GetInput();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
  uniform_sampling(polydata1, SAMPLE_POINTS_, *cloud_1);

  // Voxelgrid
  VoxelGrid<PointXYZ> grid_;
  grid_.setInputCloud(cloud_1);
  grid_.setLeafSize(leaf_size, leaf_size, leaf_size);

  pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);
  grid_.filter(*res);

  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  sensor_msgs::PointCloud cloud = convertToCloud(*res);
  cloud.header.frame_id = "world";
  cloud_pub.publish(cloud);

  ROS_INFO("Cloud published!");

  if (save_pcd)
  {
    pcl::io::savePCDFileASCII("tmp.pcd", *res);
    ROS_INFO("Cloud saved!");
  }

  ros::spin();

  return 0;
}
