// The MIT License (MIT)

// Copyright (c) 2016 Miquel Massot Campos

//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

#ifndef TERRAIN_SLAM_H
#define TERRAIN_SLAM_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Aff_transformation_3.h>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <vector>

typedef CGAL::Simple_cartesian<double>     Kernel;
typedef CGAL::Aff_transformation_3<Kernel> Aff3;
typedef Kernel::Point_3 Point3;
typedef Kernel::Vector_3 Vector3;

typedef CGAL::Exact_predicates_inexact_constructions_kernel KernelT;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned, KernelT> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
typedef CGAL::Triangulation_2<KernelT, Tds>::Locate_type LocateType;
typedef CGAL::Delaunay_triangulation_2<KernelT, Tds> Delaunay;
typedef Delaunay::Point Point;
typedef Delaunay::Face_handle FaceHandle;
typedef Delaunay::Vertex_handle VertexHandle;

namespace terrain_slam {
class TerrainSlam {
public:
  TerrainSlam(int argc, char** argv);
  void parseCommandLine(int argc, char** argv);
  void process(const std::string& clouds_dir, double size);
  void readFiles(const std::vector<std::string>& cloud_names, const std::vector<std::string>& cloud_paths);
  bool cvToCGAL(const cv::Mat& in, std::vector<Aff3>& out);
  void getCloudPaths(const std::string& path,
                     const std::string& format,
                     std::vector<std::string>& cloud_names,
                     std::vector<std::string>& cloud_paths);
  Eigen::Quaternion<double> rpyToRotationMatrix(double roll, double pitch, double yaw);
  Eigen::Matrix4d buildTransform(const Eigen::Quaternion<double> q, const Eigen::Vector3d t);
  void cv2eigen(const cv::Point3d& p, Eigen::Vector3d& v);
  void processPatches();

  double patch_size_;
  std::vector<Eigen::Matrix4d> robot_tf_;
  std::vector<Eigen::Matrix4d> camera_tf_;
  std::vector<std::vector<Eigen::Vector3d> > clouds_;

};  // Class
}   // Namespace

#endif // TERRAIN_SLAM_H
