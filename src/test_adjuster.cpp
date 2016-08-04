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

#include <terrain_slam/adjuster_xy.h>
#include <terrain_slam/adjuster.h>
#include <terrain_slam/clouds.h>
#include <terrain_slam/pcl_tools.h>

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <iostream>
#include <random>

// void createPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
//   cloud.push_back(pcl::PointXYZ(-0.7,  0.7, 0));
//   cloud.push_back(pcl::PointXYZ( 1.2,  1.3, 0));
//   cloud.push_back(pcl::PointXYZ( 1.4, -1.1, 0));
//   cloud.push_back(pcl::PointXYZ(-1.0, -1.0, 0));
// }

// void addNoise(double noise, pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output) {
//   std::random_device rd;
//   std::mt19937 gen(rd());
//   std::uniform_real_distribution<> dis(-1, +1);
//   for (size_t i = 0; i < input->size(); ++i) {
//     pcl::PointXYZ p;
//     p.x = input->points[i].x + dis(gen)*noise;
//     p.y = input->points[i].y + dis(gen)*noise;
//     p.z = input->points[i].z + dis(gen)*noise;
//     output->push_back(p);
//   }
// }

Eigen::Matrix4d registerClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
  boost::shared_ptr<terrain_slam::CloudPatch> c1(new terrain_slam::CloudPatch());
  boost::shared_ptr<terrain_slam::CloudPatch> c2(new terrain_slam::CloudPatch());
  std::cout << "Adjusting... " << std::endl;

  // double gt_x = -2.439199;
  // double gt_y = 2.290323;
  // double gt_z = 1.433578;
  // double xx = 0.941450;
  // double xy = -0.337153;
  // double yy = 0.941450;

  // c2->transform.T(0, 0) = xx;
  // c2->transform.T(0, 1) = xy;
  // c2->transform.T(1, 0) = -xy;
  // c2->transform.T(1, 1) = yy;
  // c2->transform.T(0, 3) = gt_x;
  // c2->transform.T(1, 3) = gt_y;
  // c2->transform.T(2, 3) = gt_z;

  // pcl::transformPointCloud(*cloud2, *cloud2, c2->transform.T.cast<float>());
  // pcl_tools::saveCloud(cloud1, "orig_test", 1);
  // pcl_tools::saveCloud(cloud2, "orig_test", 2);

  // Eigen::Vector4f source_centroid, target_centroid;
  // pcl::compute3DCentroid(*cloud1, source_centroid);
  // pcl::compute3DCentroid(*cloud2, target_centroid);
  // Eigen::Matrix4f source_pre_tf = Eigen::Matrix4f::Identity();
  // Eigen::Matrix4f target_pre_tf = Eigen::Matrix4f::Identity();
  // Eigen::Vector4f translation = source_centroid - target_centroid;
  // source_pre_tf(0,3) = -source_centroid(0);
  // source_pre_tf(1,3) = -source_centroid(1);
  // source_pre_tf(2,3) = -source_centroid(2);
  // target_pre_tf(0,3) = -target_centroid(0);
  // target_pre_tf(1,3) = -target_centroid(1);
  // target_pre_tf(2,3) = -target_centroid(2);
  // pcl::transformPointCloud(*cloud1, *cloud1, source_pre_tf);
  // pcl::transformPointCloud(*cloud2, *cloud2, target_pre_tf);

  pcl_tools::saveCloud(cloud1, "test_tf", 1);
  pcl_tools::saveCloud(cloud2, "test_tf", 2);

  bool bounded = false;
  bool high_precision = true;

  c1->cloud = cloud1;
  c2->cloud = cloud2;
  c1->updateSearchTree();
  c2->updateSearchTree();

  terrain_slam::Adjuster adj;
  Eigen::Matrix4d relative = adj.adjust(c1, c2, bounded, high_precision);

  // Transform pointcloud and save them for debugging
  std::cout << "Saving results. " << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_tf(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cloud2, *cloud2_tf, relative.cast<float>());
  pcl_tools::saveCloud(cloud1, "test", 1);
  pcl_tools::saveCloud(cloud2_tf, "test", 2);
  std::cout << "DONE!" << std::endl;
  return relative;
}

int main(int argc, char** argv) {
  // Create a fake set of 3D points
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::io::loadPCDFile (argv[1], *cloud1);// Load bun0.pcd -- should be available with the PCL archive in test
  pcl::io::loadPCDFile (argv[2], *cloud2);

  std::vector< int > dummy;
  pcl::removeNaNFromPointCloud(*cloud1, *cloud1, dummy);
  pcl::removeNaNFromPointCloud(*cloud2, *cloud2, dummy);

  registerClouds(cloud1, cloud2);

  return 0;
}
