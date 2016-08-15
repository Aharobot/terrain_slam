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

#include <terrain_slam/adjuster.h>
#include <terrain_slam/clouds.h>

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <iostream>
#include <fstream>
#include <random>

void fillData(int num_points, double overlap, boost::shared_ptr<terrain_slam::CloudPatch>& c1, boost::shared_ptr<terrain_slam::CloudPatch>& c2) {
  std::random_device rd;
  boost::random::mt19937 gen(rd());
  boost::random::uniform_real_distribution<> dis(-1, +1);
  int inliers = num_points*overlap;
  // std::cout << "Generating " << inliers << " inliers out of " << num_points << std::endl;
  int outliers = num_points - inliers;
  for (size_t i = 0; i < num_points; ++i) {
    Eigen::Vector4d v;
    v(0) = dis(gen)*6.0;
    v(1) = dis(gen)*6.0;
    v(2) = dis(gen)*2.0;
    v(3) = 1.0;
    c1->add(v);
    // std::cout << "Creating point at (" << v(0) << ", " << v(1) << ", " << v(2) << ")" << std::endl;
  }

  for (size_t i = 0; i < num_points; ++i) {
    Eigen::Vector4d v;

    if (i < inliers) {
      v = c1->at(i);
      v(0) -= 0.7;
      // add noise
      v(0) += dis(gen)*0.2 / (RAND_MAX + 1.0);
      v(1) += dis(gen)*0.2 / (RAND_MAX + 1.0);
      v(1) += dis(gen)*0.2 / (RAND_MAX + 1.0);
    } else {
      v(0) = dis(gen)*6.0;
      v(1) = dis(gen)*6.0;
      v(2) = dis(gen)*2.0;
      v(3) = 1.0;
    }
    c2->add(v);
    // std::cout << "Creating point at (" << v(0) << ", " << v(1) << ", " << v(2) << ")" << std::endl;
  }
}

int main(int argc, char** argv) {
  // Create a fake set of 3D points
  boost::shared_ptr<terrain_slam::CloudPatch> c1(new terrain_slam::CloudPatch());
  boost::shared_ptr<terrain_slam::CloudPatch> c2(new terrain_slam::CloudPatch());
  std::cout << "Filling cloud data... " << std::endl;

  double overlap = 0;
  double x = 0;
  double y = 0;
  double z = 0;

  std::vector<std::vector<double> > output(500000, std::vector<double>(6));

  int xM = 10;
  int iM = 50;

  std::ofstream myfile("example.csv");
  if (!myfile.is_open()) {
    std::cout << "Can't open file " << std::endl;
    return 0;
  }

  #pragma omp parallel for collapse(5)
  for (size_t o = 0; o < xM; o++) {
    for (size_t xi = 0; xi < xM; xi++) {
      for (size_t yi = 0; yi < xM; yi++) {
        for (size_t zi = 0; zi < xM; zi++) {
          for (size_t i = 0; i < iM; i++) {
            overlap = static_cast<double>(o)/10.0;
            x = 0.5*static_cast<double>(xi);
            y = 0.5*static_cast<double>(yi);
            z = 0.5*static_cast<double>(zi);
            boost::shared_ptr<terrain_slam::CloudPatch> c1(new terrain_slam::CloudPatch());
            boost::shared_ptr<terrain_slam::CloudPatch> c2(new terrain_slam::CloudPatch());
            fillData(100, overlap, c1, c2);
            // c1->save(0, std::string("../adjusted/original"), std::string(""));
            // c2->save(1, std::string("../adjusted/original"), std::string(""));
            c2->transform(0,3) = x;
            c2->transform(1,3) = y;
            c2->transform(2,3) = z;
            c1->setId(100*i+0);
            c2->setId(100*i+1);
            terrain_slam::Adjuster adj;
            // std::cout << "Adjusting... " << std::endl;
            Eigen::Matrix4d new_tf = adj.adjust(c1, c2);
            double dx = std::abs(new_tf(0, 3)) - 0.7;
            double dy = std::abs(new_tf(1, 3));
            double dz = std::abs(new_tf(2, 3));
            double error = sqrt(dx*dx+dy*dy+dz*dz);
            // std::cout << "Error: " << error << std::endl;
            myfile << overlap << "," << x << "," << y << "," << z << "," << i << "," << error << "\n";
          }
        }
      }
    }
  }
  myfile.close();
  return 0;
}
