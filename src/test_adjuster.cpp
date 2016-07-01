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

void fillData(int num_points, boost::shared_ptr<terrain_slam::CloudPatch>& c1, boost::shared_ptr<terrain_slam::CloudPatch>& c2) {
  c1->points.resize(4, num_points);
  c2->points.resize(4, num_points);
  for (size_t i = 0; i < num_points; ++i)
  {
    Eigen::Vector4d v;
    v(0) = 1024 * rand() / (RAND_MAX + 1.0);
    v(1) = 1024 * rand() / (RAND_MAX + 1.0);
    v(2) = 1024 * rand() / (RAND_MAX + 1.0);
    v(3) = 1.0;
    c1->points.col(i) = v;
    std::cout << "Creating point at (" << v(0) << ", " << v(1) << ", " << v(2) << ")" << std::endl;
  }

  for (size_t i = 0; i < num_points; ++i) {
    Eigen::Vector4d v = c1->points.col(i);
    v(0) -= 0.7;
    // add noise
    v(0) += rand()*0.2 / (RAND_MAX + 1.0);
    v(1) += rand()*0.2 / (RAND_MAX + 1.0);
    v(1) += rand()*0.2 / (RAND_MAX + 1.0);
    c2->points.col(i) = v;
    std::cout << "Creating point at (" << v(0) << ", " << v(1) << ", " << v(2) << ")" << std::endl;
  }
}

int main(int argc, char** argv) {
  // Create a fake set of 3D points
  boost::shared_ptr<terrain_slam::CloudPatch> c1(new terrain_slam::CloudPatch());
  boost::shared_ptr<terrain_slam::CloudPatch> c2(new terrain_slam::CloudPatch());
  std::cout << "Filling cloud data... " << std::endl;
  fillData(10, c1, c2);
  c1->save(0, std::string("../adjusted/original"), std::string(""));
  c2->save(1, std::string("../adjusted/original"), std::string(""));
  c1->copy2Grid();
  c2->copy2Grid();
  c2->T(0,3) = 0;
  c2->T(1,3) = 0;
  c1->setId(0);
  c2->setId(1);
  terrain_slam::Adjuster adj;
  std::cout << "Adjusting... " << std::endl;
  adj.adjust(c1, c2);
  return 0;
}
