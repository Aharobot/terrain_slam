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

#include <terrain_slam/gridder.h>

Eigen::Matrix4Xd terrain_slam::Gridder::grid() {
  if (!init_) {
    std::cerr << "[ERROR (Gridder)]: Input pointcloud not set" << std::endl;
  }

  double min_x, min_y, max_x, max_y;
  bool ok = getMinMax(min_x, min_y, max_x, max_y);
  if (!ok) {
    std::cout << "[ERROR (Gridder)]: getMinMax did not work" << std::endl;
  }

  // std::cout << "Limits from (" << min_x << ", " << min_y << ") to ("
  //                             << max_x << ", " << max_y << ") "
  //                             << std::endl;

  // Round to nearest divisible by resolution
  min_x = resolution_ * static_cast<int>((min_x - resolution_/2) / resolution_);
  min_y = resolution_ * static_cast<int>((min_y - resolution_/2) / resolution_);
  max_x = resolution_ * static_cast<int>((max_x + resolution_/2) / resolution_);
  max_y = resolution_ * static_cast<int>((max_y + resolution_/2) / resolution_);

  double size_x = max_x - min_x;
  double size_y = max_y - min_y;
  int grid_size_x = static_cast<int>(size_x / resolution_);
  int grid_size_y = static_cast<int>(size_y / resolution_);
  int points_size = grid_size_x * grid_size_y;

  // std::cout << "Range from (" << min_x << ", " << min_y << ") to ("
  //                             << max_x << ", " << max_y << ") "
  //                             << "with a resolution of " << resolution_
  //                             << " size of " << grid_size_x << "x"
  //                             << grid_size_y << std::endl;

  Eigen::Matrix3Xd m = Eigen::Matrix3Xd::Zero(3, points_size);

  // resample points to resolution
  for (size_t i = 0; i < grid_size_x; i++) {
    double x_coord = min_x + resolution_*i;
    for (size_t j = 0; j < grid_size_y; j++) {
      double y_coord = min_y + resolution_*j;
      Eigen::Vector3d p = reduce(x_coord, y_coord, search_radius_);
      m.col(i*grid_size_y + j) = p;
    }
  }
  // std::cout << "m.col(0) " << m.col(0).transpose() << std::endl;

  // // Remove zero columns
  // for (size_t i = 0; i < m.cols(); i++) {
  //   if (m.col(i).sum() == 0) {
  //     // Remove current row
  //     m.block(i, 0, m.rows()-i, m.cols()) = m.block(i + 1 , 0, m.rows()-i, m.cols());
  //     m.conservativeResize(m.rows(), m.cols());
  //   }
  // }
  // std::cout << "m.col(0) " << m.col(0).transpose() << std::endl;
  return m.colwise().homogeneous();
}

////////////////////////////////////////////////////////////////////////////////

bool terrain_slam::Gridder::getMinMax(double &min_x, double &min_y,
                                      double &max_x, double &max_y) {
  min_x = points_(0, 0);
  min_y = points_(0, 1);
  max_x = points_(0, 0);
  max_y = points_(0, 1);

  for (size_t i = 0; i < points_.cols(); i++) {
    Eigen::Vector4d v = points_.col(i);
    double vx = v(0);
    double vy = v(1);
    if (vx > max_x) {
      max_x = vx;
    } else if (vx < min_x) {
      min_x = vx;
    }

    if (vy > max_y) {
      max_y = vy;
    } else if (vy < min_y) {
      min_y = vy;
    }
  }
  if ((min_x != max_x) && (min_y != max_y)) {
    return true;
  } else {
    return false;
  }
}

////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d terrain_slam::Gridder::reduce(const double &x,
                                              const double &y,
                                              const double &distance) {
  // 2D query point
  Eigen::Vector2d c(x, y);
  // 2D point cloud
  Eigen::Matrix2Xd p_xy = points_.topRows(2);
  // Compute distance from c to all points
  Eigen::VectorXd sq_d = (p_xy.colwise() - c).colwise().squaredNorm();

  if (sq_d.size() < 3) {
    std::cout << "[WARN (Gridder)]: There are not enough points from ("
              << x << ", " << y << ") within distance: "
              << distance << std::endl;
    return Eigen::Vector3d(0, 0, 0);
  }

  // Use weighted mean to compute Z
  // weight is inversely proportional to distance
  // \f$ \overline{z} = \sum_i \frac{z_i\cdot d^{-1}_i}{d^{-1}_i} \f$
  double z = 0;
  double d = 0;
  int cnt = 0;
  for (size_t i = 0; i < sq_d.size(); i++) {
    if (sq_d(i) < distance) {
      z += points_(2, i)/sq_d(i);
      d += 1/sq_d(i);
      cnt++;
    }
  }

  // TODO get rid of extrapolated points

  if (d == 0 || cnt < 3) {
    // std::cout << "[WARN]: Distance is zero! " << cnt << " points visited" << std::endl;
    return Eigen::Vector3d(0, 0, 0);
  } else {
    z /= d;
    return Eigen::Vector3d(x, y, z);
  }
}