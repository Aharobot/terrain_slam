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
#include <omp.h>

terrain_slam::Gridder::Gridder()
    : init_(false),
      resolution_(0.2),
      search_radius_(0.4),
      ransac_max_iterations_(30),
      ransac_num_params_(15) {
  // Nothing
}


terrain_slam::Gridder::Gridder(double resolution, double search_radius) {
  ransac_max_iterations_ = 30;
  ransac_num_params_ = 15;
  setResolution(resolution);
  setSearchRadius(search_radius);
}


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
  int num_points = 0;
  for (size_t i = 0; i < grid_size_x; i++) {
    double x_coord = min_x + resolution_*i;
    for (size_t j = 0; j < grid_size_y; j++) {
      double y_coord = min_y + resolution_*j;
      Eigen::Vector3d p;
      bool success = reduce(x_coord, y_coord, search_radius_, p);
      if (success) {
        //m.col(i*grid_size_y + j) = p;
        m.col(num_points) = p;
        num_points++;
      }
    }
  }
  // std::cout << "m.col(0) " << m.col(0).transpose() << std::endl;

  // Remove zero columns
  m.conservativeResize(3, num_points);

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

bool terrain_slam::Gridder::reduce(const double &x,
                                   const double &y,
                                   const double &distance,
                                   Eigen::Vector3d& reduced_point) {
  reduced_point = Eigen::Vector3d(0, 0, 0);
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
    return false;
  }

  // Use weighted mean to compute Z
  // weight is inversely proportional to distance
  // \f$ \overline{z} = \sum_i \frac{z_i\cdot d^{-1}_i}{d^{-1}_i} \f$
  double z = 0;
  double d = 0;
  int cnt  = 0;
  std::vector<Eigen::Vector3d> samples;
  for (size_t i = 0; i < sq_d.size(); i++) {
    if (sq_d(i) < distance) {
      z += points_(2, i)/sq_d(i);
      d += 1/sq_d(i);
      cnt++;
      samples.push_back(points_.col(i).hnormalized());
    }
  }

  if (samples.size() < ransac_num_params_) return false;

  // std::cout << "Samples: " << samples.size() << std::endl;
  // std::cout << "sq_d: " << sq_d.size() << std::endl;

  // tests
  std::vector<std::vector<Eigen::Vector3d> > inliers;

  // RANSAC
  // std::cout << "ransac_max_iterations_ " << ransac_max_iterations_ << std::endl;
  for (int i = 0; i < ransac_max_iterations_; i++) {
    // std::cout << "Iteration " << i << std::endl;

    // Select random samples to avoid picking the same element more than once
    std::vector<Eigen::Vector3d> remainder_samples(samples);
    std::vector<Eigen::Vector3d> random_samples;
    std::random_shuffle(remainder_samples.begin(), remainder_samples.end());
    for (size_t k = 0; k < ransac_num_params_; k++) {
      random_samples.push_back(remainder_samples.back());
      remainder_samples.pop_back();
    }

    // Get the model
    Eigen::Vector3d mean(0, 0, 0);
    std::vector<Eigen::Vector3d> in;
    for (size_t k = 0; k < random_samples.size(); k++) {
      double xs = random_samples[k](0);
      double ys = random_samples[k](1);
      double zs = random_samples[k](2);
      mean += Eigen::Vector3d(xs, ys, zs);
      in.push_back(random_samples[k]);
    }
    mean /= random_samples.size();

    // Test the model against the remainder samples
    for (size_t k = 0; k < remainder_samples.size(); k++) {
      double mean_z = mean(2);
      double zs = remainder_samples[k](2);
      if (abs(mean_z - zs) < resolution_) {
        in.push_back(remainder_samples[k]);
      }
    }
    inliers.push_back(in);
  }

  // std::cout << "inliers.size() " << inliers.size() << std::endl;

  int best_model = inliers[0].size();
  int best_model_idx = 0;

  // Get the best model and recalculate Z from all inliers
  for (int i = 0; i < ransac_max_iterations_; i++) {
    // std::cout << "inliers[" << i << "].size()  " << inliers[i].size() << " vs " << best_model << " model" << std::endl;
    if (best_model < inliers[i].size()) {
      // std::cout << "New best model" << std::endl;
      best_model = inliers[i].size();
      best_model_idx = i;
    }
  }

  // Recalculate Z with inliers
  double mean_z = 0;
  for (size_t k = 0; k < inliers[best_model_idx].size(); k++) {
    mean_z += inliers[best_model_idx][k](2);
  }
  mean_z /= inliers[best_model_idx].size();


  // TODO get rid of extrapolated points

  if (d == 0 || cnt < 3) {
    // std::cout << "[WARN]: Distance is zero! " << cnt << " points visited" << std::endl;
    return false;
  } else {
    // z /= d;
    reduced_point = Eigen::Vector3d(x, y, mean_z);
    return true;
  }
}