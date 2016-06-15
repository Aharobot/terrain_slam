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
#include <boost/progress.hpp>
#include <iostream>
#include <algorithm>
#include <vector>

////////////////////////////////////////////////////////////////////////////////

terrain_slam::Adjuster::Adjuster() : problem_(0) {
  problem_ = new ceres::Problem();
}

////////////////////////////////////////////////////////////////////////////////


terrain_slam::Adjuster::~Adjuster() {
  boost::mutex::scoped_lock lock(mutex_adjuster_);
  delete problem_;
}

////////////////////////////////////////////////////////////////////////////////

void terrain_slam::Adjuster::reset() {
  boost::mutex::scoped_lock lock(mutex_adjuster_);
  if (problem_) {
      delete problem_;
  }
  problem_ = new ceres::Problem();
}

////////////////////////////////////////////////////////////////////////////////

void terrain_slam::Adjuster::adjust(const boost::shared_ptr<CloudPatch> &cloud_fixed,
                                    const boost::shared_ptr<CloudPatch> &cloud) {
  boost::mutex::scoped_lock lock(mutex_adjuster_);

  // Iterating for each point
  Transform relative(cloud_fixed->T.inverse()*cloud->T);
  double roll  = relative.roll();
  double pitch = relative.pitch();
  double yaw   = relative.yaw();
  double tx = relative.tx();
  double ty = relative.ty();
  double tz = relative.tz();

  // std::cout << "T " << cloud->T << std::endl;
  // std::cout << "Parameter block: " << tx << ", " << ty << ", " << tz << ", " << roll << ", " << pitch << ", " << yaw << ", " << std::endl;
  for (size_t i = 0; i < cloud->points.cols(); i++) {
    Eigen::Vector4d point = cloud->points.col(i);

    AdjusterCostFunctor *hcfunctor =
        new AdjusterCostFunctor(cloud_fixed, point, roll, pitch);
    ceres::CostFunction *cost_function =
        new ceres::NumericDiffCostFunction<AdjusterCostFunctor, ceres::FORWARD, 1, 1, 1, 1, 1>(hcfunctor);
    problem_->AddResidualBlock(cost_function, NULL, &tx, &ty, &tz, &yaw);
  }

  // Add constrains
  problem_->SetParameterLowerBound(&tx, 0, tx - 5.0);
  problem_->SetParameterUpperBound(&tx, 0, tx + 5.0);
  problem_->SetParameterLowerBound(&ty, 0, ty - 5.0);
  problem_->SetParameterUpperBound(&ty, 0, ty + 5.0);
  problem_->SetParameterLowerBound(&tz, 0, tz - 5.0);
  problem_->SetParameterUpperBound(&tz, 0, tz + 5.0);
  // problem_->SetParameterLowerBound(&roll, 0, roll - 0.1);
  // problem_->SetParameterUpperBound(&roll, 0, roll + 0.1);
  // problem_->SetParameterLowerBound(&pitch, 0, pitch - 0.1);
  // problem_->SetParameterUpperBound(&pitch, 0, pitch + 0.1);
  problem_->SetParameterLowerBound(&yaw, 0, yaw - 0.2);
  problem_->SetParameterUpperBound(&yaw, 0, yaw + 0.2);

  // Performing the optimization
  ceres::Solver::Options solver_options;

  // Global Optimization
  // solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  // solver_options.max_num_iterations = 1000;
  // solver_options.minimizer_progress_to_stdout = false;
  // solver_options.num_threads = sysconf( _SC_NPROCESSORS_ONLN );
  // solver_options.num_linear_solver_threads = sysconf( _SC_NPROCESSORS_ONLN );
  // solver_options.initial_trust_region_radius = 1e14;
  // solver_options.max_solver_time_in_seconds = 600;

  // Local Optimization
  solver_options.linear_solver_type = ceres::DENSE_QR;
  solver_options.max_num_iterations = 1000;
  solver_options.minimizer_progress_to_stdout = true;
  solver_options.num_threads = sysconf(_SC_NPROCESSORS_ONLN);
  solver_options.num_linear_solver_threads = sysconf(_SC_NPROCESSORS_ONLN);

  solver_options.initial_trust_region_radius = 4.0; // 1e14;
  solver_options.max_solver_time_in_seconds = 600;

  solver_options.parameter_tolerance = 1e-18;
  solver_options.function_tolerance  = 1e-18;  // default 1e-6
  solver_options.gradient_tolerance  = 1e-18;

  ceres::Solver::Summary sum;
  ceres::Solve(solver_options, problem_, &sum);
  std::cout << sum.FullReport() << "\n";

  std::cout << "Before: " << relative.tx() << ", " << relative.ty() << ", " << relative.tz() << ", " << relative.roll() << ", " << relative.pitch() << ", " << relative.yaw() << ", " << std::endl;
  std::cout << "After:  " << tx << ", " << ty << ", " << tz << ", " << roll << ", " << pitch << ", " << yaw << ", " << std::endl;

  // save
  Transform new_tf(tx, ty, tz, roll, pitch, yaw);

  std::string path("../adjusted");
  std::ostringstream suffix;
  suffix << "_" << cloud_fixed->getId() << "_" << cloud->getId();
  cloud_fixed->save(cloud_fixed->getId(), path, suffix.str());

  cloud->setTransform(cloud_fixed->T * new_tf.T);
  cloud->save(cloud->getId(), path, suffix.str());

  std::cout << "Before: \n" << relative.T << std::endl;
  std::cout << "After: \n" << new_tf.T << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

terrain_slam::BruteForceAdjuster::BruteForceAdjuster(double x_min,
                                                     double x_max,
                                                     double y_min,
                                                     double y_max,
                                                     double z_min,
                                                     double z_max,
                                                     double yaw_min,
                                                     double yaw_max,
                                                     double xy_resolution,
                                                     double z_resolution,
                                                     double yaw_resolution) {
  x_min_ = x_min;
  x_max_ = x_max;
  y_min_ = y_min;
  y_max_ = y_max;
  z_min_ = z_min;
  z_max_ = z_max;
  yaw_min_ = yaw_min;
  yaw_max_ = yaw_max;
  xy_resolution_ = xy_resolution;
  z_resolution_ = z_resolution;
  yaw_resolution_ = yaw_resolution;
}

void
terrain_slam::BruteForceAdjuster::adjust(
    const boost::shared_ptr<CloudPatch> &cloud_fixed,
    const boost::shared_ptr<CloudPatch> &cloud) {

  int x_steps = (x_max_ - x_min_) / xy_resolution_;
  int y_steps = (y_max_ - y_min_) / xy_resolution_;
  int z_steps = (z_max_ - z_min_) / z_resolution_;
  int yaw_steps = (yaw_max_ - yaw_min_) / yaw_resolution_;

  std::vector<double> cost(x_steps*y_steps*z_steps*yaw_steps);

  boost::progress_display show_progress(x_steps*y_steps*z_steps*yaw_steps);

  for (size_t xi = 0; xi < x_steps; xi++) {
    double x = x_min_ + xy_resolution_*xi;
    for (size_t yi = 0; yi < y_steps; yi++) {
      double y = y_min_ + xy_resolution_*yi;
      for (size_t zi = 0; zi < z_steps; zi++) {
        double z = z_min_ + z_resolution_*zi;
        for (size_t yawi = 0; yawi < yaw_steps; yawi++) {
          double yaw = yaw_min_ + yaw_resolution_*yawi;
          Transform tf(x, y, z, 0, 0, yaw);
          cost.at(yawi + yaw_steps*(zi + z_steps*(yi + y_steps*xi))) =
            computeCost(cloud_fixed, cloud, tf);
          ++show_progress;
        }
      }
    }
  }

  // TODO
  std::vector<double>::iterator result = std::min_element(cost.begin(),
                                                          cost.end());
  std::cout << "min element at: " << std::distance(cost.begin(), result);
}

double terrain_slam::BruteForceAdjuster::computeCost(
    const boost::shared_ptr<CloudPatch> &cloud_fixed,
    const boost::shared_ptr<CloudPatch> &cloud,
    const Transform& tf) {
  // Init cost
  double cost = 0;
  // Get relative transform
  Eigen::Matrix4d relative = cloud_fixed->T.inverse() * cloud->T;
  // For each point
  for (size_t i = 0; i < cloud->size(); i++) {
    // Transform point
    Eigen::Vector4d pt = relative * tf.T * cloud->point(i);
    // Find three closest points in cloud
    std::vector<Eigen::Vector4d> nn = cloud_fixed->kNN(pt, 3);

    Eigen::Vector3d p1(nn.at(0).hnormalized());
    Eigen::Vector3d p2(nn.at(1).hnormalized());
    Eigen::Vector3d p3(nn.at(2).hnormalized());

    // Calculate distance to plane
    Eigen::Vector3d v1 = p1 - p2;
    Eigen::Vector3d v2 = p1 - p3;
    Eigen::Vector3d w  = p1 - pt.hnormalized();
    Eigen::Vector3d n  = v1.cross(v2);
    cost += n.dot(w) / n.norm();
  }
}

// void
// terrain_slam::BruteForceAdjuster::copy2PM(
//     const boost::shared_ptr<CloudPatch> &cp,
//     const boost::shared_ptr<DP>& dp) {

//   std::vector<std::string> labels;
//   labels.push_back(std::string("x"));
//   labels.push_back(std::string("y"));
//   labels.push_back(std::string("z"));
//   dp.reset(new DP(cp->points, labels));
// }