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
#include <terrain_slam/eigen_tools.h>

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

Eigen::Matrix4d
terrain_slam::Adjuster::adjust(const boost::shared_ptr<CloudPatch> &cloud_fixed,
                               const boost::shared_ptr<CloudPatch> &cloud) {
  boost::mutex::scoped_lock lock(mutex_adjuster_);

  // Iterating for each point
  Eigen::Matrix4d fti = cloud_fixed->transform().inverse();
  Eigen::Matrix4d t = cloud->transform();
  Transform relative(fti*t);
  double roll  = relative.roll();
  double pitch = relative.pitch();
  double yaw   = relative.yaw();
  double tx = relative.x();
  double ty = relative.y();
  double tz = relative.z();

  // std::cout << "T " << cloud->T << std::endl;
  // std::cout << "Parameter block: " << tx << ", " << ty << ", " << tz << ", " << roll << ", " << pitch << ", " << yaw << ", " << std::endl;

  for (size_t i = 0; i < cloud->size(); i++) {
    Eigen::Vector4d point = cloud->at(i);

    AdjusterCostFunctor *hcfunctor =
        new AdjusterCostFunctor(cloud_fixed, point, roll, pitch);
    ceres::CostFunction *cost_function =
        new ceres::NumericDiffCostFunction<AdjusterCostFunctor, ceres::CENTRAL, 3, 1, 1, 1, 1>(hcfunctor);
    problem_->AddResidualBlock(cost_function, new ceres::HuberLoss(0.5), &tx, &ty, &tz, &yaw);
  }

  // Add constrains
  problem_->SetParameterLowerBound(&tx, 0, tx - 5.0);
  problem_->SetParameterUpperBound(&tx, 0, tx + 5.0);
  problem_->SetParameterLowerBound(&ty, 0, ty - 5.0);
  problem_->SetParameterUpperBound(&ty, 0, ty + 5.0);
  problem_->SetParameterLowerBound(&tz, 0, tz - 0.5);
  problem_->SetParameterUpperBound(&tz, 0, tz + 0.5);
  problem_->SetParameterLowerBound(&yaw, 0, yaw - 0.2);
  problem_->SetParameterUpperBound(&yaw, 0, yaw + 0.2);

  // Performing the optimization
  ceres::Solver::Options solver_options;

  // Local Optimization
  solver_options.linear_solver_type = ceres::DENSE_QR;
  solver_options.max_num_iterations = 100;
  solver_options.minimizer_progress_to_stdout = true;
  solver_options.num_threads = sysconf(_SC_NPROCESSORS_ONLN);
  solver_options.num_linear_solver_threads = sysconf(_SC_NPROCESSORS_ONLN);

  solver_options.initial_trust_region_radius = solver_options.max_trust_region_radius; // 4.0;
  solver_options.max_solver_time_in_seconds = 60;

  solver_options.parameter_tolerance = 1e-8;
  solver_options.function_tolerance  = 1e-6;  // default 1e-6
  solver_options.gradient_tolerance  = 1e-8;
  solver_options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary sum;
  ceres::Solve(solver_options, problem_, &sum);
  std::cout << sum.FullReport() << "\n";

  std::cout << "Before: (" << relative.x() << ", " << relative.y() << ", " << relative.z() << "); (" << relative.roll() << ", " << relative.pitch() << ", " << relative.yaw() << ")" << std::endl;
  std::cout << "After:  (" << tx << ", " << ty << ", " << tz << "); (" << roll << ", " << pitch << ", " << yaw << ")" << std::endl;


  // std::string path("../adjusted");
  // std::ostringstream suffix;
  // suffix << "_" << cloud_fixed->getId() << "_" << cloud->getId();
  // cloud_fixed->save(cloud_fixed->getId(), path, suffix.str(), false, true);

  // cloud->setTransform(cloud_fixed->T * new_tf.T);
  // cloud->save(cloud->getId(), path, suffix.str(), false, true);

  return eigen_tools::buildTransform(tx, ty, tz, roll, pitch, yaw);
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

Eigen::Matrix4d
terrain_slam::BruteForceAdjuster::adjust(
    const boost::shared_ptr<CloudPatch> &cloud_fixed,
    const boost::shared_ptr<CloudPatch> &cloud) {

  int x_steps = (x_max_ - x_min_) / xy_resolution_;
  int y_steps = (y_max_ - y_min_) / xy_resolution_;
  int z_steps = (z_max_ - z_min_) / z_resolution_;
  int yaw_steps = (yaw_max_ - yaw_min_) / yaw_resolution_;

  if (yaw_steps == 0) yaw_steps = 1;
  if (z_steps == 0) z_steps = 1;

  std::vector<double> cost(x_steps*y_steps*z_steps*yaw_steps);
  std::vector<Eigen::Matrix4d> tfs(x_steps*y_steps*z_steps*yaw_steps);

  // Get relative transform
  Eigen::Matrix4d fti = cloud_fixed->transform().inverse();
  Eigen::Matrix4d t = cloud->transform();
  Eigen::Matrix4d relative = fti*t;

  boost::progress_display show_progress(x_steps*y_steps*z_steps*yaw_steps);

  #pragma omp parallel for collapse(4)
  for (size_t xi = 0; xi < x_steps; xi++) {
    for (size_t yi = 0; yi < y_steps; yi++) {
      for (size_t zi = 0; zi < z_steps; zi++) {
        for (size_t yawi = 0; yawi < yaw_steps; yawi++) {
          double x = x_min_ + xy_resolution_*xi;
          double y = y_min_ + xy_resolution_*yi;
          double z = z_min_ + z_resolution_*zi;
          double yaw = yaw_min_ + yaw_resolution_*yawi;
          Eigen::Matrix4d tf = eigen_tools::buildTransform(x, y, z, 0, 0, yaw);
          tf = relative * tf;
          cost.at(yawi + yaw_steps*(zi + z_steps*(yi + y_steps*xi))) =
            computeCost(cloud_fixed, cloud, tf);
          tfs.at(yawi + yaw_steps*(zi + z_steps*(yi + y_steps*xi))) = tf;
          ++show_progress;
        }
      }
    }
  }

  std::cout << "COST: " << std::endl;
  for (size_t i = 0; i < cost.size(); i++)
    std::cout << cost[i] << ", ";
  std::cout << std::endl;

  // TODO
  std::vector<double>::iterator result = std::min_element(cost.begin(),
                                                          cost.end());
  std::cout << "min element at: " << std::distance(cost.begin(), result);
  std::cout << "Transform: \n" << tfs[std::distance(cost.begin(), result)] << std::endl;

  return tfs[std::distance(cost.begin(), result)];
}

double terrain_slam::BruteForceAdjuster::computeCost(
    const boost::shared_ptr<CloudPatch> &cloud_fixed,
    const boost::shared_ptr<CloudPatch> &cloud,
    const Eigen::Matrix4d& tf) {
  // Init cost
  double cost = 0;
  int npoints = 0;
  // For each point
  for (size_t i = 0; i < cloud->size(); i++) {
    // Transform point
    Eigen::Vector4d pt = tf * cloud->at(i);

    // Find three closest points in cloud
    std::vector<Eigen::Vector4d> nn = cloud_fixed->kNN(pt, 3);
    Eigen::Vector4d p1(nn.at(0));
    Eigen::Vector4d p2(nn.at(1));
    Eigen::Vector4d p3(nn.at(2));

    // Calculate distance to point
    Eigen::Vector4d v1 = p1 - pt;
    Eigen::Vector4d v2 = p2 - pt;
    Eigen::Vector4d v3 = p3 - pt;
    double xd = v1(0)*v1(0) + v2(0)*v2(0) + v3(0)*v3(0);
    double yd = v1(1)*v1(1) + v2(1)*v2(1) + v3(1)*v3(1);
    double zd = v1(2)*v1(2) + v2(2)*v2(2) + v3(2)*v3(2);

    // Decrease cost if points are away from camera
    double mean_z_dist = (p1(2) + p2(2) + p3(2) + pt(2))*0.25;

    // Account for points in the grid
    double res = 0.6;

    // Cost to add
    double added_cost = 0;

    if (xd < res && yd < res) {
      if (zd > 0.5) {
        added_cost += zd*2.0;
      }else if (zd <= 0.5 && zd > 0.2) {
        added_cost += zd*1.5;
      } else {
        added_cost += zd;
      }
      npoints++;
    }
    added_cost /= mean_z_dist;
    cost += added_cost;
  }
  if (cost == 0) cost = 1e90;
  if (npoints <= cloud->size()/4) {
    cost = 1e50;
  } else if (npoints > cloud->size()/4) {
    cost *= static_cast<double>(cloud->size()/npoints);
  }
  return cost;
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