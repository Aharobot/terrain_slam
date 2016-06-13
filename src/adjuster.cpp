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

/**
 * @brief Default class constructor.
 */
terrain_slam::Adjuster::Adjuster() : problem_(0) {
  problem_ = new ceres::Problem();
  std::cout << "Adjuster constructor" << std::endl;
}

/**
 * @brief Default destructor.
 */
terrain_slam::Adjuster::~Adjuster() {
  boost::mutex::scoped_lock lock(mutex_adjuster_);
  delete problem_;
}

/**
 * @brief Restarts the Ceres problem
 */
void terrain_slam::Adjuster::reset() {
  boost::mutex::scoped_lock lock(mutex_adjuster_);
  if (problem_) {
      delete problem_;
  }
  problem_ = new ceres::Problem();
}

/**
 * @brief      Adjusts the positions of the graph.
 *
 * @param      cloud_fixed  The cloud fixed
 * @param      cloud        The cloud
 */
void terrain_slam::Adjuster::adjust(const boost::shared_ptr<CloudPatch> &cloud_fixed,
                                    const boost::shared_ptr<CloudPatch> &cloud) {
  boost::mutex::scoped_lock lock(mutex_adjuster_);

  std::cout << "Adjuster addConstraints" << std::endl;

  // Iterating for each point
  boost::progress_display show_progress(cloud->points.cols());
  double roll  = cloud->roll();
  double pitch = cloud->pitch();
  double yaw   = cloud->yaw();
  double tx = cloud->tx();
  double ty = cloud->ty();
  double tz = cloud->tz();
  std::cout << "T " << cloud->T << std::endl;
  std::cout << "Parameter block: " << tx << ", " << ty << ", " << tz << ", " << roll << ", " << pitch << ", " << yaw << ", " << std::endl;
  for (size_t i = 0; i < cloud->points.cols(); i++) {
    Eigen::Vector4d point = cloud->points.col(i);

    AdjusterCostFunctor *hcfunctor =
        new AdjusterCostFunctor(cloud_fixed, point);
    ceres::CostFunction *cost_function =
        new ceres::NumericDiffCostFunction<AdjusterCostFunctor, ceres::FORWARD, 3, 1, 1, 1, 1, 1, 1>(hcfunctor);
    problem_->AddResidualBlock(cost_function, NULL, &tx, &ty, &tz, &roll, &pitch, &yaw);
    ++show_progress;
  }

  std::cout << "Adjuster adjust" << std::endl;

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
  solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  solver_options.max_num_iterations = 50;
  solver_options.minimizer_progress_to_stdout = false;
  solver_options.num_threads = sysconf( _SC_NPROCESSORS_ONLN );
  solver_options.num_linear_solver_threads = sysconf( _SC_NPROCESSORS_ONLN );
  solver_options.initial_trust_region_radius = 1e14;
  solver_options.max_solver_time_in_seconds = 30;

  ceres::Solver::Summary sum;
  ceres::Solve(solver_options, problem_, &sum);
  std::cout << sum.FullReport() << "\n";
}
