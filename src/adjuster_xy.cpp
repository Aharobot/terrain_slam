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
#include <terrain_slam/eigen_tools.h>

#include <boost/progress.hpp>
#include <iostream>
#include <algorithm>
#include <vector>

////////////////////////////////////////////////////////////////////////////////

terrain_slam::AdjusterXY::AdjusterXY() : problem_(0) {
  problem_ = new ceres::Problem();
}

////////////////////////////////////////////////////////////////////////////////


terrain_slam::AdjusterXY::~AdjusterXY() {
  boost::mutex::scoped_lock lock(mutex_adjuster_);
  delete problem_;
}

////////////////////////////////////////////////////////////////////////////////

void terrain_slam::AdjusterXY::reset() {
  boost::mutex::scoped_lock lock(mutex_adjuster_);
  if (problem_) {
      delete problem_;
  }
  problem_ = new ceres::Problem();
}

////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix4d
terrain_slam::AdjusterXY::adjust(const boost::shared_ptr<CloudPatch> &cloud_fixed,
                               const boost::shared_ptr<CloudPatch> &cloud,
                               bool bounded,
                               bool high_precision) {
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

    AdjusterXYCostFunctor *hcfunctor =
        new AdjusterXYCostFunctor(cloud_fixed, point, roll, pitch, yaw);
    ceres::CostFunction *cost_function =
        new ceres::NumericDiffCostFunction<AdjusterXYCostFunctor, ceres::CENTRAL, 1, 1, 1>(hcfunctor);
    problem_->AddResidualBlock(cost_function, NULL, &tx, &ty);

    // new ceres::HuberLoss(0.5)
  }

  // Performing the optimization
  ceres::Solver::Options solver_options;

  // Local Optimization
  solver_options.minimizer_type = ceres::TRUST_REGION;

  solver_options.linear_solver_type = ceres::DENSE_QR; //SPARSE_NORMAL_CHOLESKY;
  solver_options.max_num_iterations = 5000;
  solver_options.minimizer_progress_to_stdout = true;
  solver_options.num_threads = sysconf(_SC_NPROCESSORS_ONLN);
  solver_options.num_linear_solver_threads = sysconf(_SC_NPROCESSORS_ONLN);

  solver_options.initial_trust_region_radius = solver_options.max_trust_region_radius; // 4.0;
  solver_options.max_solver_time_in_seconds = 600;

  if (high_precision) {
    solver_options.parameter_tolerance = 1e-14;
    solver_options.function_tolerance  = 1e-14;  // default 1e-6
    solver_options.gradient_tolerance  = 1e-8;
  } else {
    solver_options.parameter_tolerance = 1e-8;
    solver_options.function_tolerance  = 1e-8;  // default 1e-6
    solver_options.gradient_tolerance  = 1e-8;
  }

  solver_options.minimizer_progress_to_stdout = true;
  solver_options.use_nonmonotonic_steps = true;

  ceres::Solver::Summary sum;
  ceres::Solve(solver_options, problem_, &sum);

  std::cout << sum.FullReport() << "\n";

  bool convergence = false;
  if (sum.termination_type == ceres::CONVERGENCE) convergence = true;

  std::cout << "Before: (" << relative.x() << ", " << relative.y() << ", " << relative.z() << "); (" << relative.roll() << ", " << relative.pitch() << ", " << relative.yaw() << ")" << std::endl;
  std::cout << "After:  (" << tx << ", " << ty << ", " << tz << "); (" << roll << ", " << pitch << ", " << yaw << ")" << std::endl;
  std::cout << "FINAL COST: " << sum.final_cost/(double)cloud->size() << std::endl;
  std::cout << "CONVERGENCE? " << convergence << std::endl;

  // std::string path("../adjusted");
  // std::ostringstream suffix;
  // suffix << "_" << cloud_fixed->getId() << "_" << cloud->getId();
  // cloud_fixed->save(cloud_fixed->getId(), path, suffix.str(), false, true);

  // cloud->setTransform(cloud_fixed->T * new_tf.T);
  // cloud->save(cloud->getId(), path, suffix.str(), false, true);

  return eigen_tools::buildTransform(tx, ty, tz, roll, pitch, yaw);
}
