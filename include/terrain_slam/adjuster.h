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


#ifndef ADJUSTER_H
#define ADJUSTER_H

#include <terrain_slam/clouds.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Geometry>
#include <boost/thread.hpp>

#include <boost/shared_ptr.hpp>

namespace terrain_slam {
class Adjuster {
public:
  Adjuster();
  ~Adjuster();
  void adjust(const boost::shared_ptr<CloudPatch> &cloud_fixed,
              const boost::shared_ptr<CloudPatch> &cloud);
  void reset();

protected:
  ceres::Problem* problem_;

  // Mutex to control the access to the adjuster.
  boost::mutex mutex_adjuster_;
};  // class

/**
 * @brief I am implementing the Iterative Closest Point Algorithm where the
 * residual is defined as follows:
 * r_ i = p_i - T * q_i
 * where T is the transformation to be estimated and p_i and q_i are the
 * corresponding points in the two point clouds based on the nearest neighbor
 * metric.
 */
class AdjusterCostFunctor {
public:
  AdjusterCostFunctor(const boost::shared_ptr<CloudPatch> &c,
                      const Eigen::Vector4d &p)
      : c_(c), p_(p) {
    // empty
  }

  bool operator()(double const* const _tx,
                  double const* const _ty,
                  double const* const _tz,
                  double const* const _roll,
                  double const* const _pitch,
                  double const* const _yaw,
                  double* residuals) const {
    // Copy the values
    double roll  = *_roll;
    double pitch = *_pitch;
    double yaw   = *_yaw;
    double tx    = *_tx;
    double ty    = *_ty;
    double tz    = *_tz;

    // std::cout << "Cost: " << tx << ", " << ty << ", " << tz << ", " << roll << ", " << pitch << ", " << yaw << ", " << std::endl;

    // Set the transformation matrix
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R = Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ());
    Eigen::Vector3d t(tx, ty, tz);
    Eigen::Matrix4d T;
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
    T(3,3) = 1.0;

    // Do the math
    Eigen::Matrix4Xd grid = c_->getGrid();
    Eigen::Vector4d p = p_;
    Eigen::Matrix4Xd tf_T = c_->tf();
    Eigen::Matrix4Xd inv_T = tf_T.inverse();
    Eigen::Vector4d pt = inv_T * T * p;

    // Find three closest points in cloud
    Eigen::VectorXd distances = (grid.colwise() - pt).colwise().squaredNorm();
    std::cout << "distances " << distances.transpose() << std::endl;
    int idx1, idx2, idx3;
    idx1 = 0;
    idx2 = 0;
    idx3 = 0;
    for (size_t i; i < distances.size(); i++) {
      if (distances(i) < distances(idx1)) {
        // std::cout << "distance " << distances(i) << std::endl;
        idx1 = i;
        idx2 = idx1;
        idx3 = idx2;
      }
    }

    // check that we have got three points
    if (idx2 < 0 || idx3 < 0) return false;

    Eigen::Vector3d p1(grid.col(idx1).hnormalized());
    Eigen::Vector3d p2(grid.col(idx2).hnormalized());
    Eigen::Vector3d p3(grid.col(idx3).hnormalized());

    // Calculate distance to plane
    Eigen::Vector3d v1 = p1 - p2;
    Eigen::Vector3d v2 = p1 - p3;
    Eigen::Vector3d w  = p1 - pt.hnormalized();
    Eigen::Vector3d n  = v1.cross(v2);
    residuals[0] = n.dot(w) / n.norm();
    residuals[1] = 0;
    residuals[2] = 0;

    return true;
  }

protected:
  boost::shared_ptr<CloudPatch> c_;
  Eigen::Vector4d p_;
};  // class
}   // namespace

#endif // ADJUSTER_H
