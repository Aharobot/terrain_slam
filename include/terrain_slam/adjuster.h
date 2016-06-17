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

#include <pointmatcher/PointMatcher.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Geometry>
#include <boost/thread.hpp>

#include <boost/shared_ptr.hpp>

namespace terrain_slam {
class Adjuster {
public:
  /**
   * @brief Default class constructor.
   */
  Adjuster();

  /**
   * @brief Default destructor.
   */
  ~Adjuster();

  /**
   * @brief      Adjusts the positions of the graph.
   *
   * @param      cloud_fixed  The cloud fixed
   * @param      cloud        The cloud
   */
  void adjust(const boost::shared_ptr<CloudPatch> &cloud_fixed,
              const boost::shared_ptr<CloudPatch> &cloud);

  /**
   * @brief Restarts the Ceres problem
   */
  void reset();

protected:
  ceres::Problem* problem_;

  // Mutex to control the access to the adjuster.
  boost::mutex mutex_adjuster_;
};  // class

class BruteForceAdjuster {
public:
  BruteForceAdjuster(double x_min,
                     double x_max,
                     double y_min,
                     double y_max,
                     double z_min,
                     double z_max,
                     double yaw_min,
                     double yaw_max,
                     double xy_resolution,
                     double z_resolution,
                     double yaw_resolution);
  void adjust(const boost::shared_ptr<CloudPatch> &cloud_fixed,
              const boost::shared_ptr<CloudPatch> &cloud);
  double computeCost(const boost::shared_ptr<CloudPatch> &cloud_fixed,
                    const boost::shared_ptr<CloudPatch> &cloud,
                    const Transform& tf);
private:
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;
  double yaw_min_;
  double yaw_max_;
  double xy_resolution_;
  double z_resolution_;
  double yaw_resolution_;
};  // class

// class PMAdjuster {
// public:
//   typedef PointMatcher<double> PM;
//   typedef PM::DataPoints DP;

//   PMAdjuster();
//   void adjust(const boost::shared_ptr<CloudPatch> &cloud_fixed,
//               const boost::shared_ptr<CloudPatch> &cloud);
// private:
//   DP c1;
//   DP c2;

//   void copy2PM(const boost::shared_ptr<CloudPatch> &cp,
//                DP& dp);
// };  // class

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
                      const Eigen::Vector4d &pt,
                      const double& r, const double& p)
      : c_(c), p_(pt), roll(r), pitch(p) {
    // empty
  }

  bool operator()(double const* const _tx,
                  double const* const _ty,
                  double const* const _tz,
                  double const* const _yaw,
                  double* residuals) const {
    // Copy the values
    // double roll  = *_roll;
    // double pitch = *_pitch;
    double tx    = *_tx;
    double ty    = *_ty;
    double tz    = *_tz;
    double yaw   = *_yaw;

    // std::cout << "Cost: " << tx << ", " << ty << ", " << tz << ", " << roll << ", " << pitch << ", " << yaw << ", " << std::endl;

    // Set the transformation matrix
    Eigen::Matrix4d T;
    double A = cos(yaw),  B = sin(yaw),  C  = cos(pitch), D  = sin(pitch),
           E = cos(roll), F = sin(roll), DE = D*E,        DF = D*F;

    T(0, 0) = A*C;  T(0, 1) = A*DF - B*E;  T(0, 2) = B*F + A*DE;  T(0, 3) = tx;
    T(1, 0) = B*C;  T(1, 1) = A*E + B*DF;  T(1, 2) = B*DE - A*F;  T(1, 3) = ty;
    T(2, 0) = -D;   T(2, 1) = C*F;         T(2, 2) = C*E;         T(2, 3) = tz;
    T(3, 0) = 0;    T(3, 1) = 0;           T(3, 2) = 0;           T(3, 3) = 1;


    // Do the math
    Eigen::Vector4d p = p_;
    Eigen::Vector4d pt = T * p;

    // Find three closest points in cloud
    std::vector<Eigen::Vector4d> nn = c_->kNN(pt, 3);

    Eigen::Vector3d p1(nn.at(0).hnormalized());
    Eigen::Vector3d p2(nn.at(1).hnormalized());
    Eigen::Vector3d p3(nn.at(2).hnormalized());

    // Calculate distance to plane
    Eigen::Vector3d v1 = p2 - p1;
    Eigen::Vector3d v2 = p3 - p1;
    Eigen::Vector3d w  = pt.hnormalized() - p1;
    Eigen::Vector3d n  = v1.cross(v2);
    // residuals[0] = n.dot(w) / n.norm();
    residuals[0] = w(0);
    residuals[1] = w(1);
    residuals[2] = w(2);
    return true;
  }

protected:
  boost::shared_ptr<CloudPatch> c_;
  Eigen::Vector4d p_;
  double roll;
  double pitch;
};  // class
}   // namespace

#endif // ADJUSTER_H
