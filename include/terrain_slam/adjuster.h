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

#include <terrain_slam/greedy_projector.h>
#include <terrain_slam/clouds.h>

// #include <pointmatcher/PointMatcher.h>

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
  Eigen::Matrix4d adjust(const boost::shared_ptr<CloudPatch> &cloud_fixed,
                         const boost::shared_ptr<CloudPatch> &cloud,
                               bool bounded = true,
                               bool high_precision = true);

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
  Eigen::Matrix4d adjust(const boost::shared_ptr<CloudPatch> &cloud_fixed,
                         const boost::shared_ptr<CloudPatch> &cloud);
  double computeCost(const boost::shared_ptr<CloudPatch> &cloud_fixed,
                    const boost::shared_ptr<CloudPatch> &cloud,
                    const Eigen::Matrix4d& tf);
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
                      const boost::shared_ptr<GreedyProjector> &proj,
                      const Eigen::Vector4d &pt,
                      const double& r, const double& p)
      : c_(c), p_(pt), proj_(proj), roll(r), pitch(p){
    // empty
    c_->updateSearchTree();
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

    // Set the transformation matrix
    Eigen::Matrix4d T;
    double A = cos(yaw),  B = sin(yaw),  C  = cos(pitch), D  = sin(pitch),
           E = cos(roll), F = sin(roll), DE = D*E,        DF = D*F;

    T(0, 0) = A*C;  T(0, 1) = A*DF - B*E;  T(0, 2) = B*F + A*DE;  T(0, 3) = tx;
    T(1, 0) = B*C;  T(1, 1) = A*E + B*DF;  T(1, 2) = B*DE - A*F;  T(1, 3) = ty;
    T(2, 0) = -D;   T(2, 1) = C*F;         T(2, 2) = C*E;         T(2, 3) = tz;
    T(3, 0) = 0;    T(3, 1) = 0;           T(3, 2) = 0;           T(3, 3) = 1;


    // Transform the point for the current iteration
    Eigen::Vector4d p = p_;
    Eigen::Vector4d pt = T * p;

    //std::vector<Eigen::Vector4d> nn = c_->kNN2d(pt, 3);

    // Find three closest points in cloud
    bool inside = proj_->isInside(pt);
    std::vector<Eigen::Vector4d> vertexes;
    double dz = -1;
    if (inside) {
      vertexes = proj_->locate(pt);
      if (vertexes.size() == 3) {
        // Interpolate z inside the triangle
        Eigen::Vector4d p1(vertexes.at(0));
        Eigen::Vector4d p2(vertexes.at(1));
        Eigen::Vector4d p3(vertexes.at(2));

        A = (p2(1) - p1(1))*(p3(2)-p1(2))-(p3(1)-p1(1))*(p2(2)-p1(2));
        B = (p2(2) - p1(2))*(p3(0)-p1(0))-(p3(2)-p1(2))*(p2(0)-p1(0));
        C = (p2(0) - p1(0))*(p3(1)-p1(1))-(p3(0)-p1(0))*(p2(1)-p1(1));
        D = -(A*p1(0) + B*p1(1) + C*p1(2));
        double z = -(A*pt(0) + B*pt(1) + D) / C;
        dz = std::abs(pt(2)) - std::abs(z);
      }
    }

    // Calculate distance to nearest 3D point
    std::vector<Eigen::Vector4d> nn = c_->kNN(pt, 1);
    Eigen::Vector4d nearest_point(nn.at(0));
    Eigen::Vector4d dist = pt - nearest_point;
    dist = dist.cwiseAbs();

    //TODO take into account distance to camera

    if (inside) {
      if (dz <= 0.01) {
        residuals[0] = dz;
        residuals[1] = 0;
        residuals[2] = 0;
      } else if (dz > 0.01) {
        residuals[0] = dz;
        residuals[1] = dist(1);
        residuals[2] = dist(0);
      }
    } else {
      residuals[0] = 0;
      residuals[1] = dist(1);
      residuals[2] = dist(0);
    }

    // residuals[0] = v3d(2)*v3d(2);
    // residuals[1] = v3d(1)*0.5;
    // residuals[2] = v3d(0)*0.5;

    // if ((v3d(1) + v3d(0)) > 0.05)  {
      // residuals[0] = v3d(2)*v3d(2);
      // residuals[1] = v3d(1)*0.5;
      // residuals[2] = v3d(0)*0.5;
    // } else {
    //   residuals[0] = dz; //*z_multiplier;
    //   residuals[1] = 0;
    //   residuals[2] = 0;
    // }

    return true;
  }

protected:
  boost::shared_ptr<CloudPatch> c_;
  Eigen::Vector4d p_;
  boost::shared_ptr<GreedyProjector> proj_;
  double roll;
  double pitch;
};  // class

}   // namespace

#endif // ADJUSTER_H
