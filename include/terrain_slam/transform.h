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

#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <Eigen/Geometry>

namespace terrain_slam {
class Transform {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix4d T;

  Transform(const Eigen::Matrix4d& m);
  Transform(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy);
  Transform(double x, double y, double z,
            double roll, double pitch, double yaw);
  Eigen::Matrix4d tf() const;
  Eigen::Matrix3d R() const;
  Eigen::Vector3d t() const;
  double roll() const;
  double pitch() const;
  double yaw() const;
  double tx() const;
  double ty() const;
  double tz() const;
  double distanceTo(Transform other) const;
  Eigen::Isometry3d getIsometry() const;
  Eigen::Vector3d getOrigin(void) const;
  Eigen::Vector3d getPosition(void) const;
  void setTransform(const Eigen::Matrix4d &t);
protected:
  Eigen::Quaternion<double> rpy2Rot(const Eigen::Vector3d& rpy);
  Eigen::Matrix4d buildTransform(double x, double y, double z,
                                 double roll, double pitch, double yaw);
  Eigen::Matrix4d buildTransform(const Eigen::Quaternion<double> q,
                                 const Eigen::Vector3d t);
};
}
#endif // TRANSFORM_H