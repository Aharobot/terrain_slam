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

#include <terrain_slam/transform.h>

terrain_slam::Transform::Transform(const Eigen::Matrix4d& m) : T(m) {}

terrain_slam::Transform::Transform(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy) {
  Eigen::Quaternion<double> q = rpy2Rot(rpy);
  T = buildTransform(q, xyz);
}

terrain_slam::Transform::Transform(double x, double y, double z,
          double roll, double pitch, double yaw) {
  T = buildTransform(x, y, z, roll, pitch, yaw);
}

Eigen::Matrix4d terrain_slam::Transform::tf() const {
  return T;
}

Eigen::Matrix3d terrain_slam::Transform::R() const {
  return T.block<3,3>(0,0);
}

Eigen::Vector3d terrain_slam::Transform::t() const {
  return T.block<3,1>(0,3);
}

double terrain_slam::Transform::roll() const {
  Eigen::Matrix3d R = T.block<3,3>(0,0);
  Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
  return rpy(0);
}

double terrain_slam::Transform::pitch() const {
  Eigen::Matrix3d R = T.block<3,3>(0,0);
  Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
  return rpy(1);
}

double terrain_slam::Transform::yaw() const {
  Eigen::Matrix3d R = T.block<3,3>(0,0);
  Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
  return rpy(2);
}

double terrain_slam::Transform::tx() const {
  return T(0, 3);
}

double terrain_slam::Transform::ty() const {
  return T(1, 3);
}

double terrain_slam::Transform::tz() const {
  return T(2, 3);
}

double terrain_slam::Transform::distanceTo(Transform other) const {
  Eigen::Vector3d p = getPosition();
  Eigen::Vector3d po = other.getPosition();
  Eigen::Vector3d d = p - po;
  return d.norm();
}

Eigen::Isometry3d terrain_slam::Transform::getIsometry() const {
  Eigen::Vector3d t(getPosition());
  Eigen::Quaterniond q(R());
  Eigen::Isometry3d out = (Eigen::Isometry3d)q;
  out.translation() = t;
  return out;
}

Eigen::Vector3d terrain_slam::Transform::getOrigin(void) const {
  return getPosition();
}

Eigen::Vector3d terrain_slam::Transform::getPosition(void) const {
  return T.block<3, 1>(0,3);
}

void terrain_slam::Transform::setTransform(const Eigen::Matrix4d &t) {
  T = t;
}

Eigen::Quaternion<double>
terrain_slam::Transform::rpy2Rot(const Eigen::Vector3d& rpy) {
  Eigen::AngleAxisd roll_angle(rpy(0), Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitch_angle(rpy(1), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(rpy(2), Eigen::Vector3d::UnitZ());
  Eigen::Quaternion<double> q = roll_angle * pitch_angle * yaw_angle;
  return q;
}

Eigen::Matrix4d
terrain_slam::Transform::buildTransform(double x, double y, double z,
          double roll, double pitch, double yaw) {
  Eigen::Matrix4d m;
  double A = cos(yaw),  B = sin(yaw),  C  = cos(pitch), D  = sin(pitch),
         E = cos(roll), F = sin(roll), DE = D*E,        DF = D*F;

  m(0, 0) = A*C;  m(0, 1) = A*DF - B*E;  m(0, 2) = B*F + A*DE;  m(0, 3) = x;
  m(1, 0) = B*C;  m(1, 1) = A*E + B*DF;  m(1, 2) = B*DE - A*F;  m(1, 3) = y;
  m(2, 0) = -D;   m(2, 1) = C*F;         m(2, 2) = C*E;         m(2, 3) = z;
  m(3, 0) = 0;    m(3, 1) = 0;           m(3, 2) = 0;           m(3, 3) = 1;

  return m;
}

Eigen::Matrix4d
terrain_slam::Transform::buildTransform(const Eigen::Quaternion<double> q,
                               const Eigen::Vector3d t) {
  Eigen::Matrix3d R = q.matrix();
  Eigen::Matrix4d T;
  // Set to Identity to make bottom row of Matrix 0,0,0,1
  T.setIdentity();
  T.block<3, 3>(0, 0) = R;
  T(0, 3) = t(0);
  T(1, 3) = t(1);
  T(2, 3) = t(2);
  return T;
}
