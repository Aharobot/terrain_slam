#ifndef EIGEN_TOOLS_H
#define EIGEN_TOOLS_H

#include <Eigen/Geometry>

namespace eigen_tools {

static Eigen::Matrix4d buildTransform(double x,
                                      double y,
                                      double z,
                                      double roll,
                                      double pitch,
                                      double yaw) {
  Eigen::Matrix4d m;
  double A = cos(yaw),  B = sin(yaw),  C  = cos(pitch), D  = sin(pitch),
         E = cos(roll), F = sin(roll), DE = D*E,        DF = D*F;

  m(0, 0) = A*C;  m(0, 1) = A*DF - B*E;  m(0, 2) = B*F + A*DE;  m(0, 3) = x;
  m(1, 0) = B*C;  m(1, 1) = A*E + B*DF;  m(1, 2) = B*DE - A*F;  m(1, 3) = y;
  m(2, 0) = -D;   m(2, 1) = C*F;         m(2, 2) = C*E;         m(2, 3) = z;
  m(3, 0) = 0;    m(3, 1) = 0;           m(3, 2) = 0;           m(3, 3) = 1;

  return m;
}

static Eigen::Matrix4d buildTransform(const Eigen::Vector3d& xyz,
                                      const Eigen::Vector3d& rpy) {
  double x = xyz(0),     y = xyz(1),   z = xyz(2),
      roll = rpy(0), pitch = rpy(1), yaw = rpy(2);
  return buildTransform(x, y, z, roll, pitch, yaw);
}

static Eigen::Matrix4d buildTransform(const Eigen::Quaternion<double>& q,
                                      const Eigen::Vector3d& t) {
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

static inline Eigen::Vector3d getOrigin(const Eigen::Matrix4d& m) {
  return m.block<3, 1>(0, 3);
}

static inline Eigen::Quaternion<double> getRotation(const Eigen::Matrix4d& m) {
  return Eigen::Quaternion<double>(m.block<3, 3>(0, 0));
}

static Eigen::Isometry3d toIsometry(const Eigen::Matrix4d& m) {
  Eigen::Vector3d t = m.block<3, 1>(0, 3);
  Eigen::Quaterniond q(m.block<3, 3>(0, 0));
  Eigen::Isometry3d out = (Eigen::Isometry3d)q;
  out.translation() = t;
  return out;
}

}  // namespace eigen_tools

#endif // EIGEN_TOOLS_H