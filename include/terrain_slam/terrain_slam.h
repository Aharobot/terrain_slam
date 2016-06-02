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

#ifndef TERRAIN_SLAM_H
#define TERRAIN_SLAM_H

#include <terrain_slam/ply_saver.h>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <vector>


namespace terrain_slam {

inline Eigen::Vector3d cv2eigen(const cv::Point3d &p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}

class Transform {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Transform(const Eigen::Matrix4d& tf) : tf_(tf) {}

  Transform(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy) {
    Eigen::Quaternion<double> q = rpy2Rot(rpy);
    tf_ = buildTransform(q, xyz);
  }

  Eigen::Matrix4d tf(void) const {
    return tf_;
  }

  Eigen::Vector3d getPosition(void) const {
    return tf_.block<3, 1>(0,3).transpose();
  }
protected:
  Eigen::Matrix4d tf_;

  Eigen::Quaternion<double> rpy2Rot(const Eigen::Vector3d& rpy) {
    Eigen::AngleAxisd roll_angle(rpy(0), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitch_angle(rpy(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(rpy(2), Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q = roll_angle * pitch_angle * yaw_angle;
    return q;
  }

  Eigen::Matrix4d buildTransform(const Eigen::Quaternion<double> q,
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
};

class LaserLine: public Transform {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LaserLine(int num_points, const Transform& T)
      : Transform(T) {
    points = Eigen::Matrix4Xd(4, num_points);
  }
  LaserLine(int num_points,
            const Eigen::Vector3d &xyz,
            const Eigen::Vector3d &rpy)
      : Transform(xyz, rpy) {
    points = Eigen::Matrix4Xd(4, num_points);
  }
  void add(const Eigen::Vector3d& p, int i) {
    points(0, i) = p(0);
    points(1, i) = p(1);
    points(2, i) = p(2);
    points(3, i) = 1.0;
  }

  Eigen::Vector3d getCentroid() const {
    Eigen::Vector4d c(0, 0, 0, 1);
    c = points.rowwise().sum();
    c = c / (double)points.cols();
    c = tf()*c;
    Eigen::Vector3d v = c.hnormalized();
    return v;
  }

  // Points are stored column-wise
  Eigen::Matrix4Xd points;
};

class CloudPatch: public Transform {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix4Xd points;

  CloudPatch(const std::vector<LaserLine>& lines, int start_idx, int end_idx)
      : Transform(lines[start_idx].tf()), start_idx_(start_idx), end_idx_(end_idx) {
    for (size_t i = start_idx; i <= end_idx; i++)
      add(lines[i]);
  }
  void add(const LaserLine& line) {
    // First line is the TF of the CloudPatch
    if (lines_.size() == 0) {
      tf_ = line.tf();
    }
    // save the LaserLine
    lines_.push_back(line);

    // transform the points to the local tf and add them
    Eigen::Matrix4Xd new_points = transform(line);
    // std::cout << "concatenate to points " << std::endl;
    Eigen::Matrix4Xd cat_points(points.rows(), points.cols()+new_points.cols());
    cat_points << points, new_points;
    points = cat_points;
  }

  Eigen::Matrix4Xd getPoints(bool local_coordinate_frame = false) const {
    Eigen::Matrix4Xd m(points.rows(), points.cols());
    for (int i = 0; i < points.cols(); i++) {
      if (local_coordinate_frame) {
        m.col(i) = points.col(i);
      } else {
        m.col(i) = tf()*points.col(i);
      }
    }
    return m;
  }

  Eigen::Vector3d getCentroid() const {
    Eigen::Vector4d c(0, 0, 0, 1);
    c = points.rowwise().sum();
    c = c / (double)points.cols();
    c = tf()*c;
    Eigen::Vector3d v = c.hnormalized();
    return v;
  }

protected:
  int start_idx_;
  int end_idx_;
  std::vector<LaserLine> lines_;

  Eigen::Matrix4Xd transform(const LaserLine& line) {
    Eigen::Matrix4Xd m(line.points.rows(), line.points.cols());
    for (int i = 0; i < line.points.cols(); i++) {
      if (lines_.size() == 1) {
        // set first line at origin
        m.col(i) = line.points.col(i);
      } else {
        // and the rest
        m.col(i) = tf().inverse()*line.tf()*line.points.col(i);
      }
    }
    return m;
  }
};

class TerrainSlam {
public:
  TerrainSlam(int argc, char **argv);

  /**
   * @brief      Parse command line arguments
   *
   * @param[in]  argc        The argc command line argument
   * @param      argv        The argv command line argument
   *
   * @return     true if successful
   */
  bool parseCommandLine(int argc, char **argv);

  /**
   * @brief      Main process function
   */
  void process();

  /**
   * @brief      Reads files.
   *
   * @param[in]  cloud_names  The cloud names
   * @param[in]  cloud_paths  The cloud paths
   * @param      lines        The laser lines
   */
  void readFiles(const std::vector<std::string> &cloud_names,
                 const std::vector<std::string> &cloud_paths,
                 std::vector<LaserLine> &lines);

  /**
   * @brief      Use boost filesystem to explore the desired path for yml files
   * where the pointclouds are stored using OpenCV FileStorage class.
   *
   * @param[in]  path         The path
   * @param[in]  format       The extension format
   * @param      cloud_names  Output vector containing the names of the files
   * @param      cloud_paths  Output vector containing the full path to the files
   *
   * @return     true if any files have been found, false otherwise
   */
  bool getCloudPaths(const std::string &path, const std::string &format,
                     std::vector<std::string> &cloud_names,
                     std::vector<std::string> &cloud_paths);

  /**
   * @brief Using provided patch_size, splits the pointcloud into patches
   *
   * @param[in]   Lines       Input laser lines
   * @param       patches     Output vector of CloudPatches
   */
  void createPatches(const std::vector<LaserLine>& lines,
                     std::vector<CloudPatch>& patches);

  /**
   * @brief Save the patch to a file
   *
   * @param[in]   patch       CloudPatch to save
   * @param       idx         All saved files follow the same name, you provide the numbering
   */
  void savePatch(const CloudPatch& patch, int idx);

  /**
   * @brief Finds overlapping patches, and return their paired indexes
   *
   * @param[in]   patches     Input vector of CloudPatch
   * @param       candidates  Output vector of paired candidates
   */
  void lookForCandidates(const std::vector<CloudPatch>& patches,
                         std::vector<std::pair<int, int> >& candidates);

  // int preprocessPoints(std::vector<cv::Point3d>& points, int idx);

  std::string clouds_dir_;
  double patch_size_;
  int mean_k_;
  int std_mult_;
  bool debug_;
  bool filter_;

}; // Class
} // Namespace

#endif // TERRAIN_SLAM_H
