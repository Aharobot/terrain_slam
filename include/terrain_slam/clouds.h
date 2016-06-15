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

#ifndef CLOUDS_H
#define CLOUDS_H

#include <terrain_slam/gridder.h>
#include <terrain_slam/transform.h>

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <nabo/nabo.h>



namespace terrain_slam {

inline Eigen::Vector3d cv2eigen(const cv::Point3d &p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}

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

  CloudPatch(const std::vector<LaserLine>& lines, int start_idx, int end_idx);

  void add(const LaserLine& line);

  void grid(double resolution = 0.1, double search_radius = 0.2);

  std::vector<Eigen::Vector4d> kNN(const Eigen::Vector4d& q, int k) const ;

  /**
   * @brief Save the patch to a file
   *
   * @param       idx         All saved files follow the same name, you provide the numbering
   */
  void save(int idx,
            const std::string path = std::string("../output"),
            const std::string suffix = std::string(""),
            bool local_coordinate_frame = false) ;

  /**
   * @brief      Gets the start index.
   *
   * @return     The start index.
   */
  int getStartIdx() { return start_idx_; }

  /**
   * @brief      Gets the end index.
   *
   * @return     The end index.
   */
  int getEndIdx() { return end_idx_; }

  /**
   * @brief      Sets the identifier.
   *
   * @param[in]  id    The identifier
   */
  void setId(int id) { id_ = id; }

  /**
   * @brief      Gets the identifier.
   *
   * @return     The identifier.
   */
  int getId(void) { return id_; }

  /**
   * @brief      Returns the point at location i
   *
   * @param[in]  i     index
   *
   * @return     Point
   */
  Eigen::Vector4d point(int i) const { return points.col(i); }

  /**
   * @brief      Returns the number of points
   *
   * @return     Number of points
   */
  size_t size() const { return points.cols(); }

  /**
   * @brief      Gets the grid.
   *
   * @return     The grid.
   */
  Eigen::Matrix4Xd getGrid() const { return grid_; }

  /**
   * @brief      Gets the points.
   *
   * @param[in]  local_coordinate_frame  Whether points should be transformed to
   * the local coordinate frame
   *
   * @return     The points.
   */
  Eigen::Matrix4Xd getPoints(bool local_coordinate_frame = false) const ;

  /**
   * @brief      Gets the centroid.
   *
   * @return     The centroid.
   */
  Eigen::Vector3d getCentroid() const ;

protected:
  bool gridded_;
  int start_idx_;
  int end_idx_;
  int id_;
  std::vector<LaserLine> lines_;
  Eigen::MatrixXd grid_;
  boost::shared_ptr<Nabo::NNSearchD> nns_;

  /**
   * @brief      Transform the laser line to the local coordinate frame of the
   * cloud
   *
   * @param[in]  line  The line
   *
   * @return     The points
   */
  Eigen::Matrix4Xd transform(const LaserLine& line);
};

typedef boost::shared_ptr<CloudPatch> CloudPatchPtr;

}   // Namespace

#endif // CLOUDS_H
