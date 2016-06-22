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


#ifndef GRIDDER_H
#define GRIDDER_H

#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <algorithm>
#include <random>
#include <memory>

namespace terrain_slam {
class Gridder {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief      Default constructor
   */
  Gridder();

  /**
   * @brief      Constructor
   *
   * @param[in]  resolution     The resolution
   * @param[in]  search_radius  The search radius
   */
  Gridder(double resolution, double search_radius);

  /**
   * @brief      Sets the resolution.
   *
   * @param[in]  resolution  The resolution
   */
  void setResolution(double resolution) {
    resolution_ = resolution;
  }

  /**
   * @brief      Sets the search radius.
   *
   * @param[in]  resolution  The search radius
   */
  void setSearchRadius(double search_radius) {
    search_radius_ = search_radius;
  }

  /**
   * @brief      Sets the input cloud.
   *
   * @param[in]  points  The pointcloud
   */
  inline void setInput(const Eigen::Matrix4Xd &points) {
    points_ = points;
    init_ = true;
  }

  /**
   * @brief      Main method without input parameter. Please call setInput
   * first.
   *
   * @return     Gridded pointcloud
   */
  Eigen::Matrix4Xd grid();

private:
  bool init_;
  Eigen::Matrix4Xd points_;
  double resolution_;
  double search_radius_;

  int ransac_max_iterations_;
  int ransac_num_params_;

  /**
   * @brief      Gets the minimum and maximum coordinates of the cloud
   *
   * @param      min_x  The minimum x
   * @param      min_y  The minimum y
   * @param      max_x  The maximum x
   * @param      max_y  The maximum y
   *
   * @return     success
   */
  bool getMinMax(double &min_x, double &min_y, double &max_x, double &max_y);

  /**
   * @brief      Computes a \f$z\f$ weighted mean at \f$(x,y)\f$ for the points
   * that lie within the queried distance
   *
   * @param[in]  x         X coordinate
   * @param[in]  y         Y coordinate
   * @param[in]  distance  The distance
   * @param[out] p         A point at \f$(x, y, z)\f$, where the \f$z\f$ has
   * been computed using weighted mean.
   *
   * @return     If successful
   */
  bool reduce(const double &x, const double &y, const double &distance, Eigen::Vector3d& p);
};  // class
}   // namespace


#endif // GRIDDER_H
