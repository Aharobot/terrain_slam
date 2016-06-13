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


#ifndef OUTLIER_REMOVER_H
#define OUTLIER_REMOVER_H

#include <Eigen/Geometry>

namespace terrain_slam {

class OutlierRemover {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OutlierRemover() : mean_k_(1), std_mul_(0.0) {}

  /**
   * @brief      Sets the input cloud to be filtered
   *
   * @param[in]  input  The input cloud
   */
  inline void setInput(const Eigen::MatrixXd &input) { input_ = input; }

  /**
   * @brief      Set the number of nearest neighbors to use for mean distance
   * estimation.
   *
   * @param[in]  k     The number of points to use for mean distance estimation.
   */
  inline void setMeanK(int k) { mean_k_ = k; }

  /**
   * @brief     Get the number of nearest neighbors to use for mean distance
   *            estimation.
   *
   * \return The number of points to use for mean distance estimation.
   */
  inline int getMeanK() const { return (mean_k_); }

  /**
   * @brief     Set the standard deviation multiplier for the distance threshold
   *            calculation.
   * @details   The distance threshold will be equal to:
   *            mean + std_mult * stddev.
   *            Points will be classified as inlier or outlier if their average
   *            neighbor * distance is below or above this threshold respectively.
   *
   * @param[in] std_mult The standard deviation multiplier.
   */
  inline void setStdThresh(double std_mult) { std_mul_ = std_mult; }

  /**
   * @brief     Get the standard deviation multiplier for the distance threshold
   *            calculation.
   * @details   The distance threshold will be equal to:
   *            mean + std_mult * stddev.
   *            Points will be classified as inlier or outlier if their average
   *            neighbor distance is below or above this threshold respectively.
   */
  inline double getStdThresh() const { return (std_mul_); }

  /**
   * @brief      Filtered results are stored in a separate point cloud.
   *
   * @param      output  The resulting point cloud
   */
  void filter(Eigen::MatrixXd &output);

protected:
  /**
   * @bried User input pointcloud
   */
  Eigen::MatrixXd input_;

  Eigen::VectorXi removed_indices_;

  /**
   * @brief       The number of points to use for mean distance estimation.
   */
  int mean_k_;

  /**
   * @brief       Standard deviations threshold (i.e., points outside of
   *              \f$ \mu \pm \sigma \cdot std\_mul \f$ will be marked as
   *              outliers).
   */
  double std_mul_;
}; // Class
}  // Namespace

#endif // OUTLIER_REMOVER_H
