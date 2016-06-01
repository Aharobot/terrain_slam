#ifndef STATISTICAL_OUTLIER_REMOVER_H
#define STATISTICAL_OUTLIER_REMOVER_H

#include <Eigen/Geometry>

namespace terrain_slam {

class StatisticalOutlierRemover {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StatisticalOutlierRemover() : mean_k_(1), std_mul_(0.0) {}

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

#endif // STATISTICAL_OUTLIER_REMOVER_H
