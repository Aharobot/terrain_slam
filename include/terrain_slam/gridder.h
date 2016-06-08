#ifndef GRIDDER_H
#define GRIDDER_H

#include <Eigen/Geometry>
#include <iostream>

namespace terrain_slam {
class Gridder {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief      Default constructor
   */
  Gridder() : init_(false), resolution_(0.2) {
    // empty constructor
  }

  /**
   * @brief      Constructor
   *
   * @param[in]  resolution  The resolution
   */
  Gridder(double resolution) {
    setResolution(resolution);
  }

  /**
   * @brief      Sets the resolution.
   *
   * @param[in]  resolution  The resolution
   */
  void setResolution(double resolution) {
    resolution_ = resolution;
  }

  /**
   * @brief      Sets the input cloud.
   *
   * @param[in]  points  The pointcoud
   */
  inline void setInput(const Eigen::Matrix4Xd &points) {
    Eigen::Matrix3Xd temp = points.topRows(3);
    setInput(temp);
  }

  /**
   * @brief      Sets the input cloud.
   *
   * @param[in]  points  The pointcloud
   */
  inline void setInput(const Eigen::Matrix3Xd &points) {
    points_ = points;
    init_ = true;
  }

  /**
   * @brief      Main method without input parameter. Please call setInput
   * first.
   *
   * @return     Gridded pointcloud
   */
  Eigen::Matrix3Xd grid() {
    if (!init_) {
      std::cerr << "[Gridder]: Input pointcloud not set" << std::endl;
    }

    double min_x, min_y, max_x, max_y;
    getMinMax(min_x, min_y, max_x, max_y);

    std::cout << "Limits from (" << min_x << ", " << min_y << ") to ("
                                << max_x << ", " << max_y << ") "
                                << std::endl;

    // Round to nearest divisible by resolution
    min_x = resolution_ * static_cast<int>(min_x / resolution_);
    min_y = resolution_ * static_cast<int>(min_y / resolution_);
    max_x = resolution_ * static_cast<int>(max_x / resolution_);
    max_y = resolution_ * static_cast<int>(max_y / resolution_);

    std::cout << "Range from (" << min_x << ", " << min_y << ") to ("
                                << max_x << ", " << max_y << ") "
                                << "with a resolution of " << resolution_
                                << std::endl;
    double size_x = max_x - min_x;
    double size_y = max_y - min_y;
    int grid_size_x = static_cast<int>(size_x / resolution_ + 1.0);
    int grid_size_y = static_cast<int>(size_y / resolution_ + 1.0);
    int points_size = grid_size_x * grid_size_y;

    Eigen::Matrix3Xd m(3, points_size);

    // resample points to resolution
    for (size_t i = 0; i < grid_size_x; i++) {
      double x_coord = min_x + resolution_*i;
      for (size_t j = 0; j < grid_size_y; j++) {
        double y_coord = min_y + resolution_*j;
        Eigen::Vector3d p = reduce(x_coord, y_coord, resolution_);
        m.col(i*grid_size_x + j) = p;
      }
    }

    // Remove zero columns
    for (size_t i = 0; i < m.cols(); i++) {
      if (m.col(i).sum() == 0) {
        // Remove current row
        m.block(i, 0, m.rows()-i, m.cols()) = m.block(i + 1 , 0, m.rows()-i, m.cols());
        m.conservativeResize(m.rows(),m.cols());
      }
    }
    return m;
  }



private:
  bool init_;
  Eigen::Matrix3Xd points_;
  double resolution_;

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
  bool getMinMax(double &min_x, double &min_y, double &max_x, double &max_y) {
    min_x = points_(0, 0);
    min_y = points_(0, 1);
    max_x = points_(0, 0);
    max_y = points_(0, 1);

    for (size_t i = 0; i < points_.cols(); i++) {
      Eigen::Vector3d v = points_.col(i);
      double vx = v(0);
      double vy = v(1);
      if (vx > max_x) {
        max_x = vx;
      } else if (vx < min_x) {
        min_x = vx;
      }

      if (vy > max_y) {
        max_y = vy;
      } else if (vy < min_y) {
        min_y = vy;
      }
    }
    if ((min_x =! max_x) && (min_y =! max_y)) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * @brief      Computes a \f$z\f$ weighted mean at \f$(x,y)\f$ for the points
   * that lie within the queried distance
   *
   * @param[in]  x         X coordinate
   * @param[in]  y         Y coordinate
   * @param[in]  distance  The distance
   *
   * @return     A point at \f$(x, y, z)\f$, where the \f$z\f$ has been computed using
   * weighted mean.
   */
  Eigen::Vector3d reduce(double x, double y, double distance) {
    // 2D query point
    Eigen::Vector2d c(x, y);
    // 2D point cloud
    Eigen::Matrix2Xd p_xy = points_.topRows(2);
    // Compute distance from c to all points
    Eigen::VectorXd sq_d = (p_xy.colwise() - c).colwise().squaredNorm();

    if (sq_d.size() == 0) {
      std::cout << "[ERROR]: There are no points from ("
                << x << ", " << y << ") within distance: "
                << distance << std::endl;
    }

    // Use weighted mean to compute Z
    // weight is inversely proportional to distance
    // \f$ \overline{z} = \sum_i \frac{z_i\cdot d^{-1}_i}{d^{-1}_i} \f$
    double z = 0;
    double d = 0;
    int cnt = 0;
    for (size_t i = 0; i < sq_d.size(); i++) {
      if (sq_d(i) < distance) {
        // std::cout << "KKKKK " << std::endl;
        z += points_(2, i)/sq_d(i);
        d += 1/sq_d(i);
        cnt++;
      }
    }

    if (d == 0) {
      // std::cout << "[WARN]: Distance is zero! " << cnt << " points visited" << std::endl;
      return Eigen::Vector3d(0, 0, 0);
    } else {
      z /= d;
      return Eigen::Vector3d(x, y, z);
    }
  }




};  // class
}   // namespace


#endif // GRIDDER_H
