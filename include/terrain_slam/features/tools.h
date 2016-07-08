/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef TOOLS_H
#define TOOLS_H

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/intensity_gradient.h>

// pcl definition
typedef pcl::PointXYZRGB           PointRGB;
typedef pcl::PointCloud<pcl::PointXYZ>  PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;
typedef pcl::PointCloud<PointRGB>  PointCloudRGB;

class Tools {
 public:
  /** \brief Normal estimation
    * @return
    * \param Cloud where normals will be estimated
    * \param Cloud surface with additional information to estimate the features for every point in the input dataset
    * \param Output cloud with normals
    */
  static void estimateNormals(const PointCloudXYZ::Ptr& cloud,
                              pcl::PointCloud<pcl::Normal>::Ptr& normals,
                              double radius_search)
  {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation_omp;
    normal_estimation_omp.setInputCloud(cloud);
    normal_estimation_omp.setRadiusSearch(radius_search);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_omp(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation_omp.setSearchMethod(kdtree_omp);
    normal_estimation_omp.compute(*normals);
  }

  /** \brief Compute the intensity gradients
    * @return
    * \param Input intensity cloud
    * \param Input cloud with normals
    * \param Output cloud with gradients
    */
  static void computeGradient(const pcl::PointCloud<pcl::PointXYZI>::Ptr& intensity,
                              const pcl::PointCloud<pcl::Normal>::Ptr& normals,
                              pcl::PointCloud<pcl::IntensityGradient>::Ptr& gradients,
                              double radius_search)
  {
    // Compute the intensity gradients.
    pcl::IntensityGradientEstimation<pcl::PointXYZI,
                                pcl::Normal,
                                pcl::IntensityGradient,
                                pcl::common::IntensityFieldAccessor<pcl::PointXYZI> > ge;
    ge.setInputCloud(intensity);
    ge.setInputNormals(normals);
    ge.setRadiusSearch(radius_search);
    ge.compute(*gradients);
  }

  static bool areEqual(const pcl::PointXYZ& p, const pcl::PointXYZ& q) {
    return (abs(p.x - q.x) < 0.0001) && (abs(p.y - q.y) < 0.0001) && (abs(p.z - q.z) < 0.0001);
  }

  /**
   * @brief Extracts the inliers vector from two pointclouds
   *
   * @param cloud Input clout
   * @param inliers Input cloud of inliers.
   *
   * @return Vector whose index matches cloud inliers
   */
  static void getIndices(const PointCloudXYZ::Ptr& cloud,
                         const PointCloudXYZ::Ptr& inliers,
                         std::vector<int>& inliers_indexes)
  {
    for (size_t i = 0; i < inliers->points.size(); i++) {
      for (size_t j = 0; j < cloud->points.size(); j++) {
        if (areEqual(inliers->points[i], cloud->points[j])) {
          inliers_indexes.push_back(j);
        }
      }
    }
  }

  static inline void
  PointXYZtoXYZI (const pcl::PointXYZ& in, pcl::PointXYZI& out) {
   out.x = in.x; out.y = in.y; out.z = in.z; out.intensity = 0;
  }

  static inline void
  PointCloudXYZtoXYZI(const pcl::PointCloud<pcl::PointXYZ>& in,
                      pcl::PointCloud<pcl::PointXYZI>& out) {
    out.width   = in.width;
    out.height  = in.height;
    for (size_t i = 0; i < in.points.size (); i++) {
      pcl::PointXYZI p;
      PointXYZtoXYZI(in.points[i], p);
      out.points.push_back(p);
    }
  }

  /** \brief Get cloud salient indices
   * @return
   * \param input cloud
   * \param output vector with salient indices
   * \param zeta difference, respect to the z mean, to consider a point to be salient (in percentage)
   */
  static void getCloudSalientIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<uint>& salient_indices, double z_diff_th) {
    std::vector<double> zs;
    salient_indices.clear();
    for (uint i=0; i<cloud->size(); i++)
      zs.push_back(cloud->points[i].z);

    double mean_z = accumulate(zs.begin(), zs.end(), 0.0) / zs.size();
    for (uint i=0; i < zs.size(); i++)
    {
      // Compute zeta difference in percentage
      double diff_percentage = 100.0 - std::min(mean_z, zs[i]) * 100.0 / std::max(mean_z, zs[i]);
      if (diff_percentage > z_diff_th && fabs(mean_z - zs[i]) > 0.1)
        salient_indices.push_back(i);
    }
  }
};
#endif  // TOOLS_H