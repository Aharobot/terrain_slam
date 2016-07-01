/// Copyright 2015 Pep Lluis Negre Carrasco
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef KEYPOINT_H
#define KEYPOINT_H

// Generic pcl
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

// pcl keypoints
#include <pcl/impl/point_types.hpp>
#include <pcl/keypoints/agast_2d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/susan.h>
// #include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/uniform_sampling.h>

// pcl features
#include <pcl/features/normal_3d_omp.h>

// pcl definition
typedef pcl::PointXYZRGB           PointRGB;
typedef pcl::PointCloud<pcl::PointXYZ>  PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;
typedef pcl::PointCloud<PointRGB>  PointCloudRGB;

// List the available keypoints
static const string KP_AGAST_DETECTOR_7_12s  = "AgastDetector7_12s";
static const string KP_AGAST_DETECTOR_5_8    = "AgastDetector5_8";
static const string KP_OAST_DETECTOR_9_16    = "OastDetector9_16";
static const string KP_HARRIS_3D             = "Harris3D";
static const string KP_ISS                   = "ISS";
static const string KP_NARF                  = "Narf";
static const string KP_SIFT                  = "Sift";
static const string KP_SUSAN                 = "Susan";
static const string KP_UNIFORM_SAMPLING      = "UniformSampling";

class Keypoints
{

public:

  // Constructor
  explicit Keypoints(const string kp_type);
  explicit Keypoints(const string kp_type, double normal_radius_search);

  // Detect
  void compute(const PointCloudXYZ::Ptr& cloud, PointCloudXYZ::Ptr& cloud_keypoints);

  // Normal estimation
  void normals(const PointCloudXYZ::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals);

  // Get keypoint cloud indices from pointcloud of PointUVs
  void getKeypointsCloud( const PointCloudXYZ::Ptr& cloud,
                          const pcl::PointCloud<pcl::PointUV>::Ptr& keypoints,
                          PointCloudXYZ::Ptr& cloud_keypoints);
  void getKeypointsCloud( const PointCloudXYZ::Ptr& cloud,
                          const pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoints,
                          PointCloudXYZ::Ptr& cloud_keypoints);

  // Computes the cloud resolution
  double computeCloudResolution(const PointCloudXYZ::Ptr& cloud);

private:

  string kp_type_;               //!> Stores the keypoint type
  double normal_radius_search_;  //!> Normal radius search

};

/** \brief Class constructor. Initialize the class
  */
Keypoints::Keypoints(const string kp_type) : kp_type_(kp_type)
{
  normal_radius_search_ = 0.05;
}
Keypoints::Keypoints(const string kp_type, const double normal_radius_search) :
  kp_type_(kp_type), normal_radius_search_(normal_radius_search) {}


/** \brief Detects the keypoints of the input cloud
  * @return
  * \param Input cloud
  * \param Stores the indices of the detected keypoints on the input cloud
  */
void Keypoints::compute(const PointCloudXYZ::Ptr& cloud, PointCloudXYZ::Ptr& cloud_keypoints)
{
  // AGAST
  if (kp_type_ == KP_AGAST_DETECTOR_7_12s || kp_type_ == KP_AGAST_DETECTOR_5_8 || kp_type_ == KP_OAST_DETECTOR_9_16)
  {
    // https://code.google.com/p/kfls2/source/browse/trunk/apps/src/ni_agast.cpp?r=2

    // Keypoints
    pcl::PointCloud<pcl::PointUV>::Ptr keypoints(new pcl::PointCloud<pcl::PointUV>);

    // Parameters
    double bmax = 255;
    double threshold = 30;

    // The generic agast
    pcl::AgastKeypoint2D<pcl::PointXYZ> agast;
    agast.setNonMaxSuppression(true);
    agast.setThreshold(threshold);
    agast.setMaxDataValue(bmax);
    agast.setInputCloud(cloud);

    // Detector
    if (kp_type_ == KP_AGAST_DETECTOR_7_12s)
    {
      pcl::keypoints::agast::AgastDetector7_12s::Ptr detector(new pcl::keypoints::agast::AgastDetector7_12s(cloud->width, cloud->height, threshold, bmax));
      agast.setAgastDetector(detector);
    }
    else if (kp_type_ == KP_AGAST_DETECTOR_5_8)
    {
      pcl::keypoints::agast::AgastDetector5_8::Ptr detector(new pcl::keypoints::agast::AgastDetector5_8 (cloud->width, cloud->height, threshold, bmax));
      agast.setAgastDetector(detector);

    }
    else if (kp_type_ == KP_OAST_DETECTOR_9_16)
    {
      pcl::keypoints::agast::OastDetector9_16::Ptr detector(new pcl::keypoints::agast::OastDetector9_16 (cloud->width, cloud->height, threshold, bmax));
      agast.setAgastDetector(detector);
    }

    // Compute the keypoints
    agast.compute(*keypoints);

    // Convert keypoints to pointcloud indices
    getKeypointsCloud(cloud, keypoints, cloud_keypoints);
    return;
  }

  // HARRIS 3D
  else if(kp_type_ == KP_HARRIS_3D)
  {
    // https://github.com/PointCloudLibrary/pcl/blob/master/examples/keypoints/example_get_keypoints_indices.cpp
    // http://docs.ros.org/hydro/api/pcl/html/tutorial_8cpp_source.html
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris3d;
    PointCloudXYZI::Ptr keypoints(new PointCloudXYZI);
    harris3d.setNonMaxSupression(true);
    harris3d.setInputCloud(cloud);
    harris3d.setThreshold(1e-6);
    harris3d.compute(*keypoints);

    // Extract the indices
    getKeypointsCloud(cloud, keypoints, cloud_keypoints);
    return;
  }

  // ISS
  else if(kp_type_ == KP_ISS)
  {
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;
    detector.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    detector.setSearchMethod(kdtree);
    double resolution = computeCloudResolution(cloud);
    detector.setSalientRadius(6 * resolution);
    detector.setNonMaxRadius(4 * resolution);
    detector.setMinNeighbors(5);
    detector.setThreshold21(0.975);
    detector.setThreshold32(0.975);
    detector.compute(*cloud_keypoints);
    return;
  }
  // SIFT
  else if (kp_type_ == KP_SIFT)
  {
    // https://github.com/otherlab/pcl/blob/master/examples/keypoints/example_sift_normal_keypoint_estimation.cpp

    // Parameters for sift computation
    const float min_scale = 0.25;
    const int n_octaves = 3;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.05;

    // Estimate the normals of the input cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals;
    normals(cloud, cloud_normals);

    // Estimate the sift interest points using normals values from xyz as the Intensity variants
    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointXYZI> sift;
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud_normals);
    sift.compute(*keypoints);

    // Extract the indices
    getKeypointsCloud(cloud, keypoints, cloud_keypoints);
    return;
  }

  // SUSAN
  else if (kp_type_ == KP_SUSAN)
  {
    // Detect
    pcl::SUSANKeypoint<pcl::PointXYZ, pcl::PointXYZ>* susan3D = new  pcl::SUSANKeypoint<pcl::PointXYZ, pcl::PointXYZ>;
    susan3D->setInputCloud(cloud);
    susan3D->setNonMaxSupression(true);
    susan3D->compute(*cloud_keypoints);
    return;
  }
}

/** \brief Normal estimation
  * @return
  * \param Input cloud
  * \param Output cloud with normals
  */
void Keypoints::normals(const PointCloudXYZ::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
{
  // Init
  cloud_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());

  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree_n);
  ne.setRadiusSearch(normal_radius_search_);
  ne.compute(*cloud_normals);

  // Copy the xyz info from input cloud and add it to cloud_normals as the xyz field in PointNormals estimation is zero
  for(size_t i = 0; i<cloud_normals->points.size(); ++i)
  {
    cloud_normals->points[i].x = cloud->points[i].x;
    cloud_normals->points[i].y = cloud->points[i].y;
    cloud_normals->points[i].z = cloud->points[i].z;
  }
}


/** \brief Get keypoint cloud indices from pointcloud of PointUVs
  * @return
  * \param Input cloud
  * \param Input keypoints of type PointUV
  * \param The output cloud indices corresponding to the detected keypoints
  */
void Keypoints::getKeypointsCloud(const PointCloudXYZ::Ptr& cloud,
                                  const pcl::PointCloud<pcl::PointUV>::Ptr& keypoints,
                                  PointCloudXYZ::Ptr& cloud_keypoints)
{
  // Reset output
  cloud_keypoints.reset(new PointCloudXYZ);

  // Sanity check
  if (!cloud || !keypoints || cloud->points.empty() || keypoints->points.empty())
    return;

  // Init the kdtree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  for (size_t i=0; i<keypoints->size(); ++i)
  {
    // Get the point in the pointcloud
    pcl::PointXYZ &pt = (*cloud)(static_cast<long unsigned int>(keypoints->points[i].u),
                            static_cast<long unsigned int>(keypoints->points[i].v));

    if (!pcl_isfinite(pt.x) || !pcl_isfinite(pt.y) || !pcl_isfinite(pt.z))
      continue;

    cloud_keypoints->points.push_back(pt);
  }
}

/** \brief Get keypoint cloud indices from pointcloud of PointUVs
  * @return
  * \param Input cloud
  * \param Input keypoints of type PointXYZI
  * \param The output cloud indices corresponding to the detected keypoints
  */
void Keypoints::getKeypointsCloud(const PointCloudXYZ::Ptr& cloud,
                                  const PointCloudXYZI::Ptr& keypoints,
                                  PointCloudXYZ::Ptr& cloud_keypoints)
{
  // Reset output
  cloud_keypoints.reset(new PointCloudXYZ);

  // Sanity check
  if (!cloud || !keypoints || cloud->points.empty() || keypoints->points.empty())
    return;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  for (size_t i=0; i<keypoints->size(); ++i)
  {
    // Get the point in the pointcloud
    pcl::PointXYZI pt_tmp = keypoints->points[i];
    pcl::PointXYZ pt;
    pt.x = pt_tmp.x;
    pt.y = pt_tmp.y;
    pt.z = pt_tmp.z;

    if (!pcl_isfinite(pt.x) || !pcl_isfinite(pt.y) || !pcl_isfinite(pt.z))
      continue;

    // Search this point into the cloud
    vector<int> idx_vec;
    vector<float> dist;
    if (kdtree.nearestKSearch(pt, 1, idx_vec, dist) > 0)
    {
      if (dist[0] < 0.0001)
        cloud_keypoints->points.push_back(cloud->points[idx_vec[0]]);
    }
  }
}

/** \brief Computes the cloud resolution. From http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
  * @return the cloud resolution
  * \param Input cloud
  */
double Keypoints::computeCloudResolution(const PointCloudXYZ::Ptr& cloud)
{
  double resolution = 0.0;
  int numberOfPoints = 0;
  int nres;
  vector<int> indices(2);
  vector<float> squaredDistances(2);
  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud);

  for (size_t i = 0; i < cloud->size(); ++i)
  {
    if (! pcl_isfinite((*cloud)[i].x))
      continue;

    // Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
    if (nres == 2)
    {
      resolution += sqrt(squaredDistances[1]);
      ++numberOfPoints;
    }
  }
  if (numberOfPoints != 0)
    resolution /= numberOfPoints;

  return resolution;
}


#endif // KEYPOINT_H