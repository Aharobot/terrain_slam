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

#ifndef PCL_TOOLS_H
#define PCL_TOOLS_H

#include <terrain_slam/greedy_projector.h>

// Generic pcl
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/intensity_spin.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/surface/mls.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <boost/random/uniform_real_distribution.hpp>

#include <string>

namespace pcl_tools {

/**
 * @brief      Convert an Eigen pointcloud to a PCL pointcloud
 *
 * @param[in]  points       Eigen pointcloud
 *
 * @return     PCL pointcloud pointer
 */
static pcl::PointCloud<pcl::PointXYZ>::Ptr toPCL(const Eigen::Matrix4Xd &points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = points.cols();
  std::cout << "toPCL size is " << points.cols() << std::endl;
  cloud->height = 1;  // unorganized point cloud
  cloud->is_dense = false;
  // Loop through all points
  for (int i = 0; i < points.cols(); i++) {
    double x = points(0, i);
    double y = points(1, i);
    double z = points(2, i);
    cloud->points.push_back(pcl::PointXYZ(x, y, z));
  }
  return cloud;
}

/**
 * @brief      Convert a PCL pointcloud to an eigen pointcloud
 *
 * @param[in]  cloud  PCL cloud
 *
 * @return     Eigen pointcloud
 */
static Eigen::Matrix4Xd fromPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  Eigen::Matrix4Xd points(4, cloud->points.size());
  std::cout << "fromPCL size is " << cloud->size() << std::endl;
  // Loop through all points
  for (int i = 0; i < cloud->points.size(); i++) {
    points(0, i) = cloud->points[i].x;
    points(1, i) = cloud->points[i].y;
    points(2, i) = cloud->points[i].z;
  }
  return points;
}

/**
 * @brief      Convert a PCL Normal pointcloud to an eigen pointcloud
 *
 * @param[in]  cloud  PCL cloud
 *
 * @return     Eigen pointcloud
 */
static Eigen::Matrix4Xd fromPCL(const pcl::PointCloud<pcl::Normal>::Ptr& cloud) {
  Eigen::Matrix4Xd points(4, cloud->points.size());
  // Loop through all points
  for (int i = 0; i < cloud->points.size(); i++) {
    points(0, i) = cloud->points[i].normal_x;
    points(1, i) = cloud->points[i].normal_y;
    points(2, i) = cloud->points[i].normal_z;
  }
  return points;
}

static void removeOutliers(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
          double min_z, double max_z, double radius, int neighbors,
          pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) {
  // Remove point at a Z less or bigger than values
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ>());
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, min_z)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, max_z)));
  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(cloud_in);
  condrem.setKeepOrganized(false);
  // apply filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  condrem.filter(*cloud_filtered);

  // std::cout << "ConditionalRemoval: Before: " << cloud_in->size() << " after: " << cloud_filtered->size() << std::endl;

  // Remove points that have less than N neighbours in a radius
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // build the filter
  outrem.setInputCloud(cloud_filtered);
  outrem.setRadiusSearch(radius);
  outrem.setMinNeighborsInRadius(neighbors);
  // apply filter
  outrem.filter(*cloud_out);
  // std::cout << "RadiusOutlierRemoval: Before: " << cloud_filtered->size() << " after: " << cloud_out->size() << std::endl;
}

static void exaggerateZ(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
          double multiplier,
          pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) {
  pcl::copyPointCloud(*cloud_in, *cloud_out);
  for (size_t i = 0; i < cloud_out->points.size(); i++) {
    cloud_out->points[i].z *= multiplier;
  }
}

static float sign(const pcl::PointXY& p1,
                  const pcl::PointXY& p2,
                  const pcl::PointXY& p3) {
  return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

static bool pointInTriangle(const pcl::PointXY& pt,
                            const pcl::PointXY& p1,
                            const pcl::PointXY& p2,
                            const pcl::PointXY& p3) {
  bool b1, b2, b3;
  b1 = sign(pt, p1, p2) < 0.0f;
  b2 = sign(pt, p2, p3) < 0.0f;
  b3 = sign(pt, p3, p1) < 0.0f;
  return ((b1 == b2) && (b2 == b3));
}

static pcl::PointXYZ interpolate(const pcl::PointXY& pt,
                                 const pcl::PointXYZ& p1,
                                 const pcl::PointXYZ& p2) {
  double dp2p1 = sqrt((p2.x - p1.x)*(p2.x - p1.x)+(p2.y - p1.y)*(p2.y - p1.y));
  double dptp1 = sqrt((pt.x - p1.x)*(pt.x - p1.x)+(pt.y - p1.y)*(pt.y - p1.y));
  double dptp2 = dp2p1 - dptp1;
  double w1 = dptp1/dp2p1;
  double w2 = dptp2/dp2p1;
  double z = p1.z*w1 + p2.z*w2;
  return pcl::PointXYZ(pt.x, pt.y, z);
}

static pcl::PointXYZ interpolate(const pcl::PointXY& pt,
                                 const pcl::PointXYZ& p1,
                                 const pcl::PointXYZ& p2,
                                 const pcl::PointXYZ& p3) {
  double A = (p2.y - p1.y)*(p3.z-p1.z)-(p3.y-p1.y)*(p2.z-p1.z);
  double B = (p2.z - p1.z)*(p3.x-p1.x)-(p3.z-p1.z)*(p2.x-p1.x);
  double C = (p2.x - p1.x)*(p3.y-p1.y)-(p3.x-p1.x)*(p2.y-p1.y);
  double D = -(A*p1.x + B*p1.y + C*p1.z);
  double z = -(A*pt.x + B*pt.y + D) / C;
  return pcl::PointXYZ(pt.x, pt.y, z);
}

static pcl::PointXYZ interpolate(const pcl::PointXY& pt,
                                 const std::vector<pcl::PointXYZ>& points) {
  if (points.size() == 1) {
    return points[0];
  } else if (points.size() == 2) {
    return interpolate(pt, points[0], points[1]);
  } else if (points.size() == 3) {
    return interpolate(pt, points[0], points[1], points[2]);
  }
}

static int numNeighboursAtLocation(const pcl::PointXY& query_pt,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                   double radius) {
  // Handle empty pointcloud case
  if (cloud->points.size() == 0) return 0;
  // Create an XY pointcloud
  pcl::PointCloud<pcl::PointXY>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXY>);
  pcl::copyPointCloud(*cloud, *cloud_xy);
  // Create a KD-Tree
  pcl::KdTreeFLANN<pcl::PointXY> tree;
  tree.setInputCloud(cloud_xy);
  std::vector<int> indexes;
  std::vector<float> sq_distances;
  tree.radiusSearch(query_pt, radius, indexes, sq_distances);
  return indexes.size();
}

static void randomlySample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
          int num_points,
          double radius_search,
          pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) {
  // Make sure cloud is empty
  cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>);
  // Create an XY pointcloud
  pcl::PointCloud<pcl::PointXY>::Ptr cloud_in_xy(new pcl::PointCloud<pcl::PointXY>);
  pcl::copyPointCloud(*cloud_in, *cloud_in_xy);
  // Create a KD-Tree
  pcl::KdTreeFLANN<pcl::PointXY> tree;
  tree.setInputCloud(cloud_in_xy);
  // Neighbors within radius search
  std::vector<int> point_idx(3);
  std::vector<float> point_sq_distance(3);

  // Get min and max points
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud_in, min_pt, max_pt);
  double range_x = max_pt.x - min_pt.x;
  double range_y = max_pt.y - min_pt.y;

  int orig_num_points = cloud_in->points.size();
  double orig_resolution = orig_num_points / (range_x*range_y);
  double curr_resolution = num_points / (range_x*range_y);
  int max_neigbours = orig_resolution / curr_resolution * 5;
  // std::cout << "max_neigbours " << max_neigbours << std::endl;

  int n = 0;
  int it = 0;

  boost::random::mt19937 gen;
  boost::random::uniform_real_distribution<double> dist(0.0, 1.0);

  while (n < num_points) {
    // Pick a random point
    pcl::PointXY sp;
    sp.x = min_pt.x + range_x*dist(gen);
    sp.y = min_pt.y + range_y*dist(gen);
    // Get nearest neighbours
    if (tree.nearestKSearch(sp, 3, point_idx, point_sq_distance) > 0) {
      bool valid_point = true;
      // Check that distances are less than radius_search
      for (size_t i = 0; i < 3; i++) {
        if (point_sq_distance[i] > radius_search) {
          valid_point = false;
        }
      }

      // Check that the neighbours form a triangle around the point
      pcl::PointXY p1 = cloud_in_xy->points[point_idx[0]];
      pcl::PointXY p2 = cloud_in_xy->points[point_idx[1]];
      pcl::PointXY p3 = cloud_in_xy->points[point_idx[2]];
      bool inside = pointInTriangle(sp, p1, p2, p3);

      // VERY SLOW!!
      // int neighbours = numNeighboursAtLocation(sp, cloud_out, radius_search);
      int neighbours = 1;

      // if (it % 1000 == 0) std::cout << "it " << it << " n " << n << std::endl;

      if (valid_point && inside && neighbours < max_neigbours) {
        // add point to output cloud
        pcl::PointXYZ p1 = cloud_in->points[point_idx[0]];
        pcl::PointXYZ p2 = cloud_in->points[point_idx[1]];
        pcl::PointXYZ p3 = cloud_in->points[point_idx[2]];
        pcl::PointXYZ p = interpolate(sp, p1, p2, p3);
        cloud_out->push_back(p);
        n++;
      }
    }

    if (it > num_points*1e4) {
      std::cout << "[WARN]: More than 10000 times normal iterations. Something is wrong here! (n is " << n << ")" << std::endl;
      break;
    }
    it++;
  }
}

static void randomlySampleGP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
          int num_points,
          double radius_search,
          pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) {
  // Prepare the alpha shape
  GreedyProjector gp(0.04);
  gp.setInputCloud(cloud_in);

  // Get min and max points
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud_in, min_pt, max_pt);
  double range_x = max_pt.x - min_pt.x;
  double range_y = max_pt.y - min_pt.y;

  // Generate random number between zero and one
  boost::random::mt19937 gen;
  boost::random::uniform_real_distribution<double> dist(0.0, 1.0);

  int n = 0;
  int it = 0;
  while (n < num_points) {
    // Pick a random point
    pcl::PointXY sp;
    sp.x = min_pt.x + range_x*dist(gen);
    sp.y = min_pt.y + range_y*dist(gen);
    std::vector<pcl::PointXYZ> points = gp.locate(sp);
    if (points.size() > 0) {
      pcl::PointXYZ op;
      op = interpolate(sp, points);
      cloud_out->push_back(op);
      n++;
    }
    if (it > num_points*1e4) {
      std::cout << "[WARN]: More than 10000 times normal iterations. Something is wrong here! (n is " << n << ")" << std::endl;
      break;
    }
    it++;
  }
}

static void voxelGrid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
          double grid_size,
          pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) {
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(grid_size, grid_size, grid_size);
  approximate_voxel_filter.setInputCloud(cloud_in);
  approximate_voxel_filter.filter(*cloud_out);
}

static bool normalDistributionsTransform(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
          Eigen::Matrix4f& transform,
          double score) {
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon(0.001);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize(0.05);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution(0.5);
  // Setting max number of registration iterations.
  ndt.setMaximumIterations(200);
  // Setting point cloud to be aligned.
  ndt.setInputSource(source);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget(target);
  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
  ndt.align(*aligned, transform);
  score = ndt.getFitnessScore();
  transform = ndt.getFinalTransformation();
  return ndt.hasConverged();
}

static bool icp(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
          Eigen::Matrix4f& transform,
          double& score) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaxCorrespondenceDistance(0.2);
  icp.setRANSACOutlierRejectionThreshold(0.4);
  icp.setTransformationEpsilon(0.001);
  icp.setEuclideanFitnessEpsilon(0.001);
  icp.setMaximumIterations(100);
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.align(*aligned);
  score = icp.getFitnessScore();
  transform = icp.getFinalTransformation();
  return icp.hasConverged();
}

static bool icpn(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& source,
    const pcl::PointCloud<pcl::PointNormal>::Ptr& target,
          Eigen::Matrix4f& transform,
          double& score) {
  pcl::PointCloud<pcl::PointNormal>::Ptr aligned_wn(new pcl::PointCloud<pcl::PointNormal>());
  pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icpn;
  icpn.setMaxCorrespondenceDistance(0.4);
  icpn.setRANSACOutlierRejectionThreshold(0.6);
  icpn.setTransformationEpsilon(0.001);
  icpn.setEuclideanFitnessEpsilon(0.001);
  icpn.setMaximumIterations(200);
  icpn.setInputSource(source);
  icpn.setInputTarget(target);
  icpn.align(*aligned_wn);
  score = icpn.getFitnessScore();
  transform = icpn.getFinalTransformation();
  return icpn.hasConverged();
}

static void estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                            pcl::PointCloud<pcl::Normal>::Ptr& normals,
                            double radius_search) {
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation_omp;
  normal_estimation_omp.setInputCloud(cloud);
  normal_estimation_omp.setRadiusSearch(radius_search);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_omp(new pcl::search::KdTree<pcl::PointXYZ>);
  normal_estimation_omp.setSearchMethod(kdtree_omp);
  normal_estimation_omp.compute(*normals);
}

static void addNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double normal_radius_search,
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals) {
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
  estimateNormals(cloud, cloud_normals, normal_radius_search);
  pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);
}

static void saveCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      const std::string& prefix,
                      int id) {
  std::string clouds_dir("../adjusted/");
  std::ostringstream os ;
  os << clouds_dir << prefix << "_" << id << ".pcd" ;
  pcl::io::savePCDFileASCII(os.str(), *cloud);
}

static void saveCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
                      const std::string& prefix,
                      int id) {
  std::string clouds_dir("../adjusted/");
  std::ostringstream os ;
  os << clouds_dir << prefix << "_" << id << ".pcd" ;
  pcl::io::savePCDFileASCII(os.str(), *cloud);
}

static void loadCloud(const std::string& folder,
                      const std::string& filename,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  pcl::io::loadPCDFile (folder + "/" + filename, *cloud);
}

static void smooth(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
          pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out) {
  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  std::cout << "Process IN " << std::endl;

  // Set parameters
  mls.setComputeNormals(true);
  mls.setInputCloud(cloud_in);
  mls.setPolynomialFit(true);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.2);

  // Reconstruct
  mls.process(*cloud_out);
  std::cout << "Process OUT " << std::endl;
}

static void segmentation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, pcl::PointCloud<pcl::PointXYZ>::Ptr segmented)
{
  std::cout << "segmentation..." << std::flush;
  // fit plane and keep points above that plane
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.2);

  seg.setInputCloud (source);
  seg.segment (*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (source);
  extract.setIndices (inliers);
  extract.setNegative (true);

  extract.filter (*segmented);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*segmented, *segmented, indices);
  std::cout << "OK" << std::endl;

  std::cout << "clustering..." << std::flush;
  // euclidean clustering
  typename pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (segmented);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
  clustering.setClusterTolerance (0.2); // 20cm
  clustering.setMinClusterSize (1000);
  clustering.setMaxClusterSize (250000);
  clustering.setSearchMethod (tree);
  clustering.setInputCloud(segmented);
  clustering.extract (cluster_indices);

  if (cluster_indices.size() > 0)//use largest cluster
  {
    std::cout << cluster_indices.size() << " clusters found";
    if (cluster_indices.size() > 1)
      std::cout <<" Using largest one...";
    std::cout << std::endl;
    typename pcl::IndicesPtr indices (new std::vector<int>);
    *indices = cluster_indices[0].indices;
    extract.setInputCloud (segmented);
    extract.setIndices (indices);
    extract.setNegative (false);

    extract.filter (*segmented);
  }
}

}  // namespace

#endif // PCL_TOOLS_H
