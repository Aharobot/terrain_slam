
#include <terrain_slam/clouds.h>

#include <pcl/common/transforms.h>

void terrain_slam::CloudPatch::add(const CloudPatch& other_patch) {
  Eigen::Matrix4d tf_other = other_patch.transform();
  Eigen::Matrix4d tf = transform().inverse()*tf_other;
  Cloud transformed;
  pcl::transformPointCloud(*other_patch.cloud, transformed, tf.cast<float>());
  add(transformed);
}

void terrain_slam::CloudPatch::add(const Cloud& other_cloud) {
  *cloud += other_cloud;
}

void terrain_slam::CloudPatch::add(const Point& other_point) {
  cloud->push_back(other_point);
}

void terrain_slam::CloudPatch::add(const Eigen::Vector3d& other_point) {
  Point p(other_point(0), other_point(1), other_point(2));
  cloud->push_back(p);
}

void terrain_slam::CloudPatch::add(const Eigen::Vector4d& other_point) {
  Point p(other_point(0), other_point(1), other_point(2));
  cloud->push_back(p);
}

Eigen::Vector3d terrain_slam::CloudPatch::getCentroid() const {
  Eigen::Vector3d pe;
  for (size_t i = 0; i < cloud->size(); i++) {
    Point pt = cloud->points[i];
    pe += Eigen::Vector3d(pt.x, pt.y, pt.z);
  }
  return pe/cloud->size();
}

std::vector<Eigen::Vector4d>
terrain_slam::CloudPatch::kNN(const Eigen::Vector4d& p, int n) const {
  std::vector<Eigen::Vector4d> points;
  std::vector<int> point_idx(n);
  std::vector<float> point_sq_distance(n);
  pcl::PointXY sp;
  sp.x = p(0);
  sp.y = p(1);
  kdtree.nearestKSearch(sp, n, point_idx, point_sq_distance);
  for (size_t i = 0; i < point_idx.size(); i++) {
    Point pt = cloud->points[point_idx[i]];
    points.push_back(Eigen::Vector4d(pt.x, pt.y, pt.z, 1.0));
    // if (std::abs(pt.x - sp.x) < 0.5 && std::abs(pt.y - sp.y) < 0.5) {
    //   std::ostringstream out;
    //   out << "Looking for point (" << sp.x << ", " << sp.y << ") its knn is (" << pt.x << ", " << pt.y << ", " << pt.z << ")";
    //   std::cout << out.str() << std::endl;
    // }
  }
  return points;
}