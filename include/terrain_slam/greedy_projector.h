#ifndef GREEDY_PROJECTOR_H
#define GREEDY_PROJECTOR_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Alpha_shape_2.h>

#include <boost/shared_ptr.hpp>

typedef CGAL::Exact_predicates_inexact_constructions_kernel      K;
typedef K::FT                                                    FT;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned, K> Vb;
typedef CGAL::Alpha_shape_vertex_base_2<K, Vb>                   AVb;
typedef CGAL::Alpha_shape_face_base_2<K>                         AFb;
typedef CGAL::Triangulation_data_structure_2<AVb,AFb>            ATds;
typedef CGAL::Delaunay_triangulation_2<K,ATds>                   AlphaDelaunay;
typedef CGAL::Alpha_shape_2<AlphaDelaunay>                       Alpha_shape_2;
typedef Alpha_shape_2::Point                                     Point;
typedef Alpha_shape_2::Face_handle                               Face_handle;
typedef Alpha_shape_2::Finite_faces_iterator                     Finite_faces_iterator;
typedef Alpha_shape_2::Face_iterator                             Face_iterator;
typedef CGAL::Triangulation_2<K, ATds>::Locate_type              Locate_type;

class GreedyProjector {
 public:
  GreedyProjector();
  GreedyProjector(double av);
  void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  std::vector<pcl::PointXYZ> locate(const pcl::PointXYZ& pt) const;
  std::vector<pcl::PointXYZ> locate(const pcl::PointXY& pt) const;
  std::vector<Eigen::Vector4d> locate(const Eigen::Vector4d& pt) const;
  void save(void);
  bool isInside(const pcl::PointXYZ& pt) const;
  bool isInside(const pcl::PointXY& pt) const;
  bool isInside(const Eigen::Vector4d& pt) const;
  double area();

 private:
  bool init_;
  double alpha_value_;
  boost::shared_ptr<Alpha_shape_2> alpha_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};

#endif // GREEDY_PROJECTOR_H