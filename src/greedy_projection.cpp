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
typedef Alpha_shape_2::Face_iterator                             Face_iterator;
typedef CGAL::Triangulation_2<K, ATds>::Locate_type              Locate_type;

int main (int argc, char** argv) {
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile (argv[1], cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);

  std::vector<std::pair<Point, unsigned> > point_pairs;
  std::vector<Point> points;
  for (size_t i = 0; i < cloud->size(); i++) {
    Point p(cloud->points[i].x, cloud->points[i].y);
    std::pair<Point,unsigned> point_pair(p, i);
    point_pairs.push_back(point_pair);
    points.push_back(p);
  }

  // Create alpha shape (concave hull)
  Alpha_shape_2 alpha(point_pairs.begin(), point_pairs.end(), FT(0.1), Alpha_shape_2::GENERAL);

  // Locate a point inside the hull
  Locate_type loc;
  Face_handle handle;
  int li;
  Point point(1.0, 0.1);
  handle = alpha.locate(point, loc, li);

  int v0, v1, v2;

  if(loc == Alpha_shape_2::VERTEX) {
    std::cout << "Delaunay::VERTEX " << std::endl;
    v0 = handle->vertex(li)->info();

  } else if(loc == Alpha_shape_2::EDGE) {
    std::cout << "Delaunay::EDGE " << std::endl;
    v0 = handle->vertex(handle->cw(li))->info();
    v1 = handle->vertex(handle->ccw(li))->info();

  } else if(loc == Alpha_shape_2::FACE) {
    std::cout << "Delaunay::FACE " << std::endl;
    v0 = handle->vertex(0)->info();
    v1 = handle->vertex(1)->info();
    v2 = handle->vertex(2)->info();

    pcl::PointXYZ p0 = cloud->points[v0];
    pcl::PointXYZ p1 = cloud->points[v1];
    pcl::PointXYZ p2 = cloud->points[v2];
    std::cout << "Between (" << p0.x << ", " << p0.y << ", " << p0.z << ") (" << p1.x << ", " << p1.y << ", " << p1.z << ") (" << p2.x << ", " << p2.y << ", " << p2.z << ")"<< std::endl;

  } else if(loc == Alpha_shape_2::OUTSIDE_CONVEX_HULL){
    std::cout << "Delaunay::OUTSIDE_CONVEX_HULL " << std::endl;

  } else if(loc == Alpha_shape_2::OUTSIDE_AFFINE_HULL){
    std::cout << "Delaunay::OUTSIDE_AFFINE_HULL " << std::endl;


  } else {
    std::cout << "Serious error: unknown Locate_type: " << loc << std::endl;
  }

  ofstream inliers_plyfile;
  ofstream outliers_plyfile;
  std::stringstream inliers;
  std::stringstream outliers;

  std::cout << "Writting points to ply...   "  << std::flush;
  for (size_t i = 0; i < cloud->size(); i++) {
    inliers << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << "\n";
    outliers << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << "\n";
  }

  std::cout << "Writing triangles to ply file...   "  << std::flush;

  int num_faces = alpha.number_of_faces();
  // AlphaDelaunay::Finite_faces_iterator it;
  int counter = 0;
  // for (it = dt.finite_faces_begin(); it != dt.finite_faces_end(); it++) {
  for (Face_iterator it = alpha.finite_faces_begin(); it != alpha.finite_faces_end(); it++) {
    // Alpha_shape_2::Face_handle fh = *it;
    Alpha_shape_2::Classification_type c = alpha.classify(it);
    if (c != Alpha_shape_2::EXTERIOR) {
      int idx0 = it->vertex(0)->info();
      int idx1 = it->vertex(1)->info();
      int idx2 = it->vertex(2)->info();
      inliers << "3 " << idx0 << " "  << idx1 << " "  << idx2 << std::endl;
      counter++;
    } else {
      int idx0 = it->vertex(0)->info();
      int idx1 = it->vertex(1)->info();
      int idx2 = it->vertex(2)->info();
      outliers << "3 " << idx0 << " "  << idx1 << " "  << idx2 << std::endl;
    }
  }

  std::cout << "...done writing " << counter << "/" << num_faces << " triangles."  << std::endl;

  inliers_plyfile.open("triangles.ply");
  inliers_plyfile << "ply\n"
     << "format ascii 1.0\n"
     << "element vertex " << point_pairs.size() << "\n"
     << "property float x\n"
     << "property float y\n"
     << "property float z\n"
     << "element face " << counter << "\n"
     << "property list uchar int vertex_index\n"
     << "end_header\n" << std::flush; //endl;
  inliers_plyfile << inliers.str();
  inliers_plyfile.close();

  outliers_plyfile.open("outliers.ply");
  outliers_plyfile << "ply\n"
     << "format ascii 1.0\n"
     << "element vertex " << point_pairs.size() << "\n"
     << "property float x\n"
     << "property float y\n"
     << "property float z\n"
     << "element face " << num_faces - counter << "\n"
     << "property list uchar int vertex_index\n"
     << "end_header\n" << std::flush; //endl;
  outliers_plyfile << outliers.str();
  outliers_plyfile.close();

  return (0);
}
