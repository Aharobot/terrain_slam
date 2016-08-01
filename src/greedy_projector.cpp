#include <terrain_slam/greedy_projector.h>

GreedyProjector::GreedyProjector()
  : alpha_(new Alpha_shape_2()),
  cloud_(new pcl::PointCloud<pcl::PointXYZ>()) {
  init_ = false;
  alpha_value_ = 0.1;
}

GreedyProjector::GreedyProjector(double av)
  : alpha_(new Alpha_shape_2()),
  cloud_(new pcl::PointCloud<pcl::PointXYZ>()) {
  init_ = false;
  alpha_value_ = av;
}

void GreedyProjector::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  pcl::copyPointCloud(*cloud, *cloud_);
  std::vector<std::pair<Point, unsigned> > point_pairs;
  for (size_t i = 0; i < cloud->size(); i++) {
    Point p(cloud->points[i].x, cloud->points[i].y);
    std::pair<Point,unsigned> point_pair(p, i);
    point_pairs.push_back(point_pair);
  }

  // Create alpha shape (concave hull)
  alpha_.reset(new Alpha_shape_2(point_pairs.begin(), point_pairs.end(), FT(alpha_value_), Alpha_shape_2::GENERAL));
  init_ = true;
}

std::vector<pcl::PointXYZ> GreedyProjector::locate(const pcl::PointXYZ& pt) {
  pcl::PointXY ptxy;
  ptxy.x = pt.x;
  ptxy.y = pt.y;
  return locate(ptxy);
}

/**
 * @brief      Locate a point inside the hull
 *
 * @param[in]  pt    The point
 *
 * @return     Vector of surrounding points
 */
std::vector<pcl::PointXYZ> GreedyProjector::locate(const pcl::PointXY& pt) {
  std::vector<pcl::PointXYZ> output;
  if (!init_) {
    std::cout << "[GreedyProjector] Set input cloud first!" << std::endl;
    return output;
  }

  Point point(pt.x, pt.y);
    // Check that the point belongs to the alpha complex
  Alpha_shape_2::Classification_type c = alpha_->classify(point);

  if (c != Alpha_shape_2::EXTERIOR) {
    Locate_type loc;
    Face_handle handle;
    int vertex_or_edge_idx;
    // Locate the point in the mesh
    handle = alpha_->locate(point, loc, vertex_or_edge_idx);

    int v0, v1, v2;
    if(loc == Alpha_shape_2::VERTEX) {
      // std::cout << "Delaunay::VERTEX " << std::endl;
      v0 = handle->vertex(0)->info();
      pcl::PointXYZ p0 = cloud_->points[v0];
      output.push_back(p0);
    } else if(loc == Alpha_shape_2::EDGE) {
      // std::cout << "Delaunay::EDGE " << std::endl;
      v0 = handle->vertex(0)->info();
      v1 = handle->vertex(1)->info();
      pcl::PointXYZ p0 = cloud_->points[v0];
      pcl::PointXYZ p1 = cloud_->points[v1];
      output.push_back(p0);
      output.push_back(p1);
    } else if(loc == Alpha_shape_2::FACE) {
      // std::cout << "Delaunay::FACE " << std::endl;
      v0 = handle->vertex(0)->info();
      v1 = handle->vertex(1)->info();
      v2 = handle->vertex(2)->info();
      pcl::PointXYZ p0 = cloud_->points[v0];
      pcl::PointXYZ p1 = cloud_->points[v1];
      pcl::PointXYZ p2 = cloud_->points[v2];
      output.push_back(p0);
      output.push_back(p1);
      output.push_back(p2);
    } else if(loc == Alpha_shape_2::OUTSIDE_CONVEX_HULL){
      // std::cout << "Delaunay::OUTSIDE_CONVEX_HULL " << std::endl;
    } else if(loc == Alpha_shape_2::OUTSIDE_AFFINE_HULL){
      // std::cout << "Delaunay::OUTSIDE_AFFINE_HULL " << std::endl;
    } else {
      std::cout << "Serious error: unknown Locate_type: " << loc << std::endl;
    }
  }
  return output;
}

/**
 * @brief      Determines if inside.
 *
 * @param[in]  pt    The point
 *
 * @return     True if inside, False otherwise.
 */
bool GreedyProjector::isInside(const pcl::PointXYZ& pt) {
  pcl::PointXY pt2;
  pt2.x = pt.x;
  pt2.y = pt.y;
  return isInside(pt2);
}

double GreedyProjector::area() {
  double acc = 0;
  for (Finite_faces_iterator it = alpha_->finite_faces_begin(); it != alpha_->finite_faces_end(); it++) {
    Alpha_shape_2::Face_handle face = it;
    acc += alpha_->triangle(face).area();
  }
  return acc;
}

/**
 * @brief      Determines if inside.
 *
 * @param[in]  pt    The point
 *
 * @return     True if inside, False otherwise.
 */
bool GreedyProjector::isInside(const pcl::PointXY& pt) {
  if (!init_) {
    std::cout << "[GreedyProjector] Set input cloud first!" << std::endl;
    return false;
  }

  Point point(pt.x, pt.y);
    // Check that the point belongs to the alpha complex
  Alpha_shape_2::Classification_type c = alpha_->classify(point);

  if (c != Alpha_shape_2::EXTERIOR) {
    Locate_type loc;
    Face_handle handle;
    int vertex_or_edge_idx;
    // Locate the point in the mesh
    handle = alpha_->locate(point, loc, vertex_or_edge_idx);
    if(loc == Alpha_shape_2::VERTEX || loc == Alpha_shape_2::EDGE || loc == Alpha_shape_2::FACE) {
      return true;
    }
  }
  return false;
}

void GreedyProjector::save(void) {
  std::cout << "Writting points to ply...   "  << std::endl;
  std::stringstream interior;
  std::stringstream exterior;
  for (size_t i = 0; i < cloud_->size(); i++) {
    interior << cloud_->points[i].x << " " << cloud_->points[i].y << " " << cloud_->points[i].z << "\n";
    exterior << cloud_->points[i].x << " " << cloud_->points[i].y << " " << cloud_->points[i].z << "\n";
  }

  int num_faces = alpha_->number_of_faces();
  // AlphaDelaunay::Finite_faces_iterator it;
  int counter = 0;
  // for (it = dt.finite_faces_begin(); it != dt.finite_faces_end(); it++) {
  for (Face_iterator it = alpha_->finite_faces_begin(); it != alpha_->finite_faces_end(); it++) {
    // Alpha_shape_2::Face_handle fh = *it;
    Alpha_shape_2::Classification_type c = alpha_->classify(it);
    if (c != Alpha_shape_2::EXTERIOR) {
      int idx0 = it->vertex(0)->info();
      int idx1 = it->vertex(1)->info();
      int idx2 = it->vertex(2)->info();
      interior << "3 " << idx0 << " "  << idx1 << " "  << idx2 << std::endl;
      counter++;
    } else {
      int idx0 = it->vertex(0)->info();
      int idx1 = it->vertex(1)->info();
      int idx2 = it->vertex(2)->info();
      exterior << "3 " << idx0 << " "  << idx1 << " "  << idx2 << std::endl;
    }
  }

  ofstream interior_plyfile;
  interior_plyfile.open("interior.ply");
  interior_plyfile << "ply\n"
     << "format ascii 1.0\n"
     << "element vertex " << cloud_->size() << "\n"
     << "property float x\n"
     << "property float y\n"
     << "property float z\n"
     << "element face " << counter << "\n"
     << "property list uchar int vertex_index\n"
     << "end_header\n" << std::flush; //endl;
  interior_plyfile << interior.str();
  interior_plyfile.close();

  ofstream exterior_plyfile;
  exterior_plyfile.open("exterior.ply");
  exterior_plyfile << "ply\n"
     << "format ascii 1.0\n"
     << "element vertex " << cloud_->size() << "\n"
     << "property float x\n"
     << "property float y\n"
     << "property float z\n"
     << "element face " << num_faces - counter << "\n"
     << "property list uchar int vertex_index\n"
     << "end_header\n" << std::flush; //endl;
  exterior_plyfile << exterior.str();
  exterior_plyfile.close();
}
