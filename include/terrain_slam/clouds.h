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

#ifndef CLOUDS_H
#define CLOUDS_H

#include <boost/shared_ptr.hpp>

// Generic pcl
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace terrain_slam {

class Transform {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix4d T;

  Transform(){
    T = Eigen::Matrix4d::Identity();
  }

  Transform(const Eigen::Matrix4d& m){
    T = m;
  }

  Transform& operator=(const Eigen::Matrix4d& m) {
    T = m;
    // by convention, always return *this
    return *this;
  }

  Transform& operator()(const Eigen::Matrix4d& m) {
    T = m;
    // by convention, always return *this
    return *this;
  }

  Eigen::Matrix4d operator()() const {
    return T;
  }

  double& operator()(int row, int col) {
    assert(col >= 0 && col < 4);
    assert(row >= 0 && row < 4);
    return T(row, col);
  }

  const double& operator()(int row, int col) const {
    assert(col >= 0 && col < 4);
    assert(row >= 0 && row < 4);
    return T(row, col);
  }

  double roll() const {
    Eigen::Vector3d rpy = rotation().eulerAngles(0, 1, 2);
    return rpy(0);
  }

  double pitch() const {
    Eigen::Vector3d rpy = rotation().eulerAngles(0, 1, 2);
    return rpy(1);
  }

  double yaw() const {
    Eigen::Vector3d rpy = rotation().eulerAngles(0, 1, 2);
    return rpy(2);
  }

  double x() const { return T(0, 3); }
  double y() const { return T(1, 3); }
  double z() const { return T(2, 3); }
  Eigen::Matrix3d rotation() const { return T.block<3,3>(0,0); }
  Eigen::Vector3d origin() const { return T.block<3,1>(0,3); }
};

class Vertex {
 public:
  Vertex() : id(-1) {}
  Vertex(int n) : id(n) {}
  Vertex(int n, const std::string& s) : id(n), name(s) {}
  inline void setId(int n) { id = n; }
  inline int getId() const { return id;}
  inline void setName(const std::string& s) {name = s;}
  inline std::string getName() const {return name;}

 protected:
  int id;
  std::string name;
};

class CloudPatch: public Vertex {
 public:
  // Definitions
  typedef pcl::PointXYZ                 Point;
  typedef pcl::Normal                   Normal;
  typedef pcl::PointNormal              PointNormal;
  typedef pcl::PointCloud<Point>        Cloud;
  typedef pcl::PointCloud<Normal>       CloudNormal;
  typedef pcl::PointCloud<PointNormal>  CloudPointNormal;

  Cloud::Ptr cloud;
  Cloud::Ptr cloud2;
  pcl::PointCloud<pcl::PointXY>::Ptr cloud2d;
  pcl::KdTreeFLANN<pcl::PointXY> kdtree2d;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  Transform transform;
  bool processed;

  CloudPatch() : cloud(new Cloud), cloud2d(new pcl::PointCloud<pcl::PointXY>()), processed(false) {}

  void add(const CloudPatch& other_patch);
  void add(const Cloud& other_cloud);
  void add(const Point& other_point);
  void add(const Eigen::Vector3d& other_point);
  void add(const Eigen::Vector4d& other_point);
  Eigen::Vector3d getCentroid() const;
  void fitLine(double &stddev) const;
  void getMeanStd(double &mean, double &stddev) const;

  std::vector<Eigen::Vector4d> kNN(const Eigen::Vector4d& p, int n) const;
  std::vector<Eigen::Vector4d> kNN2d(const Eigen::Vector4d& p, int n) const;

  inline size_t size() const { return cloud->points.size(); }

  inline Eigen::Vector4d at(int i) const {
    return Eigen::Vector4d(cloud->points[i].x,
                           cloud->points[i].y,
                           cloud->points[i].z,
                           1);
  }

  inline void updateSearchTree() {
    pcl::copyPointCloud(*cloud, *cloud2d);
    kdtree2d.setInputCloud(cloud2d);
    kdtree.setInputCloud(cloud);
  }

};

typedef boost::shared_ptr<CloudPatch> CloudPatchPtr;
typedef const boost::shared_ptr<CloudPatch> CloudPatchConstPtr;

}   // Namespace

#endif // CLOUDS_H
