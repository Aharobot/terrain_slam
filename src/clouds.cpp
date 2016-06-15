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

#include <terrain_slam/ply_saver.h>
#include <terrain_slam/clouds.h>

#include <boost/filesystem.hpp>

#include <fstream>
#include <iostream>
#include <iomanip>


terrain_slam::CloudPatch::CloudPatch(const std::vector<LaserLine>& lines, int start_idx, int end_idx)
    : Transform(lines[start_idx].tf()), gridded_(false), start_idx_(start_idx), end_idx_(end_idx), id_(-1) {
  for (size_t i = start_idx; i <= end_idx; i++)
    add(lines[i]);
}
void terrain_slam::CloudPatch::add(const LaserLine& line) {
  // First line is the TF of the CloudPatch
  if (lines_.size() == 0) {
    T = line.tf();
  }
  // save the LaserLine
  lines_.push_back(line);

  // transform the points to the local tf and add them
  Eigen::Matrix4Xd new_points = transform(line);
  // std::cout << "concatenate to points " << std::endl;
  Eigen::Matrix4Xd cat_points(points.rows(), points.cols()+new_points.cols());
  cat_points << points, new_points;
  points = cat_points;
}

void terrain_slam::CloudPatch::grid(double resolution, double search_radius) {
  if (!gridded_) {
    terrain_slam::Gridder g(resolution, search_radius);
    g.setInput(points);
    grid_ = g.grid();
    nns_.reset(Nabo::NNSearchD::createKDTreeLinearHeap(grid_));
    gridded_ = true;
  }
}


std::vector<Eigen::Vector4d> terrain_slam::CloudPatch::kNN(const Eigen::Vector4d& q, int k) const {
  Eigen::VectorXi indices(k);
  Eigen::VectorXd dists2(k);

  nns_->knn(q, indices, dists2, k);
  std::vector<Eigen::Vector4d> nn;

  for (size_t i = 0; i < k; i++) {
    nn.push_back(grid_.col(indices(i)));
  }
  return nn;
}

Eigen::Matrix4Xd terrain_slam::CloudPatch::getPoints(bool local_coordinate_frame) const {
  Eigen::Matrix4Xd m(points.rows(), points.cols());
  for (int i = 0; i < points.cols(); i++) {
    if (local_coordinate_frame) {
      m.col(i) = points.col(i);
    } else {
      m.col(i) = tf()*points.col(i);
    }
  }
  return m;
}

Eigen::Vector3d terrain_slam::CloudPatch::getCentroid() const {
  Eigen::Vector4d c(0, 0, 0, 1);
  c = points.rowwise().sum();
  c = c / (double)points.cols();
  c = tf()*c;
  Eigen::Vector3d v = c.hnormalized();
  return v;
}

void terrain_slam::CloudPatch::save(int idx,
          const std::string path,
          const std::string suffix,
          bool local_coordinate_frame) {
  boost::filesystem::path dir(path);
  boost::filesystem::create_directory(dir);
  std::ostringstream ss;
  ss << std::setw(4) << std::setfill('0') << idx;
  std::string cloud_filename = path + "/cloud" + ss.str() + suffix + ".ply";
  std::cout << "[INFO]: Saving ply file " << cloud_filename << std::endl;
  PlySaver saver;
  saver.saveCloud(cloud_filename, getPoints(local_coordinate_frame));
}


Eigen::Matrix4Xd terrain_slam::CloudPatch::transform(const LaserLine& line) {
  Eigen::Matrix4Xd m(line.points.rows(), line.points.cols());
  for (int i = 0; i < line.points.cols(); i++) {
    if (lines_.size() == 1) {
      // set first line at origin
      m.col(i) = line.points.col(i);
    } else {
      // and the rest
      m.col(i) = tf().inverse()*line.tf()*line.points.col(i);
    }
  }
  return m;
}

