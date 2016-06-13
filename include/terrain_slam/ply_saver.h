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


#ifndef PLY_SAVER_H
#define PLY_SAVER_H

#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class PlySaver {
public:
  PlySaver() : nan_counter_(0) {}

  void saveCloud(const std::string &filename,
                 const std::vector<Eigen::Vector3d> &points) {
    std::ofstream file;
    file.open(filename.c_str(), std::ofstream::out);
    writeHeader(file, points.size());
    for (size_t i = 0; i < points.size(); i++)
      writePoint(file, points[i]);

    if (nan_counter_ > 0)
      std::cerr << "[WARN] There are " << nan_counter_ << " NANs!" << std::endl;
  }

  void saveCloud(const std::string &filename, const Eigen::Matrix3Xd &points) {
    std::ofstream file;
    file.open(filename.c_str(), std::ofstream::out);
    writeHeader(file, points.cols());
    for (size_t i = 0; i < points.cols(); i++)
      writePoint(file, points.col(i));
    if (nan_counter_ > 0)
      std::cerr << "[WARN] There are " << nan_counter_ << " NANs!" << std::endl;
  }

  void saveCloud(const std::string &filename, const Eigen::Matrix4Xd &points) {
    std::ofstream file;
    file.open(filename.c_str(), std::ofstream::out);
    writeHeader(file, points.cols());
    for (size_t i = 0; i < points.cols(); i++)
      writePoint(file, points.block<3, 1>(0, i));
    if (nan_counter_ > 0)
      std::cerr << "[WARN] There are " << nan_counter_ << " NANs!" << std::endl;
  }

private:
  int nan_counter_;

  void writeHeader(std::ofstream &file, int num_points) {
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << num_points << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";
  }

  void writePoint(std::ofstream &file, const Eigen::Vector3d &p) {
    if (isnan(p(0)) || isnan(p(1)) || isnan(p(2))) {
      nan_counter_++;
    } else {
      file << p(0) << " " << p(1) << " " << p(2) << "\n";
    }
  }
};

#endif // PLY_SAVER_H
