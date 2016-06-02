#ifndef PLY_SAVER_H
#define PLY_SAVER_H

#include <Eigen/Geometry>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>

class PlySaver {
 public:
  PlySaver() : nan_counter_(0) {}

  void saveCloud(const std::string& filename,
                 const std::vector<Eigen::Vector3d>& points) {
    std::ofstream file;
    file.open(filename.c_str(), std::ofstream::out);
    writeHeader(file, points.size());
    for (size_t i = 0; i < points.size(); i++)
      writePoint(file, points[i]);

    if (nan_counter_ > 0)
      std::cerr << "[WARN] There are " << nan_counter_ << " NANs!" << std::endl;
  }

  void saveCloud(const std::string& filename,
                 const Eigen::Matrix4Xd& points) {
    std::ofstream file;
    file.open(filename.c_str(), std::ofstream::out);
    writeHeader(file, points.cols());
    for (size_t i = 0; i < points.cols(); i++)
      writePoint(file, points.block<3, 1>(0, i).transpose());
  }

 private:

  int nan_counter_;

  void writeHeader(std::ofstream& file, int num_points) {
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << num_points << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";
  }

  void writePoint(std::ofstream& file, const Eigen::Vector3d& p) {
    if (isnan(p(0)) || isnan(p(1)) || isnan(p(2))) {
      nan_counter_++;
    } else {
      file << p(0) << " " << p(1) << " " << p(2) << "\n";
    }
  }
};

#endif // PLY_SAVER_H
