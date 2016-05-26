#ifndef PLY_SAVER_H
#define PLY_SAVER_H

#include <Eigen/Geometry>
#include <fstream>
#include <string>
#include <vector>

class PlySaver {
 public:
  PlySaver() {}

  void saveCloud(const std::string& filename,
                 const std::vector<Eigen::Vector3d>& points) {
    std::ofstream file;
    file.open(filename.c_str(), std::ofstream::out);
    writeHeader(file, points.size());
    for (size_t i = 0; i < points.size(); i++)
      writePoint(file, points[i]);
  }

 private:

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
    file << p(0) << " " << p(1) << " " << p(2) << "\n";
  }
};

#endif // PLY_SAVER_H
