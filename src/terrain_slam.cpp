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

#include <terrain_slam/terrain_slam.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>

namespace po = boost::program_options;
using namespace std;
using namespace cv;

terrain_slam::TerrainSlam::TerrainSlam(int argc, char **argv) {
  parseCommandLine(argc, argv);
}

void terrain_slam::TerrainSlam::parseCommandLine(int argc, char **argv) {
  try {
    po::options_description description("Terrain Slam");

    string pose_filename;
    string clouds_dir;
    double size = 5.0;

    description.add_options()("help,h", "Display this help message")(
        "clouds,c", po::value<string>(),
        "Input folder where the point clouds are")(
        "size,s", po::value<double>()->default_value(size),
        "Patch size in meters");

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(description).run(),
              vm);
    po::notify(vm);

    if (vm.count("help")) {
      cout << description;
    } else if (vm.count("clouds")) {
      clouds_dir = vm["clouds"].as<string>();
      if (vm.count("size"))
        size = vm["size"].as<double>();
      std::cout.setf(std::ios::boolalpha);
      cout << "Running Terrain Slam with the following parameters:"
           << "\n\t* Clouds directory  : " << clouds_dir << endl;
      process(clouds_dir, size);
    } else {
      cout << "REQUIRED arguments were not provided." << endl;
      cout << description;
    }
  } catch (po::error e) {
    cerr << "Error: " << e.what() << ". Aborting" << endl;
  }
}

void terrain_slam::TerrainSlam::process(const std::string &clouds_dir,
                                        double size) {
  // Set patch size
  patch_size_ = size;

  // Retrieve list of files
  vector<string> cloud_names;
  vector<string> cloud_paths;
  bool files_found = getCloudPaths(clouds_dir, "yml", cloud_names, cloud_paths);

  if (files_found) {
    // Read files and load them in memory
    readFiles(cloud_names, cloud_paths);

    // Do the job!
    processPatches();
  }
}

bool terrain_slam::TerrainSlam::getCloudPaths(const string &path,
                                              const string &format,
                                              vector<string> &cloud_names,
                                              vector<string> &cloud_paths) {
  boost::filesystem::path p(path);
  typedef vector<boost::filesystem::path> vec; // store paths,
  vec v;                                       // so we can sort them later
  try {
    if (exists(p)) {
      if (is_directory(p)) {
        copy(boost::filesystem::directory_iterator(p),
             boost::filesystem::directory_iterator(), back_inserter(v));
        sort(v.begin(), v.end()); // sort, since directory iteration
                                  // is not ordered on some file systems
      } else {
        cout << p << " exists, but is neither a regular file nor a directory\n";
      }
    } else {
      cout << p << " does not exist\n";
    }
  } catch (const boost::filesystem::filesystem_error &ex) {
    cout << ex.what() << '\n';
  }

  for (size_t i = 0; i < v.size(); i++) {
    std::string full_cloud_path(v[i].string());
    std::string name(v[i].stem().string());
    std::string extension(v[i].extension().string());
    if (extension == "." + format) {
      cloud_names.push_back(name);
      cloud_paths.push_back(full_cloud_path);
    }
  }

  if (cloud_names.size() == 0) {
    std::cout << "No clouds found. Aborting... " << std::endl;
    return false;
  } else {
    std::cout << "Found " << cloud_names.size() << " clouds!" << std::endl;
    return true;
  }
}

void terrain_slam::TerrainSlam::readFiles(const vector<string> &cloud_names,
                                          const vector<string> &cloud_paths) {
  std::cout << "Reading clouds... " << std::endl;
  for (size_t i = 0; i < cloud_paths.size(); i++) {
    FileStorage fs;
    fs.open(cloud_paths[i], FileStorage::READ);

    if (fs.isOpened()) {
      std::vector<cv::Point3d> points;
      cv::Point3d robot_position, robot_orientation, camera_position,
          camera_orientation;
      fs["points"] >> points;
      fs["robot_position"] >> robot_position;
      fs["robot_orientation"] >> robot_orientation;
      fs["camera_position"] >> camera_position;
      fs["camera_orientation"] >> camera_orientation;

      int num_points = points.size();
      Eigen::Vector3d xyz = cv2eigen(robot_position);
      Eigen::Vector3d rpy = cv2eigen(robot_orientation) * M_PI / 180.0;
      LaserLine line(num_points, xyz, rpy);
      for (size_t j = 0; j < points.size(); j++) {
        line.add(cv2eigen(points[j]), j);
      }
      lines_.push_back(line);
    } else {
      cout << "Unable to open camera pose file at " << cloud_paths[i]
           << ".  Please verify the path." << endl;
    }
  }
  std::cout << "Clouds loaded! " << std::endl;
}

void terrain_slam::TerrainSlam::processPatches() {
  double distance = 0;
  int pivot_idx = 0;

  std::cout << "Processing... " << std::endl;
  for (size_t i = 0; i < lines_.size(); i++) {
    // Accumulate clouds while distance is less than patch_size_
    Eigen::Vector3d pos1 = lines_[pivot_idx].getPosition();
    Eigen::Vector3d pos2 = lines_[i].getPosition();
    Eigen::Vector3d diff = pos2 - pos1;
    distance = diff.norm();
    if (distance > patch_size_) {

      CloudPatch patch(lines_, pivot_idx, i);
      patches_.push_back(patch);

      savePatch(patch, pivot_idx);
      pivot_idx = i + 1;
    }
  }
}

// Save the patch with the position information
void terrain_slam::TerrainSlam::savePatch(
    const CloudPatch& patch, int idx) {
  std::string path("../output");
  boost::filesystem::path dir(path);
  boost::filesystem::create_directory(dir);

  std::ostringstream ss;
  ss << std::setw(8) << std::setfill('0') << idx;
  std::string cloud_filename = path + "/cloud" + ss.str() + ".ply";
  std::cout << "Saving ply file " << cloud_filename << std::endl;
  saver_.saveCloud(cloud_filename, patch.getPoints());
}

int main(int argc, char **argv) {
  terrain_slam::TerrainSlam tslam(argc, argv);
  return 0;
}