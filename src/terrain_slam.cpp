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

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <iostream>

namespace po = boost::program_options;
using namespace std;
using namespace cv;

terrain_slam::TerrainSlam::TerrainSlam(int argc, char** argv) {
  parseCommandLine(argc, argv);
}

void terrain_slam::TerrainSlam::parseCommandLine(int argc, char** argv) {
  try {
    po::options_description description("Terrain Slam");

    string pose_filename;
    string clouds_dir;

    description.add_options()
    ("help,h", "Display this help message")
    ("clouds,c", po::value<string>(), "Input folder where the point clouds are");

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(description).run(), vm);
    //po::notify(vm);

    if (vm.count("help")) {
      cout << description;
    } else if (vm.count("clouds")) {
      clouds_dir = vm["clouds"].as<string>();
      std::cout.setf(std::ios::boolalpha);
      cout << "Running Terrain Slam with the following parameters:"
         << "\n\t* Clouds directory  : " << clouds_dir << endl;
      process(clouds_dir);
    } else {
      cout << "REQUIRED arguments were not provided." << endl;
      cout << description;
    }
  } catch(po::error e) {
    cerr << "Error: " << e.what() << ". Aborting" << endl;
  }
}

void terrain_slam::TerrainSlam::process(const std::string& clouds_dir) {
  vector<string> cloud_names;
  vector<string> cloud_paths;
  getCloudPaths(clouds_dir, "yml", cloud_names, cloud_paths);
  bool success = readFiles(cloud_names, cloud_paths);
  // vector<vector<Point3> > points = readPointClouds(clouds_dir);
}

void terrain_slam::TerrainSlam::getCloudPaths(const string& path,
                                              const string& format,
                                              vector<string>& cloud_names,
                                              vector<string>& cloud_paths) {
  boost::filesystem::path p(path);
  std::vector<boost::filesystem::directory_entry> file_entries;
  try {
    if (boost::filesystem::exists(p)) {
      if (boost::filesystem::is_regular_file(p)) {
      } else if (boost::filesystem::is_directory(p)) {
        std::copy(boost::filesystem::recursive_directory_iterator(p),
                  boost::filesystem::recursive_directory_iterator(),
                  std::back_inserter(file_entries));
      } else {
        std::cout << p <<
          " exists, but is neither a regular file nor a directory\n";
      }
    } else {
      std::cout << p << " does not exist\n";
    }
  } catch (const boost::filesystem::filesystem_error& ex) {
    std::cout << ex.what() << '\n';
  }

  for (size_t i = 0; i < file_entries.size(); i++) {
    std::string full_cloud_path(file_entries[i].path().string());
    std::string name(file_entries[i].path().stem().string());
    std::string extension(file_entries[i].path().extension().string());
    if (extension == "." + format) {
      cloud_names.push_back(name);
      cloud_paths.push_back(full_cloud_path);
    }
  }
}

bool terrain_slam::TerrainSlam::readFiles(const vector<string>& cloud_names,
                                          const vector<string>& cloud_paths) {
  robot_position_.clear();
  robot_orientation_.clear();
  camera_position_.clear();
  camera_orientation_.clear();
  clouds_.clear();

  robot_position_.resize(cloud_paths.size());
  robot_orientation_.resize(cloud_paths.size());
  camera_position_.resize(cloud_paths.size());
  camera_orientation_.resize(cloud_paths.size());
  clouds_.resize(cloud_paths.size());

  std::cout << "Reading clouds... " << std::endl;
  #pragma omp parallel for
  for (size_t i = 0; i < cloud_paths.size(); i++) {
    FileStorage fs;
    fs.open(cloud_paths[i], FileStorage::READ);
    if (fs.isOpened()) {
      // cout << "File found! " << cloud_names[i] << endl;
      std::vector<cv::Point3d> points;
      cv::Point3d robot_position, robot_orientation, camera_position, camera_orientation;
      fs["points"] >> points;
      fs["robot_position"] >> robot_position;
      fs["robot_orientation"] >> robot_orientation;
      fs["camera_position"] >> camera_position;
      fs["camera_orientation"] >> camera_orientation;

      robot_position_[i] = robot_position;
      robot_orientation_[i] = robot_orientation;
      camera_position_[i] = camera_position;
      camera_orientation_[i] = camera_orientation;
      clouds_[i] = points;
    } else {
      cout << "Unable to open camera pose file at " << cloud_paths[i]
           << ".  Please verify the path." << endl;
    }
  }
  std::cout << "Clouds loaded! " << std::endl;
  return true;
}

bool terrain_slam::TerrainSlam::cvToCGAL(const Mat& in, vector<Aff3>& out) {
  out.clear();

  // in Matrix contains (x, y, z, roll, pitch, yaw) in meters and degrees
  for (size_t i = 0; i < in.rows; i++) {
    double x = in.at<double>(i, 0);
    double y = in.at<double>(i, 1);
    double z = in.at<double>(i, 2);
    double roll  = in.at<double>(i, 3)*M_PI/180.0;
    double pitch = in.at<double>(i, 4)*M_PI/180.0;
    double yaw   = in.at<double>(i, 5)*M_PI/180.0;

    double ci = cos(roll);
    double cj = cos(pitch);
    double ch = cos(yaw);
    double si = sin(roll);
    double sj = sin(pitch);
    double sh = sin(yaw);
    double cc = ci * ch;
    double cs = ci * sh;
    double sc = si * ch;
    double ss = si * sh;

    double m00 = cj * ch;
    double m01 = sj * sc - cs;
    double m02 = sj * cc + ss;
    double m10 = cj * sh;
    double m11 = sj * ss + cc;
    double m12 = sj * cs - sc;
    double m20 = -sj;
    double m21 = cj * si;
    double m22 = cj * ci;
    double m03 = x;
    double m13 = y;
    double m23 = z;
    Aff3 tf(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
    out.push_back(tf);
  }
  return true;
}

int main(int argc, char** argv) {
  terrain_slam::TerrainSlam tslam(argc, argv);
  return 0;
}