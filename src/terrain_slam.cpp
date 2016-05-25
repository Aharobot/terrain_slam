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
    double size = 5.0;

    description.add_options()
    ("help,h", "Display this help message")
    ("clouds,c", po::value<string>(), "Input folder where the point clouds are")
    ("size,s", po::value<double>()->default_value(size), "Patch size in meters");

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(description).run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
      cout << description;
    } else if (vm.count("clouds")) {
      clouds_dir = vm["clouds"].as<string>();
      if (vm.count("size")) size = vm["size"].as<double>();
      std::cout.setf(std::ios::boolalpha);
      cout << "Running Terrain Slam with the following parameters:"
         << "\n\t* Clouds directory  : " << clouds_dir << endl;
      process(clouds_dir, size);
    } else {
      cout << "REQUIRED arguments were not provided." << endl;
      cout << description;
    }
  } catch(po::error e) {
    cerr << "Error: " << e.what() << ". Aborting" << endl;
  }
}

void terrain_slam::TerrainSlam::process(const std::string& clouds_dir, double size) {
  // Set patch size
  patch_size_ = size;

  // Retrieve list of files
  vector<string> cloud_names;
  vector<string> cloud_paths;
  getCloudPaths(clouds_dir, "yml", cloud_names, cloud_paths);

  // Read files and load them in memory
  readFiles(cloud_names, cloud_paths);

  // Do the job!
  processPatches();
}

void terrain_slam::TerrainSlam::processPatches() {
  for (size_t i = 0; i < clouds_.size(); i++) {
    // Accumulate clouds while distance is less than patch_size_

  }
}

void terrain_slam::TerrainSlam::getCloudPaths(const string& path,
                                              const string& format,
                                              vector<string>& cloud_names,
                                              vector<string>& cloud_paths) {
  boost::filesystem::path p(path);
  typedef vector<boost::filesystem::path> vec; // store paths,
  vec v;                                       // so we can sort them later
  try {
    if (exists(p)) {
      if (is_directory(p)) {
        copy(boost::filesystem::directory_iterator(p),
             boost::filesystem::directory_iterator(),
             back_inserter(v));
        sort(v.begin(), v.end());  // sort, since directory iteration
                                   // is not ordered on some file systems
      } else {
        cout << p << " exists, but is neither a regular file nor a directory\n";
      }
    } else {
      cout << p << " does not exist\n";
    }
  } catch (const boost::filesystem::filesystem_error& ex) {
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
}

Eigen::Quaternion<double> terrain_slam::TerrainSlam::rpyToRotationMatrix(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitX());
  Eigen::Quaternion<double> q = roll_angle * yaw_angle * pitch_angle;
  return q;
}

Eigen::Matrix4d terrain_slam::TerrainSlam::buildTransform(const Eigen::Quaternion<double> q, const Eigen::Vector3d t) {
  Eigen::Matrix3d R = q.matrix();
  Eigen::Matrix4d T;
  // Set to Identity to make bottom row of Matrix 0,0,0,1
  T.setIdentity();
  T.block<3,3>(0,0) = R;
  T(0,3) = t(0);
  T(1,3) = t(1);
  T(2,3) = t(2);
  return T;
}

void terrain_slam::TerrainSlam::cv2eigen(const cv::Point3d& p, Eigen::Vector3d& v) {
  v(0) = p.x;
  v(1) = p.y;
  v(2) = p.z;
}

void terrain_slam::TerrainSlam::readFiles(const vector<string>& cloud_names,
                                          const vector<string>& cloud_paths) {
  clouds_.clear();
  robot_tf_.clear();
  camera_tf_.clear();

  clouds_.resize(cloud_paths.size());
  robot_tf_.resize(cloud_paths.size());
  camera_tf_.resize(cloud_paths.size());

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
      clouds_[i].resize(points.size());
      for (size_t j = 0; j < points.size(); j++) {
        cv2eigen(points[j], clouds_[i][j]);
      }

      Eigen::Quaternion<double> q;
      Eigen::Vector3d t;

      // Compute transformation matrix for robot
      q = rpyToRotationMatrix(robot_orientation.x, robot_orientation.y, robot_orientation.z);
      t = Eigen::Vector3d(robot_position.x, robot_position.y, robot_position.z);
      robot_tf_[i]  = buildTransform(q, t);

      // Compute transformpation matrix for camera
      q = rpyToRotationMatrix(camera_orientation.x, camera_orientation.y, camera_orientation.z);
      t = Eigen::Vector3d(camera_position.x, camera_position.y, camera_position.z);
      camera_tf_[i] = buildTransform(q, t);
    } else {
      cout << "Unable to open camera pose file at " << cloud_paths[i]
           << ".  Please verify the path." << endl;
    }
  }
  std::cout << "Clouds loaded! " << std::endl;
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