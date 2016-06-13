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
#include <terrain_slam/outlier_remover.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>

namespace po = boost::program_options;
using namespace std;
using namespace cv;

terrain_slam::TerrainSlam::TerrainSlam(int argc, char **argv) {
  patch_size_ = 5.0;
  mean_k_     = 10;
  std_mult_   = 6.0;
  debug_      = false;
  filter_     = false;

  if (parseCommandLine(argc, argv)) {
    process();
  }
}

bool terrain_slam::TerrainSlam::parseCommandLine(int argc, char **argv) {
  try {
    po::options_description description("Terrain Slam");
    description.add_options()("help,h", "Display this help message")(
        "clouds,c", po::value<string>(),
        "Input folder where the point clouds are")(
        "size,s", po::value<double>()->default_value(patch_size_),
        "Patch size in meters")(
        "mean_k,k", po::value<double>()->default_value(mean_k_),
        "Number for kNN (--filter)")(
        "std_mult,m", po::value<double>()->default_value(std_mult_),
        "Multiplier to stdev (--filter)")(
        "debug,d", "Show debug prints")(
        "filter,f", "Enable laser line outlier removal (Filtering)");

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(description).run(),
              vm);
    po::notify(vm);

    if (vm.count("help")) {
      cout << description;
    } else if (vm.count("clouds")) {
      clouds_dir_ = vm["clouds"].as<string>();
      if (vm.count("size"))     patch_size_ = vm["size"].as<double>();
      if (vm.count("mean_k"))   mean_k_     = vm["mean_k"].as<double>();
      if (vm.count("std_mult")) std_mult_   = vm["std_mult"].as<double>();
      if (vm.count("debug"))    debug_      = true;
      if (vm.count("filter"))   filter_     = true;
      cout.setf(ios::boolalpha);
      cout << "Running Terrain Slam with the following parameters:"
           << "\n\t* Clouds directory  : " << clouds_dir_ << endl;
      return true;
    } else {
      cout << "REQUIRED arguments were not provided." << endl;
      cout << description;
    }
  } catch (po::error e) {
    cerr << "Error: " << e.what() << ". Aborting" << endl;
  }
  return false;
}

void terrain_slam::TerrainSlam::process() {
  // Retrieve list of files
  vector<string> cloud_names;
  vector<string> cloud_paths;
  bool files_found = getCloudPaths(clouds_dir_, "yml", cloud_names, cloud_paths);

  if (files_found) {
    vector<LaserLine> lines;
    // Read files and load them in memory
    readFiles(cloud_names, cloud_paths, lines);

    // Create required length patches
    vector<CloudPatchPtr> patches;
    createPatches(lines, patches);

    // Search for overlapping patches
    vector<pair<int, int> > candidates;
    lookForCandidates(patches, candidates);

    for (size_t i = 0; i < candidates.size(); i++) {
      int id1 = candidates[i].first;
      int id2 = candidates[i].second;
      std::cout << "Find transform between " << id1 << " and " << id2 << std::endl;
      findTransform(patches, id1, id2);
    }
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
    string full_cloud_path(v[i].string());
    string name(v[i].stem().string());
    string extension(v[i].extension().string());
    if (extension == "." + format) {
      cloud_names.push_back(name);
      cloud_paths.push_back(full_cloud_path);
    }
  }

  if (cloud_names.size() == 0) {
    cout << "No clouds found. Aborting... " << endl;
    return false;
  } else {
    cout << "Found " << cloud_names.size() << " clouds!" << endl;
    return true;
  }
}

// int terrain_slam::TerrainSlam::preprocessPoints(vector<cv::Point3d>& points, int idx = 0) {
//   // we known that points are ordered from left to right in camera image
//   int k = 0;
//   for (size_t i = idx; i < points.size() - 1; i++) {
//     cv::Point3d p = points[i];
//     cv::Point3d q = points[i + 1];
//     double delta_x = p.x - q.x;  // meters
//     double delta_y = p.y - q.y;  // meters
//     double delta_z = abs(p.z - q.z);  // meters
//     double tan_slope = delta_z / sqrt(delta_x*delta_x + delta_y*delta_y);
//     double max_slope_tan = tan(80.0*M_PI/180.);
//     if (delta_z > 0.5 || tan_slope > max_slope_tan) {
//       points.erase(points.begin() + i);
//       k++;
//       k += preprocessPoints(points, i - 1);
//       break;
//     }
//   }
//   return k;
// }

void terrain_slam::TerrainSlam::readFiles(const vector<string> &cloud_names,
                                          const vector<string> &cloud_paths,
                                          vector<LaserLine>& lines) {
  cout << "Reading clouds... " << endl;
  for (size_t i = 0; i < cloud_paths.size(); i++) {
    if (debug_) cout << cloud_names[i] << " ";
    FileStorage fs;
    fs.open(cloud_paths[i], FileStorage::READ);

    if (fs.isOpened()) {
      vector<cv::Point3d> points;
      cv::Point3d robot_position, robot_orientation, camera_position,
          camera_orientation;
      fs["points"] >> points;
      fs["robot_position"] >> robot_position;
      fs["robot_orientation"] >> robot_orientation;
      fs["camera_position"] >> camera_position;
      fs["camera_orientation"] >> camera_orientation;

      // check points
      if (debug_) cout << points.size() << " ";
      // add only if there are points
      if (points.size() > 0) {
        // int k = 0;
        // k = preprocessPoints(points);
        // if (debug_) cout << k << " ";

        Eigen::Vector3d xyz = cv2eigen(robot_position);
        Eigen::Vector3d rpy = cv2eigen(robot_orientation) * M_PI / 180.0;
        LaserLine line(points.size(), xyz, rpy);
        for (size_t j = 0; j < points.size(); j++) {
          line.add(cv2eigen(points[j]), j);
        }

        // Filter line, remove outliers
        if (points.size() > 10 && filter_) {
          OutlierRemover sor;
          sor.setInput(line.points);
          sor.setMeanK(10);
          sor.setStdThresh(3.0);
          Eigen::MatrixXd output;
          sor.filter(output);
          line.points.resize(output.rows(), output.cols());
          line.points = output;
        }

        if (debug_) cout << points.size() - line.points.cols() << " ";

        lines.push_back(line);

        if (debug_) cout << line.points.cols() << endl;
      } else {
        cout << "[WARN]: Empty pointcloud in " << cloud_names[i] << endl;;
      }
    } else {
      cout << "Unable to open camera pose file at " << cloud_paths[i]
           << ".  Please verify the path." << endl;
    }
  }
  cout << "Clouds loaded! " << endl;
}

void
terrain_slam::TerrainSlam::createPatches(
    const vector<LaserLine>& lines, vector<CloudPatchPtr>& patches) {
  double robot_distance  = 0;
  double centroid_distance = 0;
  int pivot_idx = 0;

  int patch_idx = 0;

  cout << "Processing... " << endl;
  for (size_t i = 0; i < lines.size(); i++) {
    // Accumulate clouds while distance is less than patch_size_
    Eigen::Vector3d pos1 = lines[pivot_idx].getPosition();
    Eigen::Vector3d pos2 = lines[i].getPosition();
    Eigen::Vector3d robot_diff = pos2 - pos1;
    robot_distance = robot_diff.norm();

    bool jump = false;
    if (i < lines.size() - 1) {
      Eigen::Vector3d pos_b = lines[i + 1].getPosition();
      Eigen::Vector3d robot_diff_prev = pos_b - pos2;
      double d = robot_diff_prev.norm();
      if (d > 0.5) jump = true;
    }

    if (i < lines.size() - 1) {
      Eigen::Vector3d c1 = lines[i].getCentroid();
      Eigen::Vector3d c2 = lines[i + 1].getCentroid();
      Eigen::Vector3d centroid_diff = pos2 - pos1;
      centroid_distance = centroid_diff.norm();
    }

    if (robot_distance > patch_size_ || centroid_distance > 10.0 || i == lines.size() - 1 || jump) {
      CloudPatchPtr patch(new CloudPatch(lines, pivot_idx, i));
      // savePatch(patch, patch_idx);
      patches.push_back(patch);
      pivot_idx = i + 1;
      patch_idx++;
    }
  }
}

// Save the patch with the position information
void terrain_slam::TerrainSlam::savePatch(
    const CloudPatchPtr& patch, int idx) {
  string path("../output");
  boost::filesystem::path dir(path);
  boost::filesystem::create_directory(dir);

  ostringstream ss;
  ss << setw(4) << setfill('0') << idx;
  string cloud_filename = path + "/cloud" + ss.str() + ".ply";
  cout << "Saving ply file " << cloud_filename << endl;
  PlySaver saver;
  saver.saveCloud(cloud_filename, patch->getPoints());
}


void
terrain_slam::TerrainSlam::lookForCandidates(
    const vector<CloudPatchPtr>& patches,
    vector<pair<int, int> >& candidates) {
  cout << "Looking for candidates... " << endl;
  for (size_t i = 0; i < patches.size(); i++) {
    for (size_t j = i + 1; j < patches.size(); j++) {
      Eigen::Vector3d ci = patches.at(i)->getCentroid();
      Eigen::Vector3d cj = patches.at(j)->getCentroid();
      Eigen::Vector3d diff = ci - cj;
      double distance = diff.norm();
      if (distance < 5.0) {
        cout << "Between " << i << " and " << j
             << " distance: " << distance << endl;
        pair<int, int> p = make_pair(i, j);
        candidates.push_back(p);
      }
    }
  }
}

terrain_slam::Transform
terrain_slam::TerrainSlam::findTransform(const vector<CloudPatchPtr> &c,
                                         int id1,
                                         int id2) {
  c.at(id1)->grid();
  c.at(id2)->grid();

  // Save clouds
  // string path("../grids");
  // boost::filesystem::path dir(path);
  // boost::filesystem::create_directory(dir);

  // ostringstream ss;
  // ss << setw(4) << setfill('0') << id1;
  // string cloud_filename = path + "/cloud" + ss.str() + ".ply";
  // cout << "Saving ply file " << cloud_filename
  //      << " with " << c1_gridded.cols() << " points" << endl;
  // PlySaver saver;
  // saver.saveCloud(cloud_filename, c1_gridded);

  // Adjust
  std::cout << "Adjusting " << id1 << " to " << id2 << "..." << std::endl;
  adj_ = new Adjuster();
  boost::shared_ptr<CloudPatch> c1(c.at(id1));
  boost::shared_ptr<CloudPatch> c2(c.at(id2));
  adj_->adjust(c1, c2);
  std::cout << "Done!" << std::endl;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  terrain_slam::TerrainSlam tslam(argc, argv);
  return 0;
}