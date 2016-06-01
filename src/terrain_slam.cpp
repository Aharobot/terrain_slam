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
#include <terrain_slam/statistical_outlier_remover.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <nabo/nabo.h>

namespace po = boost::program_options;
using namespace std;
using namespace cv;

terrain_slam::TerrainSlam::TerrainSlam(int argc, char **argv) {
  string clouds_dir;
  double size = 5.0;

  bool success = parseCommandLine(argc, argv, clouds_dir, size);

  if (success) {
    process(clouds_dir, size);
  }
}

bool terrain_slam::TerrainSlam::parseCommandLine(
    int argc, char **argv, string& clouds_dir, double& size) {
  try {
    po::options_description description("Terrain Slam");
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
      cout.setf(ios::boolalpha);
      cout << "Running Terrain Slam with the following parameters:"
           << "\n\t* Clouds directory  : " << clouds_dir << endl;
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

void terrain_slam::TerrainSlam::process(const string &clouds_dir,
                                        double size) {
  // Set patch size
  patch_size_ = size;

  // Retrieve list of files
  vector<string> cloud_names;
  vector<string> cloud_paths;
  bool files_found = getCloudPaths(clouds_dir, "yml", cloud_names, cloud_paths);

  if (files_found) {
    vector<LaserLine> lines;
    // Read files and load them in memory
    readFiles(cloud_names, cloud_paths, lines);

    // Create required length patches
    vector<CloudPatch> patches;
    createPatches(lines, patches);

    // Search for overlapping patches
    vector<pair<int, int> > candidates;
    lookForCandidates(patches, candidates);
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

void terrain_slam::TerrainSlam::readFiles(const vector<string> &cloud_names,
                                          const vector<string> &cloud_paths,
                                          vector<LaserLine>& lines) {
  cout << "Reading clouds... " << endl;
  for (size_t i = 0; i < cloud_paths.size(); i++) {
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

      int num_points = points.size();
      Eigen::Vector3d xyz = cv2eigen(robot_position);
      Eigen::Vector3d rpy = cv2eigen(robot_orientation) * M_PI / 180.0;
      LaserLine line(num_points, xyz, rpy);
      for (size_t j = 0; j < points.size(); j++) {
        line.add(cv2eigen(points[j]), j);
      }

      // Filter line, remove outliers
      cout << "Filtering cloud " << cloud_names[i] << " with " << num_points << " points";
      if (num_points > 10) {
        StatisticalOutlierRemover sor;
        sor.setInput(line.points);
        sor.setMeanK(10);
        sor.setStdThresh(3.0);
        Eigen::MatrixXd output;
        sor.filter(output);
        line.points.resize(output.rows(), output.cols());
        line.points = output;
        if (num_points - line.points.cols() != 0) {
          cout << " - removed " << num_points - output.cols() << endl;
        } else {
          cout << endl;
        }
      } else {
        // if there are not enough points, skip the line
        cout << " - SKIPPING" << endl;
      }

      lines.push_back(line);
    } else {
      cout << "Unable to open camera pose file at " << cloud_paths[i]
           << ".  Please verify the path." << endl;
    }
  }
  cout << "Clouds loaded! " << endl;
}

void
terrain_slam::TerrainSlam::createPatches(
    const vector<LaserLine>& lines, vector<CloudPatch>& patches) {
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

    if (i < lines.size() - 1) {
      Eigen::Vector3d c1 = lines[i].getCentroid();
      Eigen::Vector3d c2 = lines[i + 1].getCentroid();
      Eigen::Vector3d centroid_diff = pos2 - pos1;
      centroid_distance = centroid_diff.norm();
    }

    if (robot_distance > patch_size_ || centroid_distance > 10.0 || i == lines.size() - 1) {
      CloudPatch patch(lines, pivot_idx, i);
      savePatch(patch, patch_idx);
      patches.push_back(patch);
      pivot_idx = i + 1;
      patch_idx++;
    }
  }
}

// Save the patch with the position information
void terrain_slam::TerrainSlam::savePatch(
    const CloudPatch& patch, int idx) {
  string path("../output");
  boost::filesystem::path dir(path);
  boost::filesystem::create_directory(dir);

  ostringstream ss;
  ss << setw(4) << setfill('0') << idx;
  string cloud_filename = path + "/cloud" + ss.str() + ".ply";
  cout << "Saving ply file " << cloud_filename << endl;
  PlySaver saver;
  saver.saveCloud(cloud_filename, patch.getPoints());
}


void
terrain_slam::TerrainSlam::lookForCandidates(
    const vector<CloudPatch>& patches,
    vector<pair<int, int> >& candidates) {
  cout << "Looking for candidates... " << endl;
  for (size_t i = 0; i < patches.size(); i++) {
    for (size_t j = i + 1; j < patches.size(); j++) {
      Eigen::Vector3d ci = patches[i].getCentroid();
      Eigen::Vector3d cj = patches[j].getCentroid();
      Eigen::Vector3d diff = ci - cj;
      double distance = diff.norm();
      if (distance < 3.0) {
        cout << "Between " << i << " and " << j
             << " distance: " << distance << endl;
        // Search nearest neighbours
        Eigen::MatrixXd M = patches[i].points;
        Eigen::MatrixXd q = patches[j].points;
        Nabo::NNSearchD* nns = Nabo::NNSearchD::createKDTreeLinearHeap(M);
        // Indices of the kNN points
        Eigen::MatrixXi indices;
        // L2 distance between the query and the kNN points
        Eigen::MatrixXd dists2;
        // the "k" of kNN, e.g. the number of nearest neighbours
        int K = 5;
        indices.resize(K, q.cols());
        dists2.resize(K, q.cols());
        nns->knn(q, indices, dists2, K, 0, Nabo::NNSearchF::SORT_RESULTS);
      }
    }
  }
}

int main(int argc, char **argv) {
  terrain_slam::TerrainSlam tslam(argc, argv);
  return 0;
}