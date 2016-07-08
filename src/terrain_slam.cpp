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
#include <terrain_slam/pcl_tools.h>
#include <terrain_slam/csv.h>

#include <terrain_slam/features/features.h>
#include <terrain_slam/features/keypoints.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>

namespace po = boost::program_options;
using namespace std;
using namespace cv;

////////////////////////////////////////////////////////////////////////////////

terrain_slam::TerrainSlam::TerrainSlam(int argc, char **argv) : adj_(new Adjuster()), graph_(new Graph()) {
// terrain_slam::TerrainSlam::TerrainSlam(int argc, char **argv) : graph_(new Graph()) {
  patch_size_ = 5.0;
  mean_k_     = 10;
  std_mult_   = 6.0;
  debug_      = false;
  filter_     = false;

  if (parseCommandLine(argc, argv)) {
    process();
  }
}

////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////

void terrain_slam::TerrainSlam::process() {
  // Retrieve list of files
  vector<string> cloud_names;
  vector<string> cloud_paths;
  bool files_found = getCloudPaths(clouds_dir_, "csv", cloud_names, cloud_paths);

  if (files_found) {
    vector<LaserLine> lines;
    // Read files and load them in memory
    readFiles(cloud_names, cloud_paths, lines);

    // Create required length patches
    createPatches(lines, patches_);

    // Search for overlapping patches
    vector<pair<int, int> > candidates;
    lookForCandidates(patches_, candidates);

    for (size_t i = 0; i < candidates.size(); i++) {
      int id1 = candidates[i].first;
      int id2 = candidates[i].second;
      // std::cout << "[INFO]: Find transform between " << id1 << " and " << id2 << std::endl;
      // findTransform(patches_, id1, id2);
    }

    findTransform(patches_, 14, 21);
    findTransform(patches_, 0, 27);

    // Save results
    graph_->saveGraph();
  }
}

////////////////////////////////////////////////////////////////////////////////

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
    cout << "[ERROR]: No clouds found. Aborting... " << endl;
    return false;
  } else {
    cout << "[INFO]: Found " << cloud_names.size() << " clouds!" << endl;
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

////////////////////////////////////////////////////////////////////////////////

void terrain_slam::TerrainSlam::readFiles(const vector<string> &cloud_names,
                                          const vector<string> &cloud_paths,
                                          vector<LaserLine>& lines) {
  for (size_t i = 0; i < cloud_paths.size(); i++) {
    if (debug_) cout << cloud_names[i] << " ";

    ifstream fs(cloud_paths[i]);
    if (!fs.is_open()) {
      std::cout << "Can't open file " << cloud_paths[i] << std::endl;
      return;
    }
    Eigen::Vector3d robot_xyz;
    Eigen::Vector3d robot_rpy;
    Eigen::Vector3d p;
    int lineno = 0;
    int num_points = 0;
    vector<Eigen::Vector3d> points;
    for(CSVIterator loop(fs); loop != CSVIterator(); ++loop) {
      // for (size_t ii = 0; ii < (*loop).size(); ii++)
      //   std::cout << "(*loop)[" << ii << "]: " << (*loop)[ii] << std::endl;
      if (lineno == 0) {
        num_points = boost::lexical_cast<int>((*loop)[0]);
      } else if (lineno == 1) {
        robot_xyz(0) = boost::lexical_cast<double>((*loop)[0]);
        robot_xyz(1) = boost::lexical_cast<double>((*loop)[1]);
        robot_xyz(2) = boost::lexical_cast<double>((*loop)[2]);
        robot_rpy(0) = boost::lexical_cast<double>((*loop)[3]);
        robot_rpy(1) = boost::lexical_cast<double>((*loop)[4]);
        robot_rpy(2) = boost::lexical_cast<double>((*loop)[5]);
        robot_rpy *= M_PI/180.0;
      } else {
        Eigen::Vector3d p;
        p(0) = boost::lexical_cast<double>((*loop)[0]);
        p(1) = boost::lexical_cast<double>((*loop)[1]);
        p(2) = boost::lexical_cast<double>((*loop)[2]);
        points.push_back(p);
      }
      lineno++;
    }

    // check points
    if (debug_) cout << points.size() << " ";
    // add only if there are points
    if (points.size() > 0) {
      LaserLine line(points.size(), robot_xyz, robot_rpy);
      for (size_t j = 0; j < points.size(); j++) {
        line.add(points[j], j);
      }

      line.setId(i);
      line.setFilename(cloud_names[i]);

      // Filter line, remove outliers
      if (points.size() > 10 && filter_) {
        OutlierRemover sor;
        sor.setInput(line.points);
        sor.setMeanK(mean_k_);
        sor.setStdThresh(std_mult_);
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
  }
  cout << "[INFO]: Clouds loaded! " << endl;
}

////////////////////////////////////////////////////////////////////////////////

void
terrain_slam::TerrainSlam::createPatches(
    const vector<LaserLine>& lines, vector<CloudPatchPtr>& patches) {
  double robot_distance  = 0;
  double centroid_distance = 0;
  int pivot_idx = 0;

  int patch_idx = 0;

  cout << "[INFO]: Processing... " << endl;
  bool joined = true;
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
      if (d > 0.5) {
        jump = true;
        // std::cout << "Jump of " << robot_diff_prev.transpose() << " from line " << i << " to " << i + 1 << std::endl;
        // std::cout << "line i + 1: " << lines[i + 1].getPosition().transpose() << " id: " << lines[i + 1].getFilename() << std::endl;
        // std::cout << "line i: " << lines[i].getPosition().transpose() << " id: " << lines[i].getFilename() << std::endl;
      }
    }

    if (i < lines.size() - 1) {
      Eigen::Vector3d c1 = lines[i].getCentroid();
      Eigen::Vector3d c2 = lines[i + 1].getCentroid();
      Eigen::Vector3d centroid_diff = pos2 - pos1;
      centroid_distance = centroid_diff.norm();
    }

    if (robot_distance > patch_size_ || centroid_distance > 10.0 || i == lines.size() - 1 || jump) {
      // create new patch
      CloudPatchPtr patch(new CloudPatch(lines, pivot_idx, i));
      patch->setId(patch_idx);
      // patch->save(patch_idx);

      // add to vector
      patches.push_back(patch);

      // add to graph
      // if (robot_distance > patch_size_) std::cout << "Robot distance ";
      // if (centroid_distance > 10.0) std::cout << "Centroid ";
      // if (i == lines.size() - 1) std::cout << "Size ";
      // if (jump) std::cout << "Jump ";
      graph_->addVertex(patch->getIsometry());

      if (joined && patch_idx > 0) {
        Transform edge(patches[patch_idx - 1]->T.inverse()*patches[patch_idx]->T);
        graph_->addEdge(patch_idx - 1, patch_idx, edge.getIsometry(), DEFAULT_WEIGHT);
      } else if (!joined) {
        joined = true;
      }

      pivot_idx = i + 1;
      patch_idx++;

      if (jump) joined = false;
    }
  }

//   // Grid patches
//   std::cout << "[INFO]: Gridding... (parallel)" << std::endl;
// #pragma omp parallel for
//   for (size_t i = 0; i < patches.size(); i++) {
//     patches.at(i)->grid();
//   }
}

////////////////////////////////////////////////////////////////////////////////

void
terrain_slam::TerrainSlam::lookForCandidates(
    const vector<CloudPatchPtr>& patches,
    vector<pair<int, int> >& candidates) {
  cout << "[INFO]: Looking for candidates... " << endl;

  vector<vector<int> > neighbors(patches.size());
  for (size_t i = 0; i < patches.size(); i++) {
    vector<double> distance;
    graph_->findClosestVertices(i, 3, neighbors[i], distance);
    for (size_t j = 0; j < neighbors[i].size(); j++) {
      if (i > neighbors[i][j]) continue;  // if 1 -> 6, 6 -> 1 too.
      if (distance[i] > 2*patch_size_) continue;
      pair<int, int> p = make_pair(i, neighbors[i][j]);
      // Check if the pair already exist
      if (std::find(candidates.begin(),
                     candidates.end(),
                     p) == candidates.end()) {
        candidates.push_back(p);
        cout << "[INFO]: ByDistance " << i << " - " << neighbors[i][j] << endl;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

terrain_slam::Transform
terrain_slam::TerrainSlam::findTransform(const vector<CloudPatchPtr> &c,
                                         int id1,
                                         int id2) {
  // Adjust
  std::cout << "Adjusting " << id1 << " to " << id2 << "..." << std::endl;
  // adj_.reset(new BruteForceAdjuster(-7.0, 7.0,  // x
  //                                   -7.0, 7.0,  // y
  //                                     0, 0,  // z
  //                                     0, 0,  // yaw
  //                                     0.1,       // x-y resolution
  //                                     0.5,       // z resolution
  //                                     0.5));    // yaw resolution
  boost::shared_ptr<CloudPatch> c1(c.at(id1));
  boost::shared_ptr<CloudPatch> c2(c.at(id2));
  // c1->save(id1, std::string("../adjusted"));
  // c2->save(id2, std::string("../adjusted"));


  // Get data
  Eigen::Matrix4d relative = c1->T.inverse()*c2->T;
  bool local_frame = true;
  bool use_grid = false;
  Eigen::Matrix4Xd p1 = c1->getPoints(local_frame, use_grid);
  Eigen::Matrix4Xd p2 = c2->getPoints(local_frame, use_grid);

  // Convert cloud to PCL
  std::cout << "Converting cloud to PCL..." << std::endl;
  PC::Ptr source_cloud = pcl_tools::toPCL(p1);
  PC::Ptr target_cloud = pcl_tools::toPCL(p2);
  pcl_tools::saveCloud(source_cloud, "orig", id1);
  pcl_tools::saveCloud(target_cloud, "orig", id2);

  PC::Ptr source_grid(new PC);
  PC::Ptr target_grid(new PC);
  pcl_tools::voxelGrid(source_cloud, 0.25, source_grid);
  pcl_tools::voxelGrid(target_cloud, 0.25, target_grid);

  // Converting it back
  std::cout << "Converting back to Eigen..." << std::endl;
  Eigen::Matrix4Xd p3 = pcl_tools::fromPCL(source_grid);
  Eigen::Matrix4Xd p4 = pcl_tools::fromPCL(target_grid);
  c1->points = p3;
  c1->copy2Grid();
  c2->points = p4;
  c2->copy2Grid();

  pcl_tools::saveCloud(source_grid, "grid", id1);
  pcl_tools::saveCloud(target_grid, "grid", id2);

  std::cout << "Running adjuster... " << std::endl;
  Transform t = adj_->adjust(c1, c2);

  PC::Ptr target_tf(new PC);
  pcl::transformPointCloud(*target_grid, *target_tf, t.tf());
  pcl_tools::saveCloud(target_tf, "tf", id2);


  c1->save(id1, std::string("../adjusted"), std::string("post"), true, false);
  c2->save(id2, std::string("../adjusted"), std::string("post"), true, false);

  // // Smooth clouds
  // std::cout << "Smoothing clouds..." << std::endl;
  // PC::Ptr source_smoothed(new PC);
  // PC::Ptr target_smoothed(new PC);
  // pcl_tools::smooth(source_cloud, source_smoothed);
  // pcl_tools::smooth(target_cloud, target_smoothed);
  // pcl_tools::saveCloud(source_smoothed, "smooth", id1);
  // pcl_tools::saveCloud(target_smoothed, "smooth", id2);

  // // Remove outliers
  // std::cout << "Removing outliers..." << std::endl;
  // PC::Ptr source_filt(new PC);
  // PC::Ptr target_filt(new PC);
  // double min_z = 0.5;   // m
  // double max_z = 10.0;  // m
  // double neighbors_radius = 0.4;  // m
  // int min_neighbors = 40;
  // pcl_tools::removeOutliers(source_smoothed, min_z, max_z, neighbors_radius, min_neighbors, source_filt);
  // pcl_tools::removeOutliers(target_smoothed, min_z, max_z, neighbors_radius, min_neighbors, target_filt);
  // pcl_tools::saveCloud(source_filt, "outl", id1);
  // pcl_tools::saveCloud(target_filt, "outl", id2);

  // // Sampling the cloud
  // std::cout << "Sampling the cloud..." << std::endl;
  // PC::Ptr source_grid(new PC);
  // PC::Ptr target_grid(new PC);
  // int num_points = 2000;
  // double radius_search = 0.25;
  // pcl_tools::randomlySample(source_filt, num_points, radius_search, source_grid);
  // pcl_tools::randomlySample(target_filt, num_points, radius_search, target_grid);
  // pcl_tools::saveCloud(source_grid, "grid", id1);
  // pcl_tools::saveCloud(target_grid, "grid", id2);

  // // Exaggerate Z component
  // std::cout << "Exaggerating Z component..." << std::endl;
  // PC::Ptr source_ex(new PC);
  // PC::Ptr target_ex(new PC);
  // double multiplier = 4.0;
  // // pcl_tools::exaggerateZ(source_grid, multiplier, source_ex);
  // // pcl_tools::exaggerateZ(target_grid, multiplier, target_ex);
  // // pcl_tools::saveCloud(source_ex, "exag", id1);
  // // pcl_tools::saveCloud(target_ex, "exag", id2);

  // // Converting it back
  // std::cout << "Converting back to Eigen..." << std::endl;
  // Eigen::Matrix4Xd p3 = pcl_tools::fromPCL(source_grid);
  // Eigen::Matrix4Xd p4 = pcl_tools::fromPCL(target_grid);
  // c1->points = p3;
  // c1->copy2Grid();
  // c2->points = p4;
  // c2->copy2Grid();

  // c1->save(id1, std::string("../adjusted"));
  // c2->save(id2, std::string("../adjusted"));

  // std::cout << "Running adjuster... " << std::endl;
  // Transform t = adj_->adjust(c1, c2);

  // return t;

  // // Compute normals
  // std::cout << "Computing normals..." << std::endl;
  // PCN::Ptr source_with_normals(new PCN);
  // PCN::Ptr target_with_normals(new PCN);
  // pcl_tools::addNormals(source_ex, 0.4, source_with_normals);
  // pcl_tools::addNormals(target_ex, 0.4, target_with_normals);

  // // ICP
  // std::cout << "Registering with ICP..." << std::endl;
  // bool success = false;
  // Eigen::Matrix4f tf;
  // double score = -1.0;
  // success = pcl_tools::icp(source_ex, target_ex, tf, score);
  // std::cout << "ICP fitness score: " << score << std::endl;

  // if (success) {
  //   PC::Ptr source_tf(new PC);
  //   pcl::transformPointCloud(*source_ex, *source_tf, tf);
  //   pcl_tools::saveCloud(source_tf, "icp", id1);
  //   pcl_tools::saveCloud(target_ex, "icp", id2);
  // }


  // // ICP with normals
  // success = false;
  // score = -1.0;
  // std::cout << "Registering with ICP with normals..." << std::endl;
  // success = pcl_tools::icpn(source_with_normals, target_with_normals, tf, score);
  // std::cout << "ICP fitness score: " << score << std::endl;

  // if (success) {
  //   PC::Ptr source_tf(new PC);
  //   pcl::transformPointCloud(*source_ex, *source_tf, tf);
  //   pcl_tools::saveCloud(source_tf, "icpn", id1);
  //   pcl_tools::saveCloud(target_ex, "icpn", id2);
  // }


  // std::cout << "Done!" << std::endl;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  terrain_slam::TerrainSlam tslam(argc, argv);
  return 0;
}