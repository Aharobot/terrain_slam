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
#include <terrain_slam/pcl_tools.h>
#include <terrain_slam/eigen_tools.h>
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

////////////////////////////////////////////////////////////////////////////////

terrain_slam::TerrainSlam::TerrainSlam(int argc, char **argv) : adj_(new Adjuster()), graph_(new Graph()) {
// terrain_slam::TerrainSlam::TerrainSlam(int argc, char **argv) : graph_(new Graph()) {
  patch_size_ = 8.0; // 5.0;
  mean_k_     = 10;
  std_mult_   = 6.0;
  debug_      = false;
  filter_     = false;
  min_pts_    = 500;

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
    vector<CloudPatchPtr> lines;
    // Read files and load them in memory
    readFiles(cloud_names, cloud_paths, lines);

    // Create required length patches
    createPatches(lines, patches_);

    // Search for overlapping patches
    vector<pair<int, int> > candidates;
    lookForCandidates(patches_, candidates);

    // findTransform(patches_, 14, 21);
    // findTransform(patches_, 0, 27);

    // Save original clouds
    for (size_t i = 0; i < patches_.size(); i++) {
      Eigen::Isometry3d pose = graph_->getVertexPose(i);
      CloudPatchPtr c(patches_.at(i));
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::transformPointCloud(*c->cloud, *cloud_tf, pose.matrix());
      pcl_tools::saveCloud(cloud_tf, "global", c->getId());
      pcl_tools::saveCloud(c->cloud, "local", c->getId());
    }

    for (size_t i = 0; i < candidates.size(); i++) {
      int id1 = candidates[i].first;
      int id2 = candidates[i].second;
      Eigen::Matrix4d edge;
      bool res = findTransform(patches_, id1, id2, edge);
      if (res) {
        Eigen::Isometry3d edge_iso = eigen_tools::toIsometry(edge);
        graph_->addEdge(id1, id2, edge_iso, LC_WEIGHT);
        graph_->run();
        graph_->saveGraph();
      }
    }
    graph_->run();
    graph_->saveGraph();

    // save optimized clouds
    for (size_t i = 0; i < patches_.size(); i++) {
      Eigen::Isometry3d pose = graph_->getVertexPose(i);
      CloudPatchPtr c(patches_.at(i));
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::transformPointCloud(*c->cloud, *cloud_tf, pose.matrix());
      pcl_tools::saveCloud(cloud_tf, "final", c->getId());
    }
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

////////////////////////////////////////////////////////////////////////////////

void terrain_slam::TerrainSlam::readFiles(const vector<string> &cloud_names,
                                          const vector<string> &cloud_paths,
                                          vector<CloudPatchPtr>& lines) {
  for (size_t i = 0; i < cloud_paths.size(); i++) {
    if (debug_) cout << cloud_names[i] << " ";

    ifstream fs(cloud_paths[i]);
    if (!fs.is_open()) {
      std::cout << "Can't open file " << cloud_paths[i] << std::endl;
      return;
    }


    CloudPatchPtr line(new CloudPatch());

    int lineno = 0;
    int num_points = 0;
    vector<Eigen::Vector3d> points;
    for(CSVIterator loop(fs); loop != CSVIterator(); ++loop) {
      // for (size_t ii = 0; ii < (*loop).size(); ii++)
      //   std::cout << "(*loop)[" << ii << "]: " << (*loop)[ii] << std::endl;
      if (lineno == 0) {
        num_points = boost::lexical_cast<int>((*loop)[0]);
      } else if (lineno == 1) {
        Eigen::Vector3d robot_xyz;
        Eigen::Vector3d robot_rpy;
        robot_xyz(0) = boost::lexical_cast<double>((*loop)[0]);
        robot_xyz(1) = boost::lexical_cast<double>((*loop)[1]);
        robot_xyz(2) = boost::lexical_cast<double>((*loop)[2]);
        robot_rpy(0) = boost::lexical_cast<double>((*loop)[3]);
        robot_rpy(1) = boost::lexical_cast<double>((*loop)[4]);
        robot_rpy(2) = boost::lexical_cast<double>((*loop)[5]);
        robot_rpy *= M_PI/180.0;
        line->transform.T = eigen_tools::buildTransform(robot_xyz, robot_rpy);
      } else {
        Eigen::Vector3d p;
        p(0) = boost::lexical_cast<double>((*loop)[0]);
        p(1) = boost::lexical_cast<double>((*loop)[1]);
        p(2) = boost::lexical_cast<double>((*loop)[2]);
        line->add(p);
      }
      lineno++;
    }

    // check points
    if (debug_) cout << num_points << "==" << line->size() << "? ";
    // add only if there are points
    if (line->size() > 0) {
      line->setId(i);
      line->setName(cloud_names[i]);
      lines.push_back(line);
    } else {
      cout << "[WARN]: Empty pointcloud in " << cloud_names[i] << endl;;
    }
  }
  cout << "[INFO]: Clouds loaded! " << endl;
}

////////////////////////////////////////////////////////////////////////////////

void
terrain_slam::TerrainSlam::createPatches(
    const vector<CloudPatchPtr>& lines, vector<CloudPatchPtr>& patches) {
  double robot_distance  = 0;
  double centroid_distance = 0;
  int pivot_idx = 0;

  int patch_idx = 0;

  cout << "[INFO]: Processing... " << endl;
  bool joined = true;
  for (size_t i = 0; i < lines.size(); i++) {
    // Accumulate clouds while distance is less than patch_size_
    Eigen::Vector3d pos1 = lines[pivot_idx]->transform.origin();
    Eigen::Vector3d pos2 = lines[i]->transform.origin();
    Eigen::Vector3d robot_diff = pos2 - pos1;
    robot_distance = robot_diff.norm();
    bool jump = false;
    if (i < lines.size() - 1) {
      Eigen::Vector3d pos_b = lines[i + 1]->transform.origin();
      Eigen::Vector3d robot_diff_prev = pos_b - pos2;
      double d = robot_diff_prev.norm();
      if (d > 0.5) {
        jump = true;
      }
    }

    if (i < lines.size() - 1) {
      Eigen::Vector3d c1 = lines[i]->getCentroid();
      Eigen::Vector3d c2 = lines[i + 1]->getCentroid();
      Eigen::Vector3d centroid_diff = pos2 - pos1;
      centroid_distance = centroid_diff.norm();
    }

    if (robot_distance > patch_size_ || centroid_distance > 10.0 || i == lines.size() - 1 || jump) {
      // create new patch
      CloudPatchPtr patch(new CloudPatch);
      patch->setId(patch_idx);
      patch->transform = lines[pivot_idx]->transform;

      for (size_t p = pivot_idx; p <= i; p++) {
        patch->add(*(lines[p]));
      }

      //patch->save(patch_idx);  // TODO

      // add to vector
      patches.push_back(patch);

      // add to graph
      // if (robot_distance > patch_size_) std::cout << "Robot distance ";
      // if (centroid_distance > 10.0) std::cout << "Centroid ";
      // if (i == lines.size() - 1) std::cout << "Size ";
      // if (jump) std::cout << "Jump ";

      graph_->addVertex(eigen_tools::toIsometry(patch->transform()));

      if (joined && patch_idx > 0) {
        Eigen::Matrix4d prev = patches[patch_idx - 1]->transform();
        Eigen::Matrix4d curr = patches[patch_idx]->transform();
        Eigen::Matrix4d edge = prev.inverse() * curr;
        Eigen::Isometry3d iso = eigen_tools::toIsometry(edge);
        graph_->addEdge(patch_idx - 1, patch_idx, iso, DEFAULT_WEIGHT);
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

  // TODO sorting

  vector<vector<int> > neighbors(patches.size());
  for (size_t i = 0; i < patches.size(); i++) {
    vector<double> distance;
    graph_->findClosestVertices(i, 3, neighbors[i], distance);
    for (size_t j = 0; j < neighbors[i].size(); j++) {
      if (i > neighbors[i][j]) continue;  // if 1 -> 6, 6 -> 1 too.
      if (distance[i] < 2*patch_size_) {
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
}

bool terrain_slam::TerrainSlam::processCloud(const CloudPatchPtr& c, pcl::PointCloud<pcl::PointXYZ>::Ptr& output) {
  // Convert cloud to PCL
  std::cout << "Converting cloud to PCL..." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(c->cloud);

  if (cloud->size() < min_pts_) return false;

  // Remove outliers
  std::cout << "Removing outliers..." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  double min_z = 1.5;   // m
  double max_z = 10.0;  // m
  double neighbors_radius = 0.1;  // m
  int min_neighbors = 10;
  pcl_tools::removeOutliers(cloud, min_z, max_z, neighbors_radius, min_neighbors, filtered);

  pcl_tools::saveCloud(filtered, "outl", c->getId());

  if (filtered->size() < min_pts_) return false;

  // // Voxel Grid filter
  // std::cout << "Voxel filtering the cloud..." << std::endl;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr vgrid(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl_tools::voxelGrid(filtered, 0.05, vgrid);

  // if (vgrid->size() < min_pts_) return false;

  // Radomly sampling the cloud
  std::cout << "Randomly downsampling the cloud... " ;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_grid(new pcl::PointCloud<pcl::PointXYZ>());
  int num_points = 8000;
  double radius_search = 0.25;
  pcl_tools::randomlySampleGP(filtered, num_points, radius_search, output);
  if (output->points.size() < num_points) {
    pcl_tools::voxelGrid(filtered, 0.1, output);
  }

  std::cout << output->size() << " points." << std::endl;
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool terrain_slam::TerrainSlam::findTransform(const vector<CloudPatchPtr> &c,
                                              int id1,
                                              int id2,
                                              Eigen::Matrix4d& transform) {
  // Adjust
  std::cout << "Adjusting " << id1 << " to " << id2 << "..." << std::endl;
  // adj_.reset(new BruteForceAdjuster(-7.0, 7.0,  // x
  //                                   -7.0, 7.0,  // y
  //                                     0, 0,  // z
  //                                     0, 0,  // yaw
  //                                     0.1,       // x-y resolution
  //                                     0.5,       // z resolution
  //                                     0.5));    // yaw resolution
  CloudPatchPtr c1(c.at(id1));
  CloudPatchPtr c2(c.at(id2));
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_orig(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_orig(new pcl::PointCloud<pcl::PointXYZ>());
  source_orig = c1->cloud;
  target_orig = c2->cloud;

  // Preprocess clouds (remove outliers, grid and random sample)
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_grid(new pcl::PointCloud<pcl::PointXYZ>());
  bool res1 = processCloud(c1, source_grid);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_grid(new pcl::PointCloud<pcl::PointXYZ>());
  bool res2 = processCloud(c2, target_grid);

  if (res1 && res2) {
    // Copy the grids to the patches
    c1->cloud = source_grid;
    c2->cloud = target_grid;

    // Prepare KdTree structure for kNN
    c1->updateSearchTree();

    std::cout << "Running adjuster (random)... " << std::endl;
    Eigen::Matrix4Xd relative = adj_->adjust(c1, c2);

    // Transform pointcloud and save them for debugging
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_tf(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*target_grid, *target_tf, relative.cast<float>());
    std::ostringstream os;
    os << "tf_" << id1 << "_" << id2;
    pcl_tools::saveCloud(source_grid, os.str(), id1);
    pcl_tools::saveCloud(target_tf, os.str(), id2);

    // Return points at original state and save the transformation
    c1->cloud = source_orig;
    c2->cloud = target_orig;
    c2->transform.T = c1->transform.T*relative;
    return true;
  } else {
    return false;
  }



  // Get data
  // bool local_frame = true;
  // bool use_grid = false;
  // Eigen::Matrix4Xd p1 = c1->getPoints(local_frame, use_grid);
  // Eigen::Matrix4Xd p2 = c2->getPoints(local_frame, use_grid);

  // // Convert cloud to PCL
  // std::cout << "Converting cloud to PCL..." << std::endl;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(c1->cloud);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(c2->cloud);
  // pcl_tools::saveCloud(source_cloud, "orig", id1);
  // pcl_tools::saveCloud(target_cloud, "orig", id2);

  // // PC::Ptr source_dd(new PC);
  // // pcl_tools::preprocessCloud(source_cloud, id1, source_dd);

  // // Remove outliers
  // std::cout << "Removing outliers..." << std::endl;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr source_filt(new pcl::PointCloud<pcl::PointXYZ>());
  // pcl::PointCloud<pcl::PointXYZ>::Ptr target_filt(new pcl::PointCloud<pcl::PointXYZ>());
  // double min_z = 0.5;   // m
  // double max_z = 10.0;  // m
  // double neighbors_radius = 0.3;  // m
  // int min_neighbors = 50;
  // pcl_tools::removeOutliers(source_cloud, min_z, max_z, neighbors_radius, min_neighbors, source_filt);
  // pcl_tools::removeOutliers(target_cloud, min_z, max_z, neighbors_radius, min_neighbors, target_filt);
  // pcl_tools::saveCloud(source_filt, "outl", id1);
  // pcl_tools::saveCloud(target_filt, "outl", id2);

  // // Smooth clouds (DOES NOT WORK!)
  // // std::cout << "Smoothing clouds..." << std::endl;
  // // pcl::PointCloud<pcl::PointNormal>::Ptr source_smoothed(new pcl::PointCloud<pcl::PointNormal>());
  // // pcl::PointCloud<pcl::PointNormal>::Ptr target_smoothed(new pcl::PointCloud<pcl::PointNormal>());
  // // pcl_tools::smooth(source_filt, source_smoothed);
  // // pcl_tools::smooth(target_filt, target_smoothed);
  // // pcl_tools::saveCloud(source_smoothed, "smooth", id1);
  // // pcl_tools::saveCloud(target_smoothed, "smooth", id2);

  // // pcl::PointCloud<pcl::PointXYZ>::Ptr source_smoothed2(new pcl::PointCloud<pcl::PointXYZ>());
  // // pcl::PointCloud<pcl::PointXYZ>::Ptr target_smoothed2(new pcl::PointCloud<pcl::PointXYZ>());
  // // pcl::copyPointCloud(*source_smoothed, *source_smoothed2);
  // // pcl::copyPointCloud(*target_smoothed, *target_smoothed2);

  // // Voxel Grid filter
  // // std::cout << "Voxel filtering the cloud..." << std::endl;
  // // pcl::PointCloud<pcl::PointXYZ>::Ptr source_vgrid(new pcl::PointCloud<pcl::PointXYZ>);
  // // pcl::PointCloud<pcl::PointXYZ>::Ptr target_vgrid(new pcl::PointCloud<pcl::PointXYZ>);
  // // pcl_tools::voxelGrid(source_cloud, 0.15, source_vgrid);
  // // pcl_tools::voxelGrid(target_cloud, 0.15, target_vgrid);
  // // pcl_tools::saveCloud(source_vgrid, "vgrid", id1);
  // // pcl_tools::saveCloud(target_vgrid, "vgrid", id2);

  // // Radomly sampling the cloud
  // std::cout << "Randomly downsampling the cloud..." << std::endl;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr source_grid(new pcl::PointCloud<pcl::PointXYZ>());
  // pcl::PointCloud<pcl::PointXYZ>::Ptr target_grid(new pcl::PointCloud<pcl::PointXYZ>());
  // int num_points = 8000;
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
  // success = pcl_tools::icp(source_grid, target_tf, tf, score);
  // std::cout << "ICP fitness score: " << score << std::endl;

  // if (success) {
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr source_tf(new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::transformPointCloud(*source_grid, *source_tf, tf);
  //   pcl_tools::saveCloud(source_tf, "icp", id1);
  //   pcl_tools::saveCloud(target_tf, "icp", id2);
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
