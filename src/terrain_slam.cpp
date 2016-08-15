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
  patch_size_ = 12.0; // 5.0;
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

    // // Save original clouds
    // std::cout << "Saving original clouds..." << std::endl;
    // #pragma omp parallel for
    // for (size_t i = 0; i < patches_.size(); i++) {
    //   Eigen::Isometry3d pose = graph_->getVertexPose(i);
    //   CloudPatchPtr c(patches_.at(i));
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf(new pcl::PointCloud<pcl::PointXYZ>());
    //   pcl::transformPointCloud(*c->cloud, *cloud_tf, pose.matrix());
    //   pcl_tools::saveCloud(cloud_tf, "global", c->getId());
    //   pcl_tools::saveCloud(c->cloud, "local", c->getId());

    //   pcl::PointCloud<pcl::PointXYZ>::Ptr dummy(new pcl::PointCloud<pcl::PointXYZ>());
    //   bool res = processCloud(c, dummy);
    // }

    for (size_t i = 0; i < candidates.size(); i++) {
      int id1 = candidates[i].first;
      int id2 = candidates[i].second;
      // int id1 = 6;  // MMC
      // int id2 = 9;  // MMC
      Eigen::Matrix4d edge;
      bool res = findTransform(patches_, id1, id2, edge);
      if (res) {
        Eigen::Isometry3d edge_iso = eigen_tools::toIsometry(edge);
        graph_->addEdge(id1, id2, edge_iso, LC_WEIGHT);
        graph_->run();
        graph_->saveGraph();
      }
    }

    // Last loop closure will already optimize and save the graph
    // graph_->run();
    // graph_->saveGraph();

    // save optimized clouds
    #pragma omp parallel for
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

    bool close_patch = robot_distance > patch_size_;
    close_patch = close_patch || centroid_distance > patch_size_;
    close_patch = close_patch || i == lines.size() - 1;
    close_patch = close_patch || jump;

    double mean, stdev;
    lines[i]->fitLine(stdev);
    std::cout << stdev << " " << close_patch << std::endl;

    if (close_patch) {
      // create new patch
      CloudPatchPtr patch(new CloudPatch);
      patch->setId(patch_idx);
      int middle_line = (i + pivot_idx)/2;
      patch->transform = lines[middle_line]->transform;

      for (size_t p = pivot_idx; p <= i; p++) {
        patch->add(*(lines[p]));
      }

      // add to vector
      patches.push_back(patch);

      // if (robot_distance > patch_size_) std::cout << "Robot distance " << endl;
      // if (centroid_distance > patch_size_) std::cout << "Centroid " << endl;
      // if (i == lines.size() - 1) std::cout << "Size " << endl;
      // if (jump) std::cout << "Jump " << endl;

      // add to graph
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

  // TODO sorting?

  vector<vector<int> > neighbors(patches.size());
  for (size_t i = 0; i < patches.size(); i++) {
    vector<double> distance;
    graph_->findClosestVertices(i, 10, neighbors[i], distance);
    cout << "[INFO]: ByDistance " << i << " - ";
    for (size_t j = 0; j < neighbors[i].size(); j++) {
      if (i > neighbors[i][j]) continue;  // if 1 -> 6, 6 -> 1 too.
      if (distance[j] < 1.0*patch_size_) {
        pair<int, int> p = make_pair(i, neighbors[i][j]);
        // Check if the pair already exist
        if (std::find(candidates.begin(),
                       candidates.end(),
                       p) == candidates.end()) {
          candidates.push_back(p);
          cout << neighbors[i][j] << " ";
        }
      }
    }
    cout << endl;
  }
}

bool terrain_slam::TerrainSlam::processCloud(const CloudPatchPtr& c, pcl::PointCloud<pcl::PointXYZ>::Ptr& output) {
  if (c->processed) {
    output = c->cloud2;
    return true;
  } else {

    // Convert cloud to PCL
    // std::cout << "Converting cloud to PCL..." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(c->cloud);

    if (cloud->size() < min_pts_) return false;

    // Remove outliers
    // std::cout << "Removing outliers 1/2..." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    double min_z = 1.5;   // m
    double max_z = 25.0;  // m
    double neighbors_radius = 0.1;  // m
    int min_neighbors = 10;
    pcl_tools::removeOutliers(cloud, min_z, max_z, neighbors_radius, min_neighbors, filtered);

    pcl_tools::saveCloud(filtered, "outl1", c->getId());

    if (filtered->size() < min_pts_) return false;

    double a = area(filtered);
    // std::cout << "Current cloud area: " << a << std::endl;
    double point_density = 200.0;
    int num_points = std::min((int)(point_density * a), 100000);
    // std::cout << "Points: " << point_density * a << std::endl;
    // std::cout << "Points: " << num_points << std::endl;

    // Radomly sampling the cloud
    // std::cout << "Randomly downsampling the cloud... " << std::endl;;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_grid(new pcl::PointCloud<pcl::PointXYZ>());
    double radius_search = 0.25;
    pcl_tools::randomlySampleGP(filtered, num_points, radius_search, source_grid);
    if (source_grid->points.size() < num_points) {
      pcl_tools::voxelGrid(filtered, 0.1, source_grid);
    }

    pcl_tools::saveCloud(source_grid, "rand", c->getId());

    // std::cout << "Removing outliers 2/2..." << std::endl;
    neighbors_radius = 0.25;  // m
    min_neighbors = 2;
    pcl_tools::removeOutliers(source_grid, min_z, max_z, neighbors_radius, min_neighbors, output);

    pcl_tools::saveCloud(output, "outl2", c->getId());

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_smooth(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_tools::smooth(output, *output_smooth);
    pcl_tools::saveCloud(output_smooth, "smooth", c->getId());

    // std::cout << output->size() << " points." << std::endl;
    pcl::copyPointCloud(*output, *(c->cloud2));
    c->processed = true;
    return true;
  }
}

void terrain_slam::TerrainSlam::overlap(CloudPatchConstPtr c1,
                                        CloudPatchConstPtr c2,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr ov) {
  // Cloud 1 is going to be fixed. Compute its alpha shape
  pcl::PointCloud<pcl::PointXYZ>::Ptr fixed (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_target(new pcl::PointCloud<pcl::PointXYZ>());
  fixed  = c1->cloud;
  target = c2->cloud;

  Eigen::Matrix4d t1 = c1->transform.T;
  Eigen::Matrix4d t2 = c2->transform.T;
  Eigen::Matrix4d relative = t1.inverse() * t2;
  pcl::transformPointCloud(*target, *transformed_target, relative.cast<float>());
  pcl_tools::saveCloud(fixed, "preoverlap", c1->getId());
  pcl_tools::saveCloud(transformed_target, "preoverlap", c2->getId());

  // Prepare the alpha shape
  GreedyProjector gp(1.0);  // Scoring spoon radius (m)
  gp.setInputCloud(fixed);

  // Loop though the other cloud and add points that lie inside
  for (size_t i = 0; i < target->size(); i++) {
    pcl::PointXYZ transformed_pt = transformed_target->points[i];

    // std::vector<pcl::PointXYZ> points = gp.locate(transformed_pt);
    // if (points.size() > 0) {

    if (gp.isInside(transformed_pt)) {
      pcl::PointXYZ pt = target->points[i];
      ov->push_back(pt);
    }
  }
}

double terrain_slam::TerrainSlam::area(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  // Prepare the alpha shape
  GreedyProjector gp(0.04);  // Scoring spoon radius (m)
  gp.setInputCloud(cloud);
  return gp.area();
}

////////////////////////////////////////////////////////////////////////////////

bool terrain_slam::TerrainSlam::findTransform(const vector<CloudPatchPtr> &c,
                                              int id1,
                                              int id2,
                                              Eigen::Matrix4d& transform) {
  // Adjust
  std::cout << "Adjusting " << id1 << " to " << id2 << "..." << std::endl;

  CloudPatchPtr c1(c.at(id1));
  CloudPatchPtr c2(c.at(id2));

  // Copy pointclouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_orig(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_orig(new pcl::PointCloud<pcl::PointXYZ>());
  source_orig = c1->cloud;
  target_orig = c2->cloud;

  // Prepare transformation
  Eigen::Matrix4Xd original = c2->transform.T;
  Eigen::Matrix4Xd relative_pre;
  Eigen::Matrix4Xd relative_post;

  // TODO check which clouds are more suitable to match
  // // Add prior pointcloud
  // pcl::PointCloud<pcl::PointXYZ>::Ptr next_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  // next_cloud = c.at(id2-1)->cloud;
  // Eigen::Matrix4d tf_relative = c2->transform.T.inverse() * c.at(id2-1)->transform.T;
  // pcl::transformPointCloud(*next_cloud, *next_cloud, tf_relative.cast<float>());
  // *target_orig += *next_cloud;

  // Preprocess clouds (remove outliers, grid and random sample)
  std::cout << "Preprocessing clouds..." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_grid(new pcl::PointCloud<pcl::PointXYZ>());
  bool res1 = processCloud(c1, source_grid);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_grid(new pcl::PointCloud<pcl::PointXYZ>());
  bool res2 = processCloud(c2, target_grid);

  // Do they overlap?
  std::cout << "Computing overlap... " << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ov_cloud1(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr ov_cloud2(new pcl::PointCloud<pcl::PointXYZ>());
  overlap(c1, c2, ov_cloud2);
  overlap(c2, c1, ov_cloud1);

  double orig_area_1 = area(c1->cloud);
  double overlap_area_1 = area(ov_cloud1);

  double orig_area_2 = area(c2->cloud);
  double overlap_area_2 = area(ov_cloud2);

  std::cout << "Overlap 1: " << overlap_area_1/orig_area_1*100 << "%%" << std::endl;
  std::cout << "Overlap 2: " << overlap_area_2/orig_area_2*100 << "%%" << std::endl;
  std::cout << "Size ratio 1/2: " << orig_area_1/orig_area_2*100 << "%%" << std::endl;
  std::cout << "Overlap 1 to 2: " << overlap_area_1/orig_area_2*100 << "%%" << std::endl;
  std::cout << "Overlap 2 to 1: " << overlap_area_2/orig_area_1*100 << "%%" << std::endl;

  if (res1 && res2) {

    // Smooth the clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_smooth(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_smooth(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_tools::smooth(source_grid, *source_smooth);
    pcl_tools::smooth(target_grid, *target_smooth);

    // Copy the smoothed to the patches
    c1->cloud = source_smooth;
    c2->cloud = target_smooth;

    // // TODO remove
    // c1->cloud = source_orig;
    // c2->cloud = target_orig;
    // return true;

    // Register
    std::cout << "Running pre-adjuster ... " << std::endl;
    adj_->reset();
    relative_pre = adj_->adjust(c1, c2);
    c2->transform.T = c1->transform.T*relative_pre;

    // Save results of this transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_smooth_tf(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*target_smooth, *target_smooth_tf, relative_pre.cast<float>());
    std::ostringstream oss1;
    oss1 << "pre_" << id1 << "_" << id2;
    pcl_tools::saveCloud(source_smooth, oss1.str(), id1);
    pcl_tools::saveCloud(target_smooth_tf, oss1.str(), id2);


    // Use the transformation for the original cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_tf(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*target_grid, *target_tf, relative_pre.cast<float>());

    // Align
    // Copy the smoothed to the patches
    c1->cloud = source_grid;
    c2->cloud = target_tf;

    // Register
    std::cout << "Running adjuster ... " << std::endl;
    adj_->reset();
    relative_post = adj_->adjust(c1, c2);
    c2->transform.T = c1->transform.T*relative_post;

    // Save results of this transformation
    pcl::transformPointCloud(*target_grid, *target_tf, relative_post.cast<float>());
    std::ostringstream oss2;
    oss2 << "post_" << id1 << "_" << id2;
    pcl_tools::saveCloud(source_smooth, oss2.str(), id1);
    pcl_tools::saveCloud(target_tf, oss2.str(), id2);

    // Return points at original state and save the transformation
    c1->cloud = source_orig;
    c2->cloud = target_orig;

    return true;
  } else {
    return false;
  }


  // // Pre-align:
  // pcl::PointCloud<pcl::PointXYZ>::Ptr source_segmented(new pcl::PointCloud<pcl::PointXYZ>());
  // pcl_tools::segmentation(source_grid, source_segmented);
  // pcl_tools::saveCloud(source_segmented, "seg", c1->getId());

  // pcl::PointCloud<pcl::PointXYZ>::Ptr target_segmented(new pcl::PointCloud<pcl::PointXYZ>());
  // pcl_tools::segmentation(target_grid, target_segmented);
  // pcl_tools::saveCloud(target_segmented, "seg", c2->getId());

  // Eigen::Vector4f source_centroid, target_centroid;
  // pcl::compute3DCentroid(*source_segmented, source_centroid);
  // pcl::compute3DCentroid(*target_segmented, target_centroid);

  // Eigen::Vector4f translation = source_centroid - target_centroid;

  // Eigen::Matrix4f pre_tf = Eigen::Matrix4f::Identity();
  // pre_tf(0,3) = translation(0);
  // pre_tf(1,3) = translation(1);
  // pre_tf(2,3) = translation(2);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr target_segmented_tf(new pcl::PointCloud<pcl::PointXYZ>());
  // pcl::transformPointCloud(*target_segmented, *target_segmented_tf, pre_tf);

  // pcl_tools::saveCloud(target_segmented_tf, "segtf", id2);

  // // ICP
  // std::cout << "Registering with NormalDistributionsTransform..." << std::endl;
  // bool success = false;
  // Eigen::Matrix4f tf;
  // double score = -1.0;
  // // success = pcl_tools::icp(source_segmented, target_segmented_tf, tf, score);

  // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  // ndt.setTransformationEpsilon(0.001);
  // ndt.setStepSize(0.005);
  // ndt.setResolution(0.1);
  // ndt.setMaximumIterations(200);
  // ndt.setInputSource(source_segmented);
  // ndt.setInputTarget(target_segmented_tf);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
  // ndt.align(*aligned, tf);
  // score = ndt.getFitnessScore();
  // tf = ndt.getFinalTransformation();
  // success = ndt.hasConverged();


  // std::cout << "ICP fitness score: " << score << " converged? " << success << std::endl;
  // if (success) {
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr source_tf(new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::transformPointCloud(*source_grid, *source_tf, tf);
  //   std::ostringstream os;
  //   os << "pre_icp_" << id1 << "_" << id2;
  //   pcl_tools::saveCloud(source_tf, os.str(), id1);
  //   pcl_tools::saveCloud(target_grid, os.str(), id2);

  //   std::cout << "PRE:\n" << pre_tf << std::endl;
  //   std::cout << "POST:\n" << tf << std::endl;
  // }


  // if (res1 && res2) {




    // pcl_tools::saveCloud(source_smooth, "smooth", id1);
    // pcl_tools::saveCloud(target_smooth, "smooth", id2);

    // // GROUND TRUTH
    // // How much I have to move c2 to register it to c1
    // double gt_x = 4.730406;
    // double gt_y = 2.4042;
    // double gt_z = 0.284647;

    // // Start from different positions
    // double xmov = 5.; //gt_x; //
    // double ymov = 5.; //gt_y; //
    // double zmov = 1.0; //gt_z; //
    // int count = 0;
    // Eigen::Matrix4Xd original = c2->transform.T;
    // Eigen::Matrix4Xd relative;
    // for (int xpos = 1; xpos < 2; xpos++) {
    //   for (int ypos = 1; ypos < 2; ypos++) {
    //     for (int zpos = 1; zpos < 2; zpos++) {
          // std::cout << "Running adjuster ... " << std::endl;

          // c2->transform.T = original;
          // c2->transform.T(0, 3) += xpos*xmov;
          // c2->transform.T(1, 3) += ypos*ymov;
          // c2->transform.T(2, 3) += zpos*zmov;

          // std::cout << "Computing overlap... " << std::endl;
          // pcl::PointCloud<pcl::PointXYZ>::Ptr ov_cloud1(new pcl::PointCloud<pcl::PointXYZ>());
          // pcl::PointCloud<pcl::PointXYZ>::Ptr ov_cloud2(new pcl::PointCloud<pcl::PointXYZ>());
          // overlap(c1, c2, ov_cloud2);
          // overlap(c2, c1, ov_cloud1);

          // double orig_area_1 = area(c1->cloud);
          // double overlap_area_1 = area(ov_cloud1);

          // double orig_area_2 = area(c2->cloud);
          // double overlap_area_2 = area(ov_cloud2);

          // std::cout << "Overlap 1: " << overlap_area_1/orig_area_1*100 << "%%" << std::endl;
          // std::cout << "Overlap 2: " << overlap_area_2/orig_area_2*100 << "%%" << std::endl;
          // std::cout << "Size ratio 1/2: " << orig_area_1/orig_area_2 << "%%" << std::endl;
          // std::cout << "Overlap 1 to 2: " << overlap_area_1/orig_area_2*100 << "%%" << std::endl;
          // std::cout << "Overlap 2 to 1: " << overlap_area_2/orig_area_1*100 << "%%" << std::endl;

          // pcl_tools::saveCloud(ov_cloud1, "overlap", id1);
          // pcl_tools::saveCloud(ov_cloud2, "overlap", id2);

          // // Save the original cloud
          // source_grid = c1->cloud;
          // target_grid = c2->cloud;
          // // Put the overlap version
          // c1->cloud = ov_cloud1;
          // c2->cloud = ov_cloud2;


          // // ICP
          // std::cout << "Registering with ICP..." << std::endl;
          // bool success = false;
          // Eigen::Matrix4f tf;
          // double score = -1.0;
          // success = pcl_tools::icp(source_grid, target_grid, tf, score);
          // std::cout << "ICP fitness score: " << score << std::endl;
          // if (success) {
          //   pcl::PointCloud<pcl::PointXYZ>::Ptr source_tf(new pcl::PointCloud<pcl::PointXYZ>);
          //   pcl::transformPointCloud(*source_grid, *source_tf, tf);
          //   std::ostringstream os;
          //   os << "icp_" << id1 << "_" << id2;
          //   pcl_tools::saveCloud(source_tf, os.str(), id1);
          //   pcl_tools::saveCloud(target_grid, os.str(), id2);
          // }

          // // NDT
          // std::cout << "Registering with NDT..." << std::endl;
          // success = pcl_tools::normalDistributionsTransform(source_grid, target_grid, tf, score);
          // std::cout << "ICP fitness score: " << score << std::endl;
          // if (success) {
          //   pcl::PointCloud<pcl::PointXYZ>::Ptr source_tf(new pcl::PointCloud<pcl::PointXYZ>);
          //   pcl::transformPointCloud(*source_grid, *source_tf, tf);
          //   std::ostringstream os;
          //   os << "ndt_" << id1 << "_" << id2;
          //   pcl_tools::saveCloud(source_tf, os.str(), id1);
          //   pcl_tools::saveCloud(target_grid, os.str(), id2);
          // }



          // // Prepare KdTree structure for kNN
          // c1->updateSearchTree();

          // // Register
          // adj_->reset();
          // relative = adj_->adjust(c1, c2);

          // // Put the original cloud back
          // c1->cloud = source_grid;
          // c2->cloud = target_grid;

          // // Transform pointcloud and save them for debugging
          // pcl::PointCloud<pcl::PointXYZ>::Ptr target_tf(new pcl::PointCloud<pcl::PointXYZ>());
          // pcl::transformPointCloud(*target_grid, *target_tf, relative.cast<float>());
          // std::ostringstream os;
          // os << "tf_" << id1 << "_" << id2;// << "_" << count;
          // pcl_tools::saveCloud(source_grid, os.str(), id1);
          // pcl_tools::saveCloud(target_tf, os.str(), id2);

          // count++;
    //     }
    //   }
    // }

    // Return points at original state and save the transformation
    // c1->cloud = source_orig;
    // c2->cloud = target_orig;
    // c2->transform.T = c1->transform.T*relative;
    // return true;
  // } else {
    // return false;
  // }
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  terrain_slam::TerrainSlam tslam(argc, argv);
  return 0;
}
