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

#include <terrain_slam/features/features.h>
#include <terrain_slam/features/keypoints.h>

// Generic pcl
#include <pcl/common/common.h>
#include <pcl/features/intensity_spin.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/visualization/histogram_visualizer.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>

namespace po = boost::program_options;
using namespace std;
using namespace cv;

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
      // std::cout << "[INFO]: Find transform between " << id1 << " and " << id2 << std::endl;
      // findTransform(patches, id1, id2);
    }

    findTransform(patches, 14, 21);
    findTransform(patches, 0, 27);

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
    } else {
      cout << "[ERROR]: Unable to open camera pose file at " << cloud_paths[i]
           << ".  Please verify the path." << endl;
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

  // Grid patches
  std::cout << "[INFO]: Gridding... (parallel)" << std::endl;
#pragma omp parallel for
  for (size_t i = 0; i < patches.size(); i++) {
    patches.at(i)->grid();
  }
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

  if (false) {
    // Convert cloud to PCL
    typedef pcl::PointXYZ PT;
    typedef pcl::PointCloud<PT> PC;
    PC::Ptr source_cloud = pcl_tools::toPCL(c[id1]->getPoints(true, false));
    PC::Ptr target_cloud = pcl_tools::toPCL(c[id2]->getPoints(true, false));

    std::cout << "source_cloud->points.size() " << source_cloud->points.size() << std::endl;
    std::cout << "target_cloud->points.size() " << target_cloud->points.size() << std::endl;

    double feat_radius_search = 0.08;
    double normal_radius_search = 0.05;

    // Find keypoints
    Keypoints kp(KP_HARRIS_3D, normal_radius_search);
    PC::Ptr source_keypoints(new PC);
    PC::Ptr target_keypoints(new PC);
    kp.compute(source_cloud, source_keypoints);
    kp.compute(target_cloud, target_keypoints);

    std::cout << "source_keypoints->points.size() " << source_keypoints->points.size() << std::endl;
    std::cout << "target_keypoints->points.size() " << target_keypoints->points.size() << std::endl;

    // Extract features
    pcl::PointCloud<pcl::Histogram<32> >::Ptr source_features(new pcl::PointCloud<pcl::Histogram<32> >);
    pcl::PointCloud<pcl::Histogram<32> >::Ptr target_features(new pcl::PointCloud<pcl::Histogram<32> >);
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_intensities(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoint_intensities(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_intensities(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoint_intensities(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::IntensityGradient>::Ptr source_gradients(new pcl::PointCloud<pcl::IntensityGradient>);
    pcl::PointCloud<pcl::IntensityGradient>::Ptr target_gradients(new pcl::PointCloud<pcl::IntensityGradient>);
    pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
    Tools::estimateNormals(source_cloud, source_normals, normal_radius_search);
    Tools::estimateNormals(target_cloud, target_normals, normal_radius_search);
    Tools::PointCloudXYZtoXYZI(*source_keypoints, *source_keypoint_intensities);
    Tools::PointCloudXYZtoXYZI(*target_keypoints, *target_keypoint_intensities);
    Tools::PointCloudXYZtoXYZI(*source_cloud, *source_intensities);
    Tools::PointCloudXYZtoXYZI(*target_cloud, *target_intensities);
    Tools::computeGradient(source_intensities, source_normals, source_gradients, normal_radius_search);
    Tools::computeGradient(target_intensities, target_normals, target_gradients, normal_radius_search);

    std::cout << "source_keypoint_intensities->points.size() " << source_keypoint_intensities->points.size() << std::endl;
    std::cout << "target_keypoint_intensities->points.size() " << target_keypoint_intensities->points.size() << std::endl;

    std::cout << "source_gradients->points.size() " << source_gradients->points.size() << std::endl;
    std::cout << "target_gradients->points.size() " << target_gradients->points.size() << std::endl;

    pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, pcl::Histogram<32> > rift;
    rift.setRadiusSearch(feat_radius_search);
    rift.setNrDistanceBins(4);
    rift.setNrGradientBins(8);
    rift.setInputCloud(source_keypoint_intensities);
    rift.setSearchSurface(source_intensities);
    rift.setInputGradient(source_gradients);
    rift.compute(*source_features);
    rift.setInputCloud(target_keypoint_intensities);
    rift.setSearchSurface(target_intensities);
    rift.setInputGradient(target_gradients);
    rift.compute(*target_features);

    std::cout << "source_features->points.size() " << source_features->points.size() << std::endl;
    std::cout << "target_features->points.size() " << target_features->points.size() << std::endl;

    pcl::visualization::PCLHistogramVisualizer viewer;
    viewer.addFeatureHistogram(*source_features, 308);
    viewer.spin();

    int source_feat_size = source_features->points.size();
    int target_feat_size = target_features->points.size();

    // Find correspondences
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    pcl::CorrespondencesPtr filtered_correspondences(new pcl::Correspondences);
    Features<pcl::Histogram<32> > feat;
    Eigen::Matrix4f ransac_tf;
    feat.findCorrespondences(source_features, target_features, correspondences);
    std::cout << "correspondences->size() " << correspondences->size() << std::endl;
    feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
    std::cout << "filtered_correspondences->size() " << filtered_correspondences->size() << std::endl;

    // Transform cloud
    std::string clouds_dir("../adjusted/");
    std::string pcd = std::to_string(id1) + "__" + std::to_string(id2) + "sift.pcd";
    PC::Ptr output_ransac(new PC);
    pcl::transformPointCloud(*source_cloud, *output_ransac, ransac_tf);
    pcl::io::savePCDFile(clouds_dir + pcd, *output_ransac);

  }

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
  adj_->adjust(c1, c2);
  std::cout << "Done!" << std::endl;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  terrain_slam::TerrainSlam tslam(argc, argv);
  return 0;
}