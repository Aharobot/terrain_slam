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

#ifndef TERRAIN_SLAM_H
#define TERRAIN_SLAM_H

#include <terrain_slam/clouds.h>
#include <terrain_slam/graph.h>
#include <terrain_slam/adjuster.h>

#include <vector>

#define DEFAULT_WEIGHT 100

namespace terrain_slam {
class TerrainSlam {
public:
  TerrainSlam(int argc, char **argv);

  /**
   * @brief      Parse command line arguments
   *
   * @param[in]  argc        The argc command line argument
   * @param      argv        The argv command line argument
   *
   * @return     true if successful
   */
  bool parseCommandLine(int argc, char **argv);

  /**
   * @brief      Main process function
   */
  void process();

  /**
   * @brief      Reads files.
   *
   * @param[in]  cloud_names  The cloud names
   * @param[in]  cloud_paths  The cloud paths
   * @param      lines        The laser lines
   */
  void readFiles(const std::vector<std::string> &cloud_names,
                 const std::vector<std::string> &cloud_paths,
                 std::vector<LaserLine> &lines);

  /**
   * @brief      Use boost filesystem to explore the desired path for yml files
   * where the pointclouds are stored using OpenCV FileStorage class.
   *
   * @param[in]  path         The path
   * @param[in]  format       The extension format
   * @param      cloud_names  Output vector containing the names of the files
   * @param      cloud_paths  Output vector containing the full path to the files
   *
   * @return     true if any files have been found, false otherwise
   */
  bool getCloudPaths(const std::string &path, const std::string &format,
                     std::vector<std::string> &cloud_names,
                     std::vector<std::string> &cloud_paths);

  /**
   * @brief Using provided patch_size, splits the pointcloud into patches
   *
   * @param[in]   Lines       Input laser lines
   * @param       patches     Output vector of CloudPatches
   */
  void createPatches(const std::vector<LaserLine>& lines,
                     std::vector<CloudPatchPtr>& patches);

  /**
   * @brief Finds overlapping patches, and return their paired indexes
   *
   * @param[in]   patches     Input vector of CloudPatch
   * @param       candidates  Output vector of paired candidates
   */
  void lookForCandidates(const std::vector<CloudPatchPtr>& patches,
                         std::vector<std::pair<int, int> >& candidates);

  // int preprocessPoints(std::vector<cv::Point3d>& points, int idx);

  Transform findTransform(const std::vector<CloudPatchPtr> &c, int id1, int id2);

  // boost::shared_ptr<BruteForceAdjuster> adj_;
  boost::shared_ptr<Adjuster> adj_;
  boost::shared_ptr<Graph> graph_;

  std::string clouds_dir_;
  double patch_size_;
  int mean_k_;
  int std_mult_;
  bool debug_;
  bool filter_;

}; // Class
} // Namespace

#endif // TERRAIN_SLAM_H
