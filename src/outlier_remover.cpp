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


#include <terrain_slam/outlier_remover.h>
#include <nabo/nabo.h>

#include <iostream>

void terrain_slam::OutlierRemover::filter(Eigen::MatrixXd &output) {
  // Instance kNN tree
  Nabo::NNSearchD *nns = Nabo::NNSearchD::createKDTreeLinearHeap(input_);

  // Indices of the kNN points
  Eigen::VectorXi nn_indices(mean_k_);
  // Squared distance between the query and the kNN points
  Eigen::VectorXd nn_dists2(mean_k_);

  // Mean kNN distance for each point
  Eigen::VectorXd distances(input_.cols());

  int valid_distances = 0;

  for (size_t i = 0; i < input_.cols(); i++) {
    // kNN point per point
    nns->knn(input_.col(i), nn_indices, nn_dists2, mean_k_, 0,
             Nabo::NNSearchF::SORT_RESULTS);
    // Minimum distance (if mean_k_ == 2) or mean distance
    double dist_sum = 0;
    for (int j = 1; j < mean_k_; ++j) {
      dist_sum += sqrt(nn_dists2[j]);
    }
    distances[i] = dist_sum / static_cast<double>(mean_k_ - 1);
    if (dist_sum > 0) valid_distances++;
  }

  // Check if everything is right
  if (valid_distances != input_.cols())
    std::cerr << "Missing distance. Something is wrong..." << std::endl;

  // Estimate the mean and the standard deviation of the distance vector
  double mean = distances.mean();
  double variance = 0;
  for (size_t i = 0; i < distances.size(); i++) {
    variance += (distances[i] - mean) * (distances[i] - mean);
  }
  variance /= static_cast<double>(distances.size()) - 1;
  double stddev = sqrt(variance);

  // a distance that is bigger than this signals an outlier
  double distance_threshold = mean + std_mul_ * stddev;

  // Build a new cloud by neglecting outliers
  int nr_outliers = 0;
  int nr_inliers = 0;
  removed_indices_.resize(input_.cols());
  output.resize(input_.rows(), input_.cols());
  for (size_t i = 0; i < input_.cols(); i++) {
    if (distances[i] > distance_threshold) {
      removed_indices_[nr_outliers] = i;
      nr_outliers++;
    } else {
      output.col(nr_inliers) = input_.col(i);
      nr_inliers++;
    }
  }

  // Resize vector to actual size
  removed_indices_.conservativeResize(nr_outliers);
  output.conservativeResize(input_.rows(), nr_inliers);

  // cleanup kd-tree
  delete nns;
}
