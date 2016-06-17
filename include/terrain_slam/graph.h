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

#ifndef GRAPH_H
#define GRAPH_H

#include <boost/thread.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

using namespace std;

static string WORKING_DIRECTORY("../output/");

namespace terrain_slam {
class Graph {
public:
  /** \brief Class constructor
   */
  Graph();

  /** \brief Initialize the graph
   */
  void init();

  /** \brief Starts graph
   */
  void run();

  /** \brief Add a vertex to the graph
   * @return the vertex id
   * \param Vertex pose
   */
  int addVertex(const Eigen::Isometry3d &pose);

  /** \brief Add an edge to the graph
   * \param Index of vertex 1
   * \param Index of vertex 2
   * \param Eigen::Isometry3dation between vertices
   * \param Sigma information
   */
  void addEdge(int i, int j, const Eigen::Isometry3d &edge, int sigma);

  /** \brief Get the closest neighbors by distance
   * \param The vertex id to retrieve its neighbors
   * \param Number of neighbors to be retrieved.
   * \param Will contain the list of best neighbors by distance.
   * \param Distance of the retrieved neighbours.
   */
  void findClosestVertices(int vertex_id,
                           int best_n,
                           vector<int> &neighbors,
                          vector<double> &distances);

  /** \brief Get the graph vertex pose
   * @return graph vertex pose
   * \param vertex id
   * \param set to true to lock the graph
   */
  Eigen::Isometry3d getVertexPose(int vertex_id, bool lock = true);

  /** \brief Save the graph to file
   */
  void saveGraph();

protected:
  g2o::SparseOptimizer graph_optimizer_;
  boost::mutex mutex_graph_;

  /** \brief Sort 2 pairs by size
   * @return true if pair 1 is smaller than pair 2
   * \param pair 1
   * \param pair 2
   */
  static bool sortByDistance(const pair<int, double> d1,
                             const pair<int, double> d2)  {
   return (d1.second < d2.second);
  }

};  // Class
}   // Namespace

#endif // GRAPH_H
