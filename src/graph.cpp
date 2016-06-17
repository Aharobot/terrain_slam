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

#include <terrain_slam/graph.h>

using namespace std;

terrain_slam::Graph::Graph() {
  init();
}

void terrain_slam::Graph::init() {
  // Initialize the g2o graph optimizer
  g2o::BlockSolverX::LinearSolverType * linear_solver_ptr;
  linear_solver_ptr = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
  g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linear_solver_ptr);
  g2o::OptimizationAlgorithmLevenberg * solver =
    new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  graph_optimizer_.setAlgorithm(solver);
}

int terrain_slam::Graph::addVertex(const Eigen::Isometry3d& pose) {
  boost::mutex::scoped_lock lock(mutex_graph_);

  // Set node id equal to graph size
  int id = graph_optimizer_.vertices().size();

  // std::cout << "Adding vertex " << id << std::endl;

  // Build the vertex
  g2o::VertexSE3* cur_vertex = new g2o::VertexSE3();
  cur_vertex->setId(id);
  cur_vertex->setEstimate(pose);
  if (id == 0) {
    // First time, no edges.
    cur_vertex->setFixed(true);
  }
  graph_optimizer_.addVertex(cur_vertex);
  return id;
}

void terrain_slam::Graph::addEdge(int i, int j, const Eigen::Isometry3d& edge, int sigma) {
  boost::mutex::scoped_lock lock(mutex_graph_);
  // Get the vertices
  g2o::VertexSE3* v_i = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[i]);
  g2o::VertexSE3* v_j = dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[j]);

  // std::cout << "Adding edge between " << i << " and " << j << std::endl;

  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity() * (double)sigma;

  // Add the new edge to graph
  g2o::EdgeSE3* e = new g2o::EdgeSE3();
  Eigen::Isometry3d t(edge);
  e->setVertex(0, v_i);
  e->setVertex(1, v_j);
  e->setMeasurement(t);
  e->setInformation(information);
  graph_optimizer_.addEdge(e);
}

void terrain_slam::Graph::findClosestVertices(int vertex_id,
                                              int best_n,
                                              vector<int> &neighbors,
                                              vector<double> &distances) {
  // Init
  neighbors.clear();
  Eigen::Isometry3d vertex_pose = getVertexPose(vertex_id);

  // Loop thought all the other nodes
  vector< pair< int,double > > neighbor_distances;
  for (size_t i = 0; i < graph_optimizer_.vertices().size(); i++) {
    // Do not allow same
    if ((int)i == vertex_id) continue;

    // Do not allow consecutive
    if ((int)i == vertex_id - 1) continue;
    if ((int)i == vertex_id + 1) continue;

    // Get the node pose
    Eigen::Isometry3d cur_pose = getVertexPose(i);

    Eigen::Vector3d d = cur_pose.translation() - vertex_pose.translation();

    double dist = d.norm();
    neighbor_distances.push_back(make_pair(i, dist));
  }

  // Exit if no neighbors
  if (neighbor_distances.size() == 0) return;

  // Sort the neighbors
  sort(neighbor_distances.begin(), neighbor_distances.end(), sortByDistance);

  // Min number
  if ((int)neighbor_distances.size() < best_n)
    best_n = neighbor_distances.size();

  for (int i=0; i<best_n; i++) {
    neighbors.push_back(neighbor_distances[i].first);
    distances.push_back(neighbor_distances[i].second);
  }
}

Eigen::Isometry3d terrain_slam::Graph::getVertexPose(int vertex_id, bool lock) {
  if (lock) {
    boost::mutex::scoped_lock lock(mutex_graph_);
    if( vertex_id >= 0) {
      g2o::VertexSE3* vertex =  dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[vertex_id]);
      return vertex->estimate();
    } else {
      Eigen::Isometry3d tmp;
      tmp.setIdentity();
      return tmp;
    }
  } else {
    if( vertex_id >= 0) {
      g2o::VertexSE3* vertex =  dynamic_cast<g2o::VertexSE3*>(graph_optimizer_.vertices()[vertex_id]);
      return vertex->estimate();
    } else {
      Eigen::Isometry3d tmp;
      tmp.setIdentity();
      return tmp;
    }
  }
}

void terrain_slam::Graph::saveGraph() {
  string vertices_file, edges_file;
  vertices_file = WORKING_DIRECTORY + "graph_vertices.txt";
  edges_file = WORKING_DIRECTORY + "graph_edges.txt";

  // Open to append
  fstream f_vertices(vertices_file.c_str(), ios::out | ios::trunc);
  fstream f_edges(edges_file.c_str(), ios::out | ios::trunc);

  boost::mutex::scoped_lock lock(mutex_graph_);

  // Output the vertices file
  for (size_t i = 0; i < graph_optimizer_.vertices().size(); i++) {
    Eigen::Isometry3d pose = getVertexPose(i, false);
    Eigen::Quaterniond q(pose.rotation());
    Eigen::Vector3d t(pose.translation());
    f_vertices << fixed <<
      setprecision(6) <<
         i << "," <<
      t(0) << "," <<
      t(1) << "," <<
      t(2) << "," <<
      q.x() << "," <<
      q.y() << "," <<
      q.z() << "," <<
      q.w() <<  endl;
  }
  f_vertices.close();

  // Output the edges file
  for (g2o::OptimizableGraph::EdgeSet::iterator it = graph_optimizer_.edges().begin();
      it != graph_optimizer_.edges().end(); it++) {
    g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*> (*it);
    if (e) {
      // Get the frames corresponding to these edges
      int vertex_a = e->vertices()[0]->id();
      int vertex_b = e->vertices()[1]->id();

      // if (abs(vertex_a - vertex_b) > 1)
      {

        Eigen::Isometry3d pose0 = getVertexPose(vertex_a, false);//*camera2odom_;
        Eigen::Isometry3d pose1 = getVertexPose(vertex_b, false);//*camera2odom_;
        Eigen::Quaterniond q0(pose0.rotation());
        Eigen::Vector3d t0(pose0.translation());
        Eigen::Quaterniond q1(pose1.rotation());
        Eigen::Vector3d t1(pose1.translation());

        // Extract the inliers
        Eigen::Matrix<double, 6, 6> information = e->information();
        int inliers = 0;
        if (information(0,0) > 0.0001)
          inliers = (int)information(0,0);

        // Write
        f_edges <<
          e->vertices()[0]->id() << "," <<
          e->vertices()[1]->id() << "," <<
          inliers << "," <<
          setprecision(6) <<
          t0(0) << "," <<
          t0(1) << "," <<
          t0(2) << "," <<
          q0.x() << "," <<
          q0.y() << "," <<
          q0.z() << "," <<
          q0.w() << "," <<
          t1(0) << "," <<
          t1(1) << "," <<
          t1(2) << "," <<
          q1.x() << "," <<
          q1.y() << "," <<
          q1.z() << "," <<
          q1.w() << endl;
      }
    }
  }
  f_edges.close();
}

void terrain_slam::Graph::run() {
  boost::mutex::scoped_lock lock(mutex_graph_);

  // Optimize!
  graph_optimizer_.initializeOptimization();
  graph_optimizer_.optimize(20);

  std::cout << "[INFO]: Optimization done in graph with "
            << graph_optimizer_.vertices().size() << " vertices." << std::endl;
}
