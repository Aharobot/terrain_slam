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

#ifndef PLY_LOADER_H
#define PLY_LOADER_H

#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class PlyLoader {
public:
  PlyLoader() {}

  Eigen::Matrix4Xd load(const std::string &filename) {
    try {
      std::ifstream infile;
      infile.open(filename);
      std::string line;
      bool header = true;
      std::vector<Eigen::Vector4d> pts;
      while (std::getline(infile, line)) {
        if (line.compare("end_header") == 0) header = false;
        if (!header) {
          Eigen::Vector4d p = readPoint(line);
          pts.push_back(p);
        }
      }
      infile.close();

      Eigen::Matrix4Xd m(4, pts.size());
      for (size_t i = 0; i < pts.size(); i++)
        m.col(i) = pts[i];
      return m;
    } catch(std::exception const& e) {
      std::cout << "There was an error: " << e.what() << std::endl;
    }
  }

private:
  Eigen::Vector4d readPoint(const std::string &line) {
    Eigen::Vector3d p;
    std::istringstream iss(line);
    iss >> p(0);
    iss >> p(1);
    iss >> p(2);
    return p.homogeneous();
  }
};

#endif // PLY_LOADER_H
