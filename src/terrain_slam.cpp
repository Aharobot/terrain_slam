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

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <iostream>

namespace po = boost::program_options;
using namespace std;
using namespace cv;

terrain_slam::TerrainSlam::TerrainSlam(int argc, char** argv) {
  parseCommandLine(argc, argv);
}

void terrain_slam::TerrainSlam::parseCommandLine(int argc, char** argv) {
  try {
    po::options_description description("Terrain Slam");

    string pose_filename;
    string clouds_dir;

    description.add_options()
    ("help,h", "Display this help message")
    ("pose,p", po::value<string>(), "Input file with robot positions")
    ("clouds,c", po::value<string>(), "Input folder where the point clouds are");

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(description).run(), vm);
    //po::notify(vm);

    if (vm.count("help")) {
      cout << description;
    } else if (vm.count("pose") && vm.count("clouds")) {
      pose_filename   = vm["pose"].as<string>();
      clouds_dir = vm["clouds"].as<string>();
      std::cout.setf(std::ios::boolalpha);
      cout << "Running Terrain Slam with the following parameters:"
         << "\n\t* Clouds directory  : " << clouds_dir
         << "\n\t* Pose file         : " << pose_filename << endl;
      process(pose_filename, clouds_dir);
    } else {
      cout << "REQUIRED arguments were not provided." << endl;
      cout << description;
    }
  } catch(po::error e) {
    cerr << "Error: " << e.what() << ". Aborting" << endl;
  }
}

void terrain_slam::TerrainSlam::process(const std::string& pose_filename,
                                   const std::string& clouds_dir) {
  vector<Aff3> tf;
  bool success = readCameraPoses(pose_filename, tf);
  // vector<vector<Point3> > points = readPointClouds(clouds_dir);
}

bool terrain_slam::TerrainSlam::readCameraPoses(const std::string& filename, vector<Aff3>& tf) {
  try {
    if (boost::filesystem::exists(filename)) {
      FileStorage fs;
      fs.open(filename, FileStorage::READ);
      if (fs.isOpened()) {
        cout << "Camera parameter file found!" << endl;
        Mat poses;
        fs["poses"] >> poses;
        return cvToCGAL(poses, tf);
      } else {
        cout << "Unable to open camera pose file at " << filename
             << ".  Please verify the path." << endl;
        return false;
      }
    } else {
      std::cout << "Pose filename: " << filename << " does not exist!\n";
    }
  } catch (const boost::filesystem::filesystem_error& ex) {
    std::cout << ex.what() << '\n';
  }
  return false;
}

bool terrain_slam::TerrainSlam::cvToCGAL(const Mat& in, vector<Aff3>& out) {
  out.clear();

  // in Matrix contains (x, y, z, roll, pitch, yaw) in meters and degrees
  for (size_t i = 0; i < in.rows; i++) {
    double x = in.at<double>(i, 0);
    double y = in.at<double>(i, 1);
    double z = in.at<double>(i, 2);
    double roll  = in.at<double>(i, 3)*M_PI/180.0;
    double pitch = in.at<double>(i, 4)*M_PI/180.0;
    double yaw   = in.at<double>(i, 5)*M_PI/180.0;

    double ci = cos(roll);
    double cj = cos(pitch);
    double ch = cos(yaw);
    double si = sin(roll);
    double sj = sin(pitch);
    double sh = sin(yaw);
    double cc = ci * ch;
    double cs = ci * sh;
    double sc = si * ch;
    double ss = si * sh;

    double m00 = cj * ch;
    double m01 = sj * sc - cs;
    double m02 = sj * cc + ss;
    double m10 = cj * sh;
    double m11 = sj * ss + cc;
    double m12 = sj * cs - sc;
    double m20 = -sj;
    double m21 = cj * si;
    double m22 = cj * ci;
    double m03 = x;
    double m13 = y;
    double m23 = z;
    Aff3 tf(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
    out.push_back(tf);
  }
  return true;
}

int main(int argc, char** argv) {
  terrain_slam::TerrainSlam tslam(argc, argv);
  return 0;
}