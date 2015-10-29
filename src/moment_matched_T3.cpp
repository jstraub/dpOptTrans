/* Copyright (c) 2015, Julian Straub <jstraub@csail.mit.edu> Licensed
 * under the MIT license. See the license file LICENSE.
 */

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

#include "dpvMFoptRot/pc_helper.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

void ComputeMoments(const pcl::PointCloud<pcl::PointXYZRGBNormal>& pc,
    Eigen::Matrix3d& S, Eigen::Vector3d& mu) {
  mu.fill(0.); S.fill(0.);
  double W = 0.;
  for (uint32_t i=0; i<pc.size(); ++i) {
    Eigen::Map<const Eigen::Vector3f> p(&(pc.at(i).x));
    double w = pc.at(i).curvature; // Stores the weights
    mu += p.cast<double>()*w;
    W+=w;
  }
  mu /= W;
  for (uint32_t i=0; i<pc.size(); ++i) {
    Eigen::Map<const Eigen::Vector3f> p(&(pc.at(i).x));
    double w = pc.at(i).curvature; // Stores the weights
    S += w*(p.cast<double>()-mu) * (p.cast<double>()-mu).transpose();
  }
  S /= W;
}

int main(int argc, char** argv) {
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("in_a,a", po::value<std::string>(), "path to first input file")
    ("in_b,b", po::value<std::string>(), "path to second input file")
    ("out,o", po::value<std::string>(), "path to output file")
    ("display,d", "display results")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  std::string pathA = "";
  std::string pathB = "";
  std::string pathOut = "";
  //  std::string mode = "";
  //  if(vm.count("mode")) mode = vm["mode"].as<std::string>();
  if(vm.count("in_a")) pathA = vm["in_a"].as<std::string>();
  if(vm.count("in_b")) pathB = vm["in_b"].as<std::string>();
  if(vm.count("out")) pathOut = vm["out"].as<std::string>();

  // Load point clouds.
  pcl::PointCloud<pcl::PointXYZRGBNormal> pcA, pcB;
  pcl::PLYReader reader;
  if (reader.read(pathA, pcA)) 
    std::cout << "error reading " << pathA << std::endl;
  else
    std::cout << "loaded pc from " << pathA << ": " << pcA.width << "x"
      << pcA.height << std::endl;
  if (reader.read(pathB, pcB)) 
    std::cout << "error reading " << pathB << std::endl;
  else
    std::cout << "loaded pc from " << pathB << ": " << pcB.width << "x"
      << pcB.height << std::endl;

  ComputeAreaWeightsPc(pcA);
  ComputeAreaWeightsPc(pcB);

  Eigen::Vector3d muA, muB;
  Eigen::Matrix3d SA, SB;

  ComputeMoments(pcA, SA, muA);
  ComputeMoments(pcB, SB, muB);

  // Assume xB = R*xA + t
  Eigen::EigenSolver<Eigen::Matrix3d> eigA(SA);
  Eigen::EigenSolver<Eigen::Matrix3d> eigB(SB);
  Eigen::Matrix3d U = eigA.eigenvectors().real();
  Eigen::Matrix3d V = eigB.eigenvectors().real();

  Eigen::Matrix3d R = V*U.inverse();
  Eigen::Quaterniond q(R);
  Eigen::Vector3d t = muB - R*muA;

  std::cout << R << std::endl;
  std::cout << t.transpose() << std::endl;
  std::cout << R*SA*R.transpose() -SB << std::endl;

  if(pathOut.size() > 1) {
    std::ofstream out(pathOut + std::string(".csv"));
    out << "q_w q_x q_y q_z t_x t_y t_z" << std::endl; 
    out << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " " << t(0)
      << " " << t(1) << " " << t(2) << " " << std::endl;
    out.close();
  }

  if (vm.count("display")) {
    DisplayPcs(pcA, pcB, q, t, 1.0);
  }
}

