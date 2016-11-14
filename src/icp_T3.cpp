/* Copyright (c) 2015, Julian Straub <jstraub@csail.mit.edu> Licensed
 * under the MIT license. See the license file LICENSE.
 */

#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include "dpOptTrans/pc_helper.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, char** argv) {
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("in_a,a", po::value<std::string>(), "path to first input file")
    ("in_b,b", po::value<std::string>(), "path to second input file")
    ("out,o", po::value<std::string>(), "path to output file")
    ("cutoff,c", po::value<float>(), "cutoff for data association in ICP")
    ("transformation,t", po::value<std::string>(), "path to a transformation file with qw qx qy qz tx ty tz in the second line.")
//    ("transformation,t", po::value<std::string>(), "transformation guess")
//    ("rotation,q", po::value<std::string>(), "rotation guess in the form of a quaternion")
    ("normals,n", "use surface normals in ICP")
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
  std::string pathTransformation  = "";
  float cutoff = 0.3;
  //  std::string mode = "";
  //  if(vm.count("mode")) mode = vm["mode"].as<std::string>();
  if(vm.count("out")) pathOut = vm["out"].as<std::string>();
  if(vm.count("in_a")) pathA = vm["in_a"].as<std::string>();
  if(vm.count("in_b")) pathB = vm["in_b"].as<std::string>();
  if(vm.count("cutoff")) cutoff = vm["cutoff"].as<float>();
  if(vm.count("transformation")) pathTransformation = vm["transformation"].as<std::string>();

  // Load point clouds.
  pcl::PointCloud<pcl::PointXYZRGBNormal> pcA, pcB, pcB_T;
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

  Eigen::Matrix4f T_guess = Eigen::Matrix4f::Identity();
  bool have_guess = false;
  std::ifstream in(pathTransformation);
  if (pathTransformation.size() > 1 && in.is_open()) {
    std::string dummy;
    std::getline(in, dummy);
    Eigen::Vector4f q;
    Eigen::Vector3f t;
    in >> q.w() >> q.x() >> q.y() >> q.z() >> t(0) >> t(1) >> t(2);
    std::cout << "loaded intial transformation guess from file : " 
      << pathTransformation << std::endl;
    std::cout << "formatting: " << dummy << std::endl;
    std::cout << "q: " << q.transpose() << " t: " << t.transpose() << std::endl;
    T_guess.topLeftCorner<3,3>() = Eigen::Quaternionf(q).toRotationMatrix();
    T_guess.topRightCorner<3,1>() = t;
    have_guess = true;
  }

  ComputeAreaWeightsPc(pcA);
  ComputeAreaWeightsPc(pcB);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcA_ptr = pcA.makeShared();
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcB_ptr = pcB.makeShared();

  pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>* icp;
  if (vm.count("normals")) {
    icp = new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, 
        pcl::PointXYZRGBNormal>();
    std::cout << "Using points and surface normals in ICP" << std::endl;
  } else {
    icp = new pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, 
        pcl::PointXYZRGBNormal>();
    std::cout << "Using only points in ICP" << std::endl;
  }
  icp->setMaximumIterations(100);
  icp->setMaxCorrespondenceDistance (cutoff);
  icp->setTransformationEpsilon (1e-8);
  icp->setEuclideanFitnessEpsilon(0.01);
  icp->setInputSource(pcA_ptr);
  icp->setInputTarget(pcB_ptr);
  if (have_guess)
    icp->align(pcB_T, T_guess); 
  else
    icp->align(pcB_T); 

  bool success = icp->hasConverged();
  std::cout << "has converged:" << icp->hasConverged() << " score: " <<
    icp->getFitnessScore() << std::endl;
  std::cout << icp->getFinalTransformation() << std::endl;
  Eigen::Matrix4d T = icp->getFinalTransformation().cast<double>();
  Eigen::Matrix3d R = T.topLeftCorner<3,3>();
  Eigen::Quaterniond q(R);
  Eigen::Vector3d t = T.topRightCorner<3,1>();

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
  delete icp;

  return success ? 0 : 1;
}


