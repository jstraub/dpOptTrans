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

  Eigen::Matrix3d R = V*U.transpose();
  Eigen::Quaterniond q(R);
  Eigen::Vector3d t = muB - R*muA;
  std::cout << "R:\n"<< R << std::endl;
  std::cout << "t: " << t.transpose() << std::endl;
  std::cout << "norm of difference in covariance matrixes after matching: " 
    << (R*SA*R.transpose() -SB).norm() << std::endl;

  if (vm.count("display")) {
    std::cout << "Cov A" << std::endl << SA << std::endl;
    std::cout << "EV A" << std::endl << U << std::endl;
    std::cout << "E A" << std::endl << eigA.eigenvalues() << std::endl;
    std::cout << "mu A " << std::endl << muA << std::endl;
    std::cout << "Cov B" << std::endl << SB << std::endl;
    std::cout << "EV B" << std::endl << V << std::endl;
    std::cout << "E B" << std::endl << eigB.eigenvalues() << std::endl;
    std::cout << "mu B " << std::endl << muB << std::endl;
    std::cout << "Cov B Transformed" << std::endl
      << R.transpose()*SB*R << std::endl;
    std::cout << "mu B Transformed" << std::endl 
      << R.transpose()*(muB - t) << std::endl;
  }

  if (false) {
    Eigen::Matrix<double,3,6> M;
    M << 1,-1,0,0,0,0,
      0,0,1,-1,0,0,
      0,0,0,0,1,-1;
    uint32_t k =0;
    double min_cost = 9999.;
    Eigen::Matrix3d R_star;
    Eigen::Vector3d t_star;
    for(uint32_t i=0; i<6; ++i)
      for(uint32_t j=0; j<6; ++j) {
        Eigen::Matrix3d Rperm;
        Rperm.col(0) = M.col(i);
        Rperm.col(1) = M.col(j);
        Rperm.col(2) = M.col(i).cross(M.col(j));
        if (fabs(Rperm.determinant() -1.) < 1e-6) {
          //        std::cout << Rperm << std::endl;
          Eigen::Matrix3d R_ = Rperm*R;
          Eigen::Quaterniond q_(R_);
          Eigen::Vector3d t_ = muB - R_*muA;
          double c = ComputeClosestPointEucledianCost(pcA, pcB, &R_, &t_); 
          std::cout << "permutation k=" << k << "\t cost=" << c << std::endl;
          std::cout << q_.coeffs().transpose() 
            << "\tt: " << t_.transpose() << std::endl;
          ++k; 
          if(min_cost > c) {
            min_cost = c;
            R_star = R_;
            t_star = t_;
          }
        }
      }
    q = Eigen::Quaterniond(R_star);
    t = t_star;
  }
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

