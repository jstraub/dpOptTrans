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

void DisplayPcs(const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcA, 
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcB, 
  const Eigen::Quaterniond& q_star, const Eigen::Vector3d& t_star,
  const Eigen::Vector3d& muA,
  const Eigen::Vector3d& muB,
  float scale) {

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcA_ptr = pcA.makeShared();
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcB_ptr = pcB.makeShared();
    // Construct transformed point cloud.
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcB_T_ptr(new
        pcl::PointCloud<pcl::PointXYZRGBNormal>(pcB));
    for (uint32_t i=0; i<pcB_T_ptr->size(); ++i) {
      Eigen::Map<Eigen::Vector3f> p(&(pcB_T_ptr->at(i).x));
      p = q_star.cast<float>().inverse()._transformVector( p - t_star.cast<float>());
      Eigen::Map<Eigen::Vector3f> n(pcB_T_ptr->at(i).normal);
      n = q_star.cast<float>().inverse()._transformVector(n);
      pcB_T_ptr->at(i).rgb = ((int)128) << 16 | ((int)255) << 8 | ((int)128);
    }
    for (uint32_t i=0; i<pcA_ptr->size(); ++i) {
      pcA_ptr->at(i).rgb = ((int)255) << 16 | ((int)128) << 8 | ((int)128);
    }
    // Construct surface normal point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr nA_ptr =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new
          pcl::PointCloud<pcl::PointXYZ>(pcA.size(),1));
    nA_ptr->getMatrixXfMap(3,4,0) = pcA.getMatrixXfMap(3,12,4);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nB_ptr =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new
          pcl::PointCloud<pcl::PointXYZ>(pcB.size(),1));
    nB_ptr->getMatrixXfMap(3,4,0) = pcB.getMatrixXfMap(3,12,4);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nB_T_ptr =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new
          pcl::PointCloud<pcl::PointXYZ>(*nB_ptr));
    for (uint32_t i=0; i<nA_ptr->size(); ++i) {
      nA_ptr->at(i).z -=1.1;
    }
    for (uint32_t i=0; i<nB_T_ptr->size(); ++i) {
      Eigen::Map<Eigen::Vector3f> n(&(nB_T_ptr->at(i).x));
      n = q_star.cast<float>().inverse()._transformVector(n);
      n(2) += 1.1;
    }

    // Otherwise the default is a identity rotation with scaling of
    // 0.5.
    pcA_ptr->sensor_orientation_.setIdentity();
    pcB_ptr->sensor_orientation_.setIdentity();
    pcB_T_ptr->sensor_orientation_.setIdentity();
    nA_ptr->sensor_orientation_.setIdentity();
    nB_ptr->sensor_orientation_.setIdentity();
    nB_T_ptr->sensor_orientation_.setIdentity();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerPc (new
        pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewerPc->initCameraParameters ();
    viewerPc->setBackgroundColor (1., 1., 1.);
//    viewerPc->addCoordinateSystem (scale);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>
      rgbA(pcA_ptr);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>
      rgbB(pcB_ptr);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>
      rgbB_T(pcB_T_ptr);
    viewerPc->addPointCloud<pcl::PointXYZRGBNormal> (pcA_ptr, rgbA, "cloudA");
//    viewerPc->addPointCloud<pcl::PointXYZRGBNormal> (pcB_ptr, rgbB, "cloudB",v1);
    viewerPc->addPointCloud<pcl::PointXYZRGBNormal> (pcB_T_ptr, rgbB_T,
        "cloudB transformed");

    char label[10];
    pcl::PointXYZ p;
    p.x = muA(0); p.y = muA(1); p.z = muA(2);
    sprintf(label,"SA%d",0);
    viewerPc->addSphere(p, scale, 1,0,0, label);
    p.x = muB(0); p.y = muB(1); p.z = muB(2);
    sprintf(label,"SB%d",0);
    viewerPc->addSphere(p, scale, 0.5,1,0.5, label);
    Eigen::Vector3d mu =
      q_star.inverse()._transformVector(muB -t_star);
    p.x = mu(0); p.y = mu(1); p.z = mu(2);
    sprintf(label,"S%d",0);
    viewerPc->addSphere(p, scale, 0,1,0, label);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerNc (new
        pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewerNc->initCameraParameters ();
    viewerNc->setBackgroundColor (0., 0., 0.);
    viewerNc->addCoordinateSystem (1.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
      nA_color(nA_ptr, 255, 128, 128);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
      nB_color(nB_ptr, 128, 255, 128);
    viewerNc->addPointCloud<pcl::PointXYZ> (nA_ptr, nA_color,  
        "normalsA");
//    viewerNc->addPointCloud<pcl::PointXYZRGBNormal> (pcB_ptr, rgbB, "cloudB",v1);
    viewerNc->addPointCloud<pcl::PointXYZ> (nB_T_ptr, nB_color, 
        "normals B transformed");

    while (!viewerPc->wasStopped () && !viewerNc->wasStopped()) {
      viewerPc->spinOnce (50);
      viewerNc->spinOnce (50);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
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
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigA(SA);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigB(SB);
  Eigen::Matrix3d U = eigA.eigenvectors();
  Eigen::Matrix3d V = eigB.eigenvectors();

  // Because R apears in a quadratic form we are free to negate it.
  // Since R \in R^3 this negates the determinant and makes it +1. -- a
  // rotation instead of a reflection.
  double min_cost = 9999999.;
  Eigen::Matrix3d R_star;
  Eigen::Vector3d t_star;

  for (uint32_t x=0; x < 3; x++)
    for (uint32_t y=0; y < 3; y++)
      for (uint32_t z=0; z < 3; z++)
        for (float i=-1.; i<=1.; i+=2.) 
          for (float j=-1.; j<=1.; j+=2.) 
            for (float k=-1.; k<=1.; k+=2.) {
              Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
              if (x == y || x == z || y == z)
                continue;
              P(0, x) = 1;
              P(1, y) = 1;
              P(2, z) = 1;
              Eigen::Matrix3d R_ = V*P*U.inverse();
              R_.col(0) *= i;
              R_.col(1) *= j;
              R_.col(2) *= k;
              if (R_.determinant() > 0.) {
                Eigen::Quaterniond q_(R_);
                Eigen::Vector3d t_ = muB - R_*muA;
                double c = ComputeClosestPointEucledianCost(pcA, pcB, &R_, &t_); 
                std::cout << "permutation " << P << "\t cost=" << c << std::endl;
                std::cout << q_.coeffs().transpose() 
                  << "\tt: " << t_.transpose() << std::endl;
                if(min_cost > c) {
                  min_cost = c;
                  R_star = R_;
                  t_star = t_;
                }
              }
            }
  Eigen::Matrix3d R = R_star;
  Eigen::Quaterniond q(R);
  Eigen::Vector3d t = muB - R.transpose()*muA;
  std::cout << "R:\n"<< R << std::endl;
  std::cout << "det(R) = " << R.determinant() << std::endl;
  std::cout << "t: " << t.transpose() << std::endl;
  std::cout << "norm of difference in covariance matrixes after matching: " 
    << (R*SA*R.transpose() -SB).norm() << std::endl;
  std::cout << "norm of the difference in the mean vectors after matching: " 
    << (muA - R.transpose()*(muB - t)).norm() << std::endl;

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
    double scale = sqrt(eigA.eigenvalues().real().maxCoeff())/5.;
    DisplayPcs(pcA, pcB, q, t, muA, muB, scale);
  }
}

