/* Copyright (c) 2015, Julian Straub <jstraub@csail.mit.edu> Licensed
 * under the MIT license. See the license file LICENSE.
 */

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/transforms.h>

#include <cuda_runtime.h>
#include <nvidia/helper_cuda.h> 
#include <jsCore/global.hpp>

//#include "rtDDPvMF/rtDDPvMF.hpp"
//#include "rtDDPvMF/realtimeDDPvMF_openni.hpp"
#include "bbTrans/lower_bound_R3.h"
#include "bbTrans/upper_bound_indep_R3.h"
#include "bbTrans/upper_bound_convex_R3.h"
#include "bbTrans/lower_bound_S3.h"
#include "bbTrans/upper_bound_indep_S3.h"
#include "bbTrans/upper_bound_convex_S3.h"
#include "bbTrans/lower_bound_Lin.h"
#include "bbTrans/upper_bound_Lin.h"
#include "bbTrans/node_TpS3.h"
#include "bbTrans/node_AA.h"
#include "bbTrans/branch_and_bound.h"
#include "bbTrans/vmf.h"
#include "bbTrans/vmf_mm.h"
#include "bbTrans/normal.h"
#include "dpOptTrans/dp_vmf_opt_rot.h"
#include "dpOptTrans/pc_helper.h"
#include "dpOptTrans/pcHelpers.h"
#include "dpMMlowVar/ddpmeansCUDA.hpp"
#include "dpMMlowVar/dpmeans.hpp"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

//struct CfgRtDDPvMF
//{
//
//  CfgRtDDPvMF() : f_d(540.0), lambda(-1.), beta(1e5), Q(-2.),
//      nSkipFramesSave(30), nFramesSurvive_(0),lambdaDeg_(90.)
//  {};
//  CfgRtDDPvMF(const CfgRtDDPvMF& cfg)
//    : f_d(cfg.f_d), lambda(cfg.lambda), beta(cfg.beta), Q(cfg.Q),
//      nSkipFramesSave(cfg.nSkipFramesSave), nFramesSurvive_(cfg.nFramesSurvive_),
//      lambdaDeg_(cfg.lambdaDeg_)
//  {
//    pathOut = cfg.pathOut;
//  };
//
//  double f_d;
//  double lambda;
//  double beta;
//  double Q;
//
//  int32_t nSkipFramesSave;
//  std::string pathOut;
//
//  int32_t nFramesSurvive_;
//  double lambdaDeg_;
//
//  void lambdaFromDeg(double lambdaDeg)
//  {
//    lambdaDeg_ = lambdaDeg;
//    lambda = cos(lambdaDeg*M_PI/180.0)-1.;
//  };
//  void QfromFrames2Survive(int32_t nFramesSurvive)
//  {
//    nFramesSurvive_ = nFramesSurvive;
//    Q = nFramesSurvive == 0? -2. : lambda/double(nFramesSurvive);
//  };
//};

Eigen::Vector3d ComputePcMean(pcl::PointCloud<pcl::PointXYZRGBNormal>&
    pc) {
  // take 3 values (x,y,z of normal) with an offset of 4 values (x,y,z
  // and one float which is undefined) and the step is 12 (4 for xyz, 4
  // for normal xyz and 4 for curvature and rgb).
  auto xyz = pc.getMatrixXfMap(3, 12, 0); // this works for PointXYZRGBNormal
  Eigen::Vector3d mean =  Eigen::Vector3d::Zero();

  for (size_t i=0; i<xyz.cols(); ++i) {
    mean += xyz.col(i).cast<double>();
  }
  return  mean /= xyz.cols();
}

bool ComputevMFMMfromPC(const pcl::PointCloud<pcl::PointXYZRGBNormal>&
    pc, const CfgRtDDPvMF& cfg, std::vector<vMF<3>>& vmfs) {
  // take 3 values (x,y,z of normal) with an offset of 4 values (x,y,z
  // and one float which is undefined) and the step is 12 (4 for xyz, 4
  // for normal xyz and 4 for curvature and rgb).
  auto n_map = pc.getMatrixXfMap(3, 12, 4); // this works for PointXYZRGBNormal
  boost::shared_ptr<Eigen::MatrixXf> n(new Eigen::MatrixXf(n_map));
//  std::cout << "normals: " << std::endl << n->rows() 
//    << "x" << n->cols() << std::endl;
  // Setup the DPvMF clusterer.
  shared_ptr<jsc::ClDataGpuf> cld(new jsc::ClDataGpuf(n,0));
  dplv::DDPMeansCUDA<float,dplv::Spherical<float>> pddpvmf(cld,
      cfg.lambda, cfg.Q, cfg.beta);
  // Run the clustering algorithm.
  pddpvmf.nextTimeStep(n);
  for(uint32_t i=0; i<10; ++i)
  {
//    cout<<"@"<<i<<" :"<<endl;
    pddpvmf.updateLabels();
    pddpvmf.updateCenters();
    if(pddpvmf.convergedCounts(n->cols()/100)) break;
  }
  pddpvmf.getZfromGpu(); // cache z_ back from gpu
  const VectorXu& z = pddpvmf.z();
  // Output results.
  MatrixXf centroids = pddpvmf.centroids();
  VectorXf counts = pddpvmf.counts().cast<float>();
//  std::cout << counts.transpose() << std::endl;
//  std::cout << centroids << std::endl;
  uint32_t K = counts.rows();
//  std::cout << "K: " << K << std::endl;
  // Compute vMF statistics: area-weighted sum over surface normals
  // associated with respective cluster. 
  MatrixXd xSum = MatrixXd::Zero(3,K);
  VectorXf ws = Eigen::VectorXf::Zero(K);
  for (uint32_t i=0; i<n->cols(); ++i) 
    if(z(i) < K) {
      float w = pc.at(i).curvature; // curvature is used to store the weights.
      xSum.col(z(i)) += n->col(i).cast<double>()*w;
      ws(z(i)) += w;
    }
  // Fractions belonging to each cluster.
  Eigen::VectorXd pis = (ws.array() / ws.sum()).matrix().cast<double>();
  Eigen::VectorXf taus(K);
  for(uint32_t k=0; k<K; ++k)
    if (counts(k) > 5) {
      taus(k) = bb::vMF<3>::MLEstimateTau(xSum.col(k),
          xSum.col(k).cast<double>()/xSum.col(k).norm(), ws(k));
    }
  
  for(uint32_t k=0; k<K; ++k)
    if (counts(k) > 5) {
      vmfs.push_back(bb::vMF<3>(xSum.col(k).cast<double>()/xSum.col(k).norm(),
            taus(k), pis(k)));
    }
  return true;
}

bool ComputeGMMfromPC(pcl::PointCloud<pcl::PointXYZRGBNormal>&
    pc, const CfgRtDDPvMF& cfg, std::vector<Normal<3>>& gmm,
    bool colorLables) {
  // take 3 values (x,y,z of normal) with an offset of 4 values (x,y,z
  // and one float which is undefined) and the step is 12 (4 for xyz, 4
  // for normal xyz and 4 for curvature and rgb).
  auto xyz_map = pc.getMatrixXfMap(3, 12, 0); // this works for PointXYZRGBNormal
  boost::shared_ptr<Eigen::MatrixXf> xyz(new Eigen::MatrixXf(xyz_map));
//  std::cout << "xyz: " << std::endl << xyz->rows() 
//    << "x" << xyz->cols() << std::endl;

  // Setup the DPvMF clusterer.
  shared_ptr<jsc::ClDataGpuf> cld(new jsc::ClDataGpuf(xyz,0));
  dplv::DDPMeansCUDA<float,dplv::Euclidean<float> > dpmeans(cld,
      cfg.lambda, cfg.Q, cfg.beta);
  // Run the clustering algorithm.
  dpmeans.nextTimeStep(xyz);
  for(uint32_t i=0; i<10; ++i)
  {
//    cout<<"@"<<i<<" :"<<endl;
    dpmeans.updateLabels();
    dpmeans.updateCenters();
    if(dpmeans.convergedCounts(xyz->cols()/100)) break;
  }
  dpmeans.getZfromGpu(); // cache z_ back from gpu
  const VectorXu& z = dpmeans.z();
  // Output results.
  MatrixXf centroids = dpmeans.centroids();
  VectorXf counts = dpmeans.counts().cast<float>();
//  std::cout << counts.transpose() << std::endl;
//  std::cout << centroids << std::endl;
  uint32_t K = counts.rows();
  std::cout << "lambda=" << cfg.lambda << " K=" << K << std::endl;
  // Compute Gaussian statistics: 
  std::vector<MatrixXd> Ss(K,Matrix3d::Zero());
  Eigen::MatrixXd xSum = Eigen::MatrixXd::Zero(3,K);
  Eigen::VectorXf ws = Eigen::VectorXf::Zero(K);
  for (uint32_t i=0; i<xyz->cols(); ++i) 
    if(z(i) < K) {
      float w = pc.at(i).curvature; // curvature is used to store the weights.
      ws(z(i)) += w;
      xSum.col(z(i)) += xyz->col(i).cast<double>()*w;
    }
  for(uint32_t k=0; k<K; ++k)
    centroids.col(k) = xSum.col(k).cast<float>()/ws(k);

  for (uint32_t i=0; i<xyz->cols(); ++i) 
    if(z(i) < K) {
      float w = pc.at(i).curvature; // curvature is used to store the weights.
      Ss[z(i)] += w*(xyz->col(i) - centroids.col(z(i))).cast<double>()
        * (xyz->col(i) - centroids.col(z(i))).cast<double>().transpose();
    }
  if (colorLables) {
    for (uint32_t i=0; i<xyz->cols(); ++i) 
      if(z(i) < K) {
        uint8_t r = floor(float(z(i))/float(K-1)*255.);
        uint8_t g = floor(float(K-1-z(i))/float(K-1)*255.);
        uint8_t b = floor(float(z(i)<=(K-1)/2? z(i)+(K-1)/2 :
              z(i)-(K-1)/2-1)/float(K-1)*255.);
        pc.at(i).rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
      }
  }
  // Fractions belonging to each cluster.
  Eigen::VectorXd pis = (ws.array() / ws.sum()).matrix().cast<double>();
  
  const double maxEvFactor = 1e-2;
  for(uint32_t k=0; k<K; ++k)
//    if (pis(k) > 0.) {
    if (counts(k) > 5) {
      Matrix3d cov = Ss[k]/float(ws(k));
      Eigen::SelfAdjointEigenSolver<Matrix3d> eig(cov);
      Eigen::Vector3d e = eig.eigenvalues();
      uint32_t iMax = 0;
      double eMax = e.maxCoeff(&iMax);
//      std::cout << cov << std::endl;
      bool regularized = false;
      for (uint32_t i=0; i<3; ++i)
        if (i!=iMax && eMax*maxEvFactor > e(i)) {
          std::cout << "small eigenvalue: " << e(i) << " replaced by " 
            << eMax*maxEvFactor << std::endl;
          e(i) = eMax*maxEvFactor;
          regularized = true;
        }
      if (regularized) {
        Eigen::Matrix3d V = eig.eigenvectors();
        cov = V*e.asDiagonal()*V.inverse();
      }
//      std::cout << cov << std::endl;
      gmm.push_back(bb::Normal<3>(centroids.col(k).cast<double>(),
            cov, pis(k)));
//        Ss[k]/float(ws(k)), pis(k)));
    }
  return true;
}

void DisplayPcs(const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcA, 
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcB, 
  const std::vector<bb::Normal<3>>& gmmA, 
  const std::vector<bb::Normal<3>>& gmmB, 
  const std::vector<bb::vMF<3>>& vmfsA,
  const std::vector<bb::vMF<3>>& vmfsB,
  const Eigen::Quaterniond& q_star, const Eigen::Vector3d& t_star,
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
    for(uint32_t k=0; k<gmmA.size(); ++k) {
      pcl::PointXYZ p;
      p.x = gmmA[k].GetMu()(0); p.y = gmmA[k].GetMu()(1); p.z =
        gmmA[k].GetMu()(2);
      sprintf(label,"SA%d",k);
      viewerPc->addSphere(p, scale, 1,0,0, label);
    }
    for(uint32_t k=0; k<gmmB.size(); ++k) {
      Eigen::Vector3d mu =
        q_star.inverse()._transformVector(gmmB[k].GetMu() -t_star);
      pcl::PointXYZ p;
      p.x = mu(0); p.y = mu(1); p.z = mu(2);
      sprintf(label,"SB%d",k);
      viewerPc->addSphere(p, scale,0,1,0, label);
    }

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

    for(uint32_t k=0; k<vmfsA.size(); ++k) {
      pcl::PointXYZ p;
      p.x = vmfsA[k].GetMu()(0); p.y = vmfsA[k].GetMu()(1); p.z =
        vmfsA[k].GetMu()(2) -1.1;
      sprintf(label,"SvA%d",k);
      viewerNc->addSphere(p, 0.05, 1,0,0, label);
    }
    for(uint32_t k=0; k<vmfsB.size(); ++k) {
      Eigen::Vector3d mu =
        q_star.inverse()._transformVector(vmfsB[k].GetMu());
      pcl::PointXYZ p;
      p.x = mu(0); p.y = mu(1); p.z = mu(2)+1.1;
      sprintf(label,"SvB%d",k);
      viewerNc->addSphere(p, 0.05,0,1,0, label);
    }

    while (!viewerPc->wasStopped () && !viewerNc->wasStopped()) {
      viewerPc->spinOnce (50);
      viewerNc->spinOnce (50);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
//
//bool ComputevMFMMfromPC(const pcl::PointCloud<pcl::PointXYZRGBNormal>&
//    pc, const CfgRtDDPvMF& cfg, std::vector<bb::vMF<3>>& vmfs) {
//  // take 3 values (x,y,z of normal) with an offset of 4 values (x,y,z
//  // and one float which is undefined) and the step is 12 (4 for xyz, 4
//  // for normal xyz and 4 for curvature and rgb).
//  auto n_map = pc.getMatrixXfMap(3, 12, 4); // this works for PointXYZRGBNormal
//  boost::shared_ptr<Eigen::MatrixXf> n(new Eigen::MatrixXf(n_map));
////  std::cout << "normals: " << std::endl << n->rows() 
////    << "x" << n->cols() << std::endl;
//  // Setup the DPvMF clusterer.
//  shared_ptr<jsc::ClDataGpuf> cld(new jsc::ClDataGpuf(n,0));
//  dplv::DDPMeansCUDA<float,dplv::Spherical<float> > pddpvmf(cld,
//      cfg.lambda, cfg.Q, cfg.beta);
//  // Run the clustering algorithm.
//  pddpvmf.nextTimeStep(n);
//  for(uint32_t i=0; i<10; ++i)
//  {
////    cout<<"@"<<i<<" :"<<endl;
//    pddpvmf.updateLabels();
//    pddpvmf.updateCenters();
//    if(pddpvmf.convergedCounts(n->cols()/100)) break;
//  }
//  pddpvmf.getZfromGpu(); // cache z_ back from gpu
//  const VectorXu& z = pddpvmf.z();
//  // Output results.
//  MatrixXf centroids = pddpvmf.centroids();
//  VectorXf counts = pddpvmf.counts().cast<float>();
////  std::cout << counts.transpose() << std::endl;
////  std::cout << centroids << std::endl;
//  uint32_t K = counts.rows();
////  std::cout << "K: " << K << std::endl;
//  // Compute vMF statistics: area-weighted sum over surface normals
//  // associated with respective cluster. 
//  MatrixXd xSum = MatrixXd::Zero(3,K);
//  VectorXf ws = Eigen::VectorXf::Zero(K);
//  for (uint32_t i=0; i<n->cols(); ++i) 
//    if(z(i) < K) {
//      float w = pc.at(i).curvature; // curvature is used to store the weights.
//      xSum.col(z(i)) += n->col(i).cast<double>()*w;
//      ws(z(i)) += w;
//    }
//  // Fractions belonging to each cluster.
//  Eigen::VectorXd pis = (ws.array() / ws.sum()).matrix().cast<double>();
//  Eigen::VectorXf taus(K);
//  for(uint32_t k=0; k<K; ++k)
//    if (counts(k) > 100) {
//      taus(k) = bb::vMF<3>::MLEstimateTau(xSum.col(k),
//          xSum.col(k).cast<double>()/xSum.col(k).norm(), ws(k));
//    }
//  
//  for(uint32_t k=0; k<K; ++k)
//    if (counts(k) > 100) {
//      vmfs.push_back(bb::vMF<3>(xSum.col(k).cast<double>()/xSum.col(k).norm(),
//            taus(k), pis(k)));
//    }
//  return true;
//}
//
//bool ComputeGMMfromPC(pcl::PointCloud<pcl::PointXYZRGBNormal>&
//    pc, const CfgRtDDPvMF& cfg, std::vector<bb::Normal<3>>& gmm,
//    bool colorLables) {
//  // take 3 values (x,y,z of normal) with an offset of 4 values (x,y,z
//  // and one float which is undefined) and the step is 12 (4 for xyz, 4
//  // for normal xyz and 4 for curvature and rgb).
//  auto xyz_map = pc.getMatrixXfMap(3, 12, 0); // this works for PointXYZRGBNormal
//  boost::shared_ptr<Eigen::MatrixXf> xyz(new Eigen::MatrixXf(xyz_map));
////  std::cout << "xyz: " << std::endl << xyz->rows() 
////    << "x" << xyz->cols() << std::endl;
//
//  // Setup the DPvMF clusterer.
//  shared_ptr<jsc::ClDataGpuf> cld(new jsc::ClDataGpuf(xyz,0));
//  dplv::DDPMeansCUDA<float,dplv::Euclidean<float> > dpmeans(cld,
//      cfg.lambda, cfg.Q, cfg.beta);
//  // Run the clustering algorithm.
//  dpmeans.nextTimeStep(xyz);
//  for(uint32_t i=0; i<10; ++i)
//  {
////    cout<<"@"<<i<<" :"<<endl;
//    dpmeans.updateLabels();
//    dpmeans.updateCenters();
//    if(dpmeans.convergedCounts(xyz->cols()/100)) break;
//  }
//  dpmeans.getZfromGpu(); // cache z_ back from gpu
//  const VectorXu& z = dpmeans.z();
//  // Output results.
//  MatrixXf centroids = dpmeans.centroids();
//  VectorXf counts = dpmeans.counts().cast<float>();
////  std::cout << counts.transpose() << std::endl;
////  std::cout << centroids << std::endl;
//  uint32_t K = counts.rows();
////  std::cout << "K: " << K << std::endl;
//  // Compute Gaussian statistics: 
//  std::vector<MatrixXd> Ss(K,Matrix3d::Zero());
//  Eigen::MatrixXd xSum = Eigen::MatrixXd::Zero(3,K);
//  Eigen::VectorXf ws = Eigen::VectorXf::Zero(K);
//  for (uint32_t i=0; i<xyz->cols(); ++i) 
//    if(z(i) < K) {
//      float w = pc.at(i).curvature; // curvature is used to store the weights.
//      ws(z(i)) += w;
//      xSum.col(z(i)) += xyz->col(i).cast<double>()*w;
//    }
//  for(uint32_t k=0; k<K; ++k)
//    centroids.col(k) = xSum.col(k).cast<float>()/ws(k);
//
//  for (uint32_t i=0; i<xyz->cols(); ++i) 
//    if(z(i) < K) {
//      float w = pc.at(i).curvature; // curvature is used to store the weights.
//      Ss[z(i)] += w*(xyz->col(i) - centroids.col(z(i))).cast<double>()
//        * (xyz->col(i) - centroids.col(z(i))).cast<double>().transpose();
//    }
//  if (colorLables) {
//    for (uint32_t i=0; i<xyz->cols(); ++i) 
//      if(z(i) < K) {
//        uint8_t r = floor(float(z(i))/float(K-1)*255.);
//        uint8_t g = floor(float(K-1-z(i))/float(K-1)*255.);
//        uint8_t b = floor(float(z(i)<=(K-1)/2? z(i)+(K-1)/2 :
//              z(i)-(K-1)/2-1)/float(K-1)*255.);
//        pc.at(i).rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
//      }
//  }
//  // Fractions belonging to each cluster.
//  Eigen::VectorXd pis = (ws.array() / ws.sum()).matrix().cast<double>();
//  
//  for(uint32_t k=0; k<K; ++k)
////    if (pis(k) > 0.) {
//    if (counts(k) > 100) {
//      gmm.push_back(bb::Normal<3>(centroids.col(k).cast<double>(),
//            Ss[k]/float(ws(k))+0.001*Eigen::Matrix3d::Identity(), pis(k)));
////        Ss[k]/float(ws(k)), pis(k)));
//    }
//  return true;
//}

bool ComputePcBoundaries(const pcl::PointCloud<pcl::PointXYZRGBNormal>&
    pc, Eigen::Vector3d& min, Eigen::Vector3d& max) {
  auto x = pc.getMatrixXfMap(1, 12, 0); // this works for PointXYZRGBNormal
  auto y = pc.getMatrixXfMap(1, 12, 1); // this works for PointXYZRGBNormal
  auto z = pc.getMatrixXfMap(1, 12, 2); // this works for PointXYZRGBNormal
  min << x.minCoeff(), y.minCoeff(), z.minCoeff();
  max << x.maxCoeff(), y.maxCoeff(), z.maxCoeff();
  return true;
}

int main(int argc, char** argv) {
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("lambdaDeg,l", po::value<double>(), "lambda in degree for DPvMF-means")
    ("lambdaT,t", po::value<double>(), "lambda in meters for DPMeans")
    ("in_a,a", po::value<string>(), "path to first input file")
    ("in_b,b", po::value<string>(), "path to second input file")
    ("out,o", po::value<string>(), "path to output file")
    ("scale,s", po::value<float>(),"scale for point-cloud")
    ("egi,e", "make the vMF MM pis uniform - like a EGI")
    ("simpleTrans", "use means of PCs to compute translation")
    ("oB0", "output bounds at level 0")
    ("TpS", "use TpS-based tessellation instead of 600-cell based (not recommended)")
    ("AA", "use axis-angle-based tessellation instead of 600-cell based (not recommended)")
    ("display,d", "display results")
    ("verbose,v", "be verbose")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  CfgRtDDPvMF cfg;
  cfg.f_d = 540.;
  cfg.beta = 1e5;
  cfg.nFramesSurvive_ = 1; // DPvMFMM
  cfg.pathOut = std::string("../results/");
  double lambdaDeg = 93.;
  if(vm.count("lambdaDeg")) lambdaDeg = vm["lambdaDeg"].as<double>();
  double lambdaT = 1.0;
  if(vm.count("lambdaT")) lambdaT = vm["lambdaT"].as<double>();

  cfg.lambdaFromDeg(lambdaDeg);
  cfg.QfromFrames2Survive(cfg.nFramesSurvive_);

  bool output_init_bounds = false;
  bool egi_mode = false;
  bool TpS_mode = false;
  bool AA_mode = false;
  bool simpleTrans = false;
  string pathA = "";
  string pathB = "";
  std::string pathOut = "";
  //  string mode = "";
  //  if(vm.count("mode")) mode = vm["mode"].as<string>();
  if(vm.count("out")) pathOut = vm["out"].as<std::string>();
  if(vm.count("in_a")) pathA = vm["in_a"].as<string>();
  if(vm.count("in_b")) pathB = vm["in_b"].as<string>();
  float scale = 1.;
  if(vm.count("scale")) scale = vm["scale"].as<float>();
  if(vm.count("egi")) egi_mode = true;
  if(vm.count("TpS")) TpS_mode = true;
  if(vm.count("AA")) AA_mode = true;
  if(vm.count("oB0")) output_init_bounds = true;
  if(vm.count("simpleTrans")) simpleTrans = true;
  if (egi_mode)
    std::cout << "Using EGI mode - making pis of vMF MM uniform." << std::endl;

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

  ShufflePc(pcA);
  ShufflePc(pcB);

  ComputeAreaWeightsPc(pcA);
  ComputeAreaWeightsPc(pcB);

  // Obtain the range for the translation.
  // TODO: this could be made more tight by getting min and max for the
  // rotated point-clouds.
  Eigen::Vector3d minA, minB, maxA, maxB, min, max;
  ComputePcBoundaries(pcA, minA, maxA);
  ComputePcBoundaries(pcB, minB, maxB);

  findCudaDevice(argc,(const char**)argv);

  std::cout<<"rtDDPvMFmeans lambdaDeg="<<cfg.lambdaDeg_<<" beta="<<cfg.beta
    <<"nFramesSurvive="<<cfg.nFramesSurvive_<<std::endl;
  std::cout<<"output path: "<<cfg.pathOut<<std::endl;

  std::vector<bb::vMF<3>> vmfsA;
  ComputevMFMMfromPC(pcA, cfg, vmfsA);
  if(vm.count("verbose")) for(uint32_t k=0; k<vmfsA.size(); ++k) vmfsA[k].Print(); 

  std::vector<bb::vMF<3>> vmfsB;
  ComputevMFMMfromPC(pcB, cfg, vmfsB);
  if(vm.count("verbose")) for(uint32_t k=0; k<vmfsB.size(); ++k) vmfsB[k].Print(); 

  if (egi_mode) {
    for(uint32_t k=0; k<vmfsA.size(); ++k) 
      vmfsA[k].SetPi(1./float(vmfsA.size()));
    for(uint32_t k=0; k<vmfsB.size(); ++k) 
      vmfsB[k].SetPi(1./float(vmfsB.size()));
  }

  cfg.lambda = lambdaT;
  std::vector<bb::Normal<3>> gmmA;
  if (!simpleTrans) {
    ComputeGMMfromPC(pcA, cfg, gmmA, false);
    if(vm.count("verbose")) for(uint32_t k=0; k<gmmA.size(); ++k) gmmA[k].Print(); 
  }

  std::vector<bb::Normal<3>> gmmB;
  if (!simpleTrans) {
    ComputeGMMfromPC(pcB, cfg, gmmB, false);
    if(vm.count("verbose")) for(uint32_t k=0; k<gmmB.size(); ++k) gmmB[k].Print(); 
  }

  bb::vMFMM<3> vmfmmA(vmfsA);
  bb::vMFMM<3> vmfmmB(vmfsB);
  bb::LowerBoundS3 lower_bound_S3(vmfmmA, vmfmmB);
  bb::UpperBoundIndepS3 upper_bound_S3(vmfmmA, vmfmmB);
  bb::UpperBoundConvexS3 upper_bound_convex_S3(vmfmmA, vmfmmB);

  std::list<bb::NodeS3> nodesS3;
  Eigen::Quaterniond q_star;
  double lb_star = 1e99;
  double eps = 1e-8;
  //  double eps = 8e-7;
  uint32_t max_lvl = 20;
  uint32_t max_it = 5000;
  if (TpS_mode)  {

    std::cout << " Tessellate TpS3" << std::endl;
    std::list<bb::NodeTpS3> nodes = bb::TessellateTpS3();
    std::cout << "# initial nodes: " << nodes.size() << std::endl;

    bb::LowerBoundTpS3 lower_bound_TpS3(lower_bound_S3);
    bb::UpperBoundIndepTpS3 upper_bound_TpS3(upper_bound_S3);
    bb::UpperBoundConvexTpS3 upper_bound_convex_TpS3(upper_bound_convex_S3);
    if (output_init_bounds) {
      WriteBounds<bb::NodeTpS3>(lower_bound_TpS3, upper_bound_TpS3,
          upper_bound_convex_TpS3, nodes);
    }
    std::cout << " BB on S3 eps=" << eps << " max_it=" << max_it << std::endl;
    bb::BranchAndBound<bb::NodeTpS3> bb(lower_bound_TpS3, upper_bound_convex_TpS3);
    bb::NodeTpS3 node_star = bb.Compute(nodes, eps, max_lvl, max_it);
//  bb::CountBranchesInTree<bb::NodeS3>(nodes);
    q_star = node_star.GetLbArgument();
    lb_star = node_star.GetLB();
    // output a list of NodeS3s for further processing; use the
    // interior Node for this.
    for (const auto& node : nodes)
      nodesS3.push_back(node.GetNodeS3());
  } else if (AA_mode)  {
    std::cout << " Tessellate axis angle space" << std::endl;
    std::list<bb::NodeAA> nodes = bb::TessellateAA();
    std::cout << "# initial nodes: " << nodes.size() << std::endl;

    bb::LowerBoundAA lower_bound_AA(lower_bound_S3);
    bb::UpperBoundIndepAA upper_bound_AA(upper_bound_S3);
    bb::UpperBoundConvexAA upper_bound_convex_AA(upper_bound_convex_S3);
    if (output_init_bounds) {
      WriteBounds<bb::NodeAA>(lower_bound_AA, upper_bound_AA,
          upper_bound_convex_AA, nodes);
    }
    std::cout << " BB on S3 eps=" << eps << " max_it=" << max_it << std::endl;
    bb::BranchAndBound<bb::NodeAA> bb(lower_bound_AA, upper_bound_convex_AA);
    bb::NodeAA node_star = bb.Compute(nodes, eps, max_lvl, max_it);
//  bb::CountBranchesInTree<bb::NodeS3>(nodes);
    q_star = node_star.GetLbArgument();
    lb_star = node_star.GetLB();
    // output a list of NodeS3s for further processing; use the
    // interior Node for this.
    for (const auto& node : nodes)
      nodesS3.push_back(node.GetNodeS3());
  } else {
    std::cout << " Tessellate S3" << std::endl;
    nodesS3 = bb::GenerateNotesThatTessellateS3();
    std::cout << "# initial nodes: " << nodesS3.size() << std::endl;
    if (output_init_bounds) {
      WriteBounds<bb::NodeS3>(lower_bound_S3, upper_bound_S3,
          upper_bound_convex_S3, nodesS3);
    }
    std::cout << " BB on S3 eps=" << eps << " max_it=" << max_it << std::endl;
    bb::BranchAndBound<bb::NodeS3> bb(lower_bound_S3, upper_bound_convex_S3);
    bb::NodeS3 node_star = bb.Compute(nodesS3, eps, max_lvl, max_it);
//  bb::CountBranchesInTree<bb::NodeS3>(nodesS3);
    q_star = node_star.GetLbArgument();
    lb_star = node_star.GetLB();
  }
  std::cout << "optimum BB quaternion: "  << q_star.coeffs().transpose()
      << " angle: " << 2.*acos(q_star.w()) * 180. / M_PI << std::endl
      << q_star.toRotationMatrix()
      << std::endl;
  Eigen::Vector3d t_star; 

  shared_ptr<Eigen::MatrixXd> qAll(new MatrixXd(4, nodesS3.size()));
  auto it=nodesS3.begin();
  for (uint32_t i=0; i < nodesS3.size(); ++i, it++) {
    qAll->col(i) = it->GetTetrahedron().GetCenter();
  }
  double lambda_q = cos(2.*10.*M_PI/180.) - 1.; // TODO
  dplv::DPMeans<double, dplv::Spherical<double>> dpvMF(qAll, 0, lambda_q);
  for (uint32_t t=0; t<20; ++t) {
    dpvMF.updateLabels(); 
    dpvMF.updateCenters(); 
  }
  std::cout << dpvMF.centroids() << std::endl; 

  std::vector<Eigen::Quaterniond> qsPrelim;
  std::vector<double> lbsS3prelim;
  if (dpvMF.K() == 1) {
    qsPrelim.push_back(q_star);
    lbsS3prelim.push_back(lb_star);
  } else {
    std::cout << "======== K > 1: " << dpvMF.K() << std::endl;
    for (uint32_t k=0; k<dpvMF.K(); ++k) {
      auto it = nodesS3.begin();
      auto z = dpvMF.z();
      // Find the node whith maximum LB in each cluster.
      double lb_max = -1.e20;
      Eigen::Quaterniond q_k; 
      for (uint32_t i=0; i<nodesS3.size(); ++i, it++) 
        if (z(i) == k && lb_max < it->GetLB()) {
          lb_max = it->GetLB();
          q_k = it->GetTetrahedron().GetCenterQuaternion();
        }
      qsPrelim.push_back(q_k);
      lbsS3prelim.push_back(lb_max);
      //  Eigen::Quaterniond q = node_star.GetTetrahedron().GetCenterQuaternion();
      std::cout << "in cluster " << k << ": center = "  
        << dpvMF.centroids().col(k).transpose() << std::endl;
      std::cout << " max LB quaternion: "  << q_k.coeffs().transpose()
        << " angle: " << 2.*acos(q_k.w()) * 180. / M_PI << std::endl;
    }
  }

  // TODO could be foldet into above.
  std::vector<bool> toDelete(dpvMF.K(), false);
  uint32_t n_delete = 0;
  for (uint32_t k=0; k<dpvMF.K(); ++k) 
    for (uint32_t j=k+1; j<dpvMF.K(); ++j) { 
      double dAng = qsPrelim[k].angularDistance(qsPrelim[j]);
      std::cout << " dang " << k << ";" << j << ": " << dAng*180./M_PI << std::endl;
      toDelete[k] = dAng < 0.1/180.*M_PI;
      if (toDelete[k]) ++n_delete;
    }
  Eigen::VectorXd lbsS3(qsPrelim.size()-n_delete);
  std::vector<Eigen::Quaterniond> qs;
  uint32_t j=0;
  for (uint32_t k=0; k<dpvMF.K(); ++k) 
    if (!toDelete[k]) {
      qs.push_back(qsPrelim[k]);
      lbsS3(j++) = lbsS3prelim[k];
    }
  Eigen::VectorXd lbsR3(qs.size());
  Eigen::VectorXd lbs(qs.size());

  std::cout << "===filtered==== K: " << qs.size() << std::endl;
  std::vector<Eigen::Vector3d> ts;
  for (uint32_t k=0; k<qs.size(); ++k) {
    // This q is the inverse of the rotation that brings B to A.
    qs[k] = qs[k].inverse(); // A little ugly but this is because of the way we setup the problem...
  }
  for (uint32_t k=0; k<qs.size(); ++k) {
    Eigen::Quaterniond q = qs[k]; 
    if (simpleTrans) {
      std::cout << "Simple transformation " << std::endl;
      Eigen::Vector3d meanA = ComputePcMean(pcA);
      Eigen::Vector3d meanB = ComputePcMean(pcB);
      std::cout << meanA.transpose() << std::endl << meanB.transpose() << std::endl;
      Eigen::Vector3d t = meanB - q._transformVector(meanA);
      std::cout << t.transpose() << std::endl;
//      std::cout << q._transformVector(meanB) - meanA << std::endl;
//      std::cout << q._transformVector(meanA) - meanB << std::endl;
      
      ts.push_back(t);
      lbsR3(k) = 0.;
      lbs(k) = lbsS3(k);
    } else {
      // To get all corners of the bounding box.j
      bb::Box box(minA, maxA);
      // Update the boundaries of the the rotated point cloud A to get
      // the full translation space later on.
      for (uint32_t i=0; i<8; ++i) {
        Eigen::Vector3d c;
        box.GetCorner(i,c);
        c = q.inverse()._transformVector(c);
        for (uint32_t d=0; d<3; ++d) {
          minA(d) = std::min(minA(d), c(d));
          maxA(d) = std::max(maxA(d), c(d));
        }
      }
      for (uint32_t d=0; d<3; ++d) {
        Eigen::Matrix<double,4,1> dt;
        dt(0) = -minA(d) + minB(d);
        dt(1) = -minA(d) + maxB(d);
        dt(2) = -maxA(d) + minB(d);
        dt(3) = -maxA(d) + maxB(d);
        min(d) = dt.minCoeff();
        max(d) = dt.maxCoeff();
      }
      std::cout << "min t: " << min.transpose() 
        << " max t: " << max.transpose() << std::endl;

      std::list<bb::NodeR3> nodesR3 =
        bb::GenerateNotesThatTessellateR3(min, max, (max-min).norm());
      bb::LowerBoundR3 lower_bound_R3(gmmA, gmmB, q);
      bb::UpperBoundIndepR3 upper_bound_R3(gmmA, gmmB, q);
      bb::UpperBoundConvexR3 upper_bound_convex_R3(gmmA, gmmB, q);

      if (output_init_bounds) {
        WriteBounds<bb::NodeR3>(lower_bound_R3, upper_bound_R3,
            upper_bound_convex_R3, nodesR3);
      }

      std::cout << "# initial nodes: " << nodesR3.size() << std::endl;
      double eps = 1e-10;
      uint32_t max_it = 5000;
      uint32_t max_lvl = 22;
      bb::BranchAndBound<bb::NodeR3> bbR3(lower_bound_R3, upper_bound_convex_R3);
      std::cout << " BB on R3 eps=" << eps << " max_it=" << max_it << std::endl;
      bb::NodeR3 nodeR3_star = bbR3.Compute(nodesR3, eps, max_lvl, max_it);
      Eigen::Vector3d t =  nodeR3_star.GetLbArgument();
      //    bb::CountBranchesInTree<bb::NodeR3>(nodesR3);
      std::cout << "with LB " << nodeR3_star.GetLB() << " optimum translation: " 
        << t.transpose() << std::endl;
      ts.push_back(t);
      lbsR3(k) = nodeR3_star.GetLB();
      lbs(k) = lbsS3(k) * lbsR3(k);
    }

//    for (auto& node: nodesR3) {
//      std::cout << node.GetLbArgument().transpose() << std::endl;
//    }

    // Display the loaded point clouds.
    if (vm.count("display") && vm.count("verbose")) {
      DisplayPcs(pcA, pcB, gmmA, gmmB, vmfsA, vmfsB, qs[k], ts[k], cfg.lambda/10.);
    }
  }
  std::cout << "Have LbS3: " << lbsS3.transpose() << std::endl;
  std::cout << "Have LbR3: " << lbsR3.transpose() << std::endl;
  std::cout << "Have LBs (LbS3 * LbR3): " << lbs.transpose() << std::endl;
  uint32_t id_max = 0;
  double lb_max = lbs.maxCoeff(&id_max);
  double lbS3 = lbsS3(id_max);
  double lbR3 = lbsR3(id_max);
  t_star = ts[id_max];
  q_star = qs[id_max];
  std::cout << "Updating overall optimum transformation to: " << std::endl;
  std::cout << "q: " << q_star.coeffs().transpose() 
    << " t: " << t_star.transpose() <<  std::endl;

  if(pathOut.size() > 1) {
    std::ofstream out(pathOut + std::string(".csv"));
    out << "q_w q_x q_y q_z t_x t_y t_z lb_S3 lb_R3 KvmfA KvmfB KgmmA KgmmB K" << std::endl; 
    out << q_star.w() << " " << q_star.x() << " " 
      << q_star.y() << " " << q_star.z() << " " << t_star(0)
      << " " << t_star(1) << " " << t_star(2) 
      << " " << lbS3 << " " << lbR3
//      << " " << node_star.GetLB() << " " << nodeR3_star.GetLB()
      << " " << vmfsA.size() << " " << vmfsB.size() 
      << " " << gmmA.size() << " " << gmmB.size() 
      << " " << qs.size()
      << std::endl;
    out.close();
  }

  // Display the loaded point clouds.
  if (vm.count("display")) {
    DisplayPcs(pcA, pcB, gmmA, gmmB, vmfsA, vmfsB, q_star, t_star, cfg.lambda/10.);
  }

  std::cout<<cudaDeviceReset()<<std::endl;
  return (0);
}
