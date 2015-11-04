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

//#include "rtDDPvMF/rtDDPvMF.hpp"
//#include "rtDDPvMF/realtimeDDPvMF_openni.hpp"
#include "optRot/lower_bound_R3.h"
#include "optRot/upper_bound_indep_R3.h"
#include "optRot/upper_bound_convex_R3.h"
#include "optRot/lower_bound_S3.h"
#include "optRot/upper_bound_indep_S3.h"
#include "optRot/upper_bound_convex_S3.h"
#include "optRot/branch_and_bound.h"
#include "optRot/vmf.h"
#include "optRot/vmf_mm.h"
#include "optRot/normal.h"
#include "dpvMFoptRot/dp_vmf_opt_rot.h"
#include "dpvMFoptRot/pc_helper.h"
#include "dpMMlowVar/ddpmeansCUDA.hpp"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

struct CfgRtDDPvMF
{

  CfgRtDDPvMF() : f_d(540.0), lambda(-1.), beta(1e5), Q(-2.),
      nSkipFramesSave(30), nFramesSurvive_(0),lambdaDeg_(90.)
  {};
  CfgRtDDPvMF(const CfgRtDDPvMF& cfg)
    : f_d(cfg.f_d), lambda(cfg.lambda), beta(cfg.beta), Q(cfg.Q),
      nSkipFramesSave(cfg.nSkipFramesSave), nFramesSurvive_(cfg.nFramesSurvive_),
      lambdaDeg_(cfg.lambdaDeg_)
  {
    pathOut = cfg.pathOut;
  };

  double f_d;
  double lambda;
  double beta;
  double Q;

  int32_t nSkipFramesSave;
  std::string pathOut;

  int32_t nFramesSurvive_;
  double lambdaDeg_;

  void lambdaFromDeg(double lambdaDeg)
  {
    lambdaDeg_ = lambdaDeg;
    lambda = cos(lambdaDeg*M_PI/180.0)-1.;
  };
  void QfromFrames2Survive(int32_t nFramesSurvive)
  {
    nFramesSurvive_ = nFramesSurvive;
    Q = nFramesSurvive == 0? -2. : lambda/double(nFramesSurvive);
  };
};

void DisplayPcs(const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcA, 
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcB, 
  const std::vector<OptRot::Normal<3>>& gmmA, 
  const std::vector<OptRot::Normal<3>>& gmmB, 
  const std::vector<OptRot::vMF<3>>& vmfsA,
  const std::vector<OptRot::vMF<3>>& vmfsB,
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

bool ComputevMFMMfromPC(const pcl::PointCloud<pcl::PointXYZRGBNormal>&
    pc, const CfgRtDDPvMF& cfg, std::vector<OptRot::vMF<3>>& vmfs) {
  // take 3 values (x,y,z of normal) with an offset of 4 values (x,y,z
  // and one float which is undefined) and the step is 12 (4 for xyz, 4
  // for normal xyz and 4 for curvature and rgb).
  auto n_map = pc.getMatrixXfMap(3, 12, 4); // this works for PointXYZRGBNormal
  boost::shared_ptr<Eigen::MatrixXf> n(new Eigen::MatrixXf(n_map));
//  std::cout << "normals: " << std::endl << n->rows() 
//    << "x" << n->cols() << std::endl;
  // Setup the DPvMF clusterer.
  shared_ptr<jsc::ClDataGpuf> cld(new jsc::ClDataGpuf(n,0));
  dplv::DDPMeansCUDA<float,dplv::Spherical<float> > pddpvmf(cld,
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
    if (counts(k) > 100) {
      taus(k) = OptRot::vMF<3>::MLEstimateTau(xSum.col(k),
          xSum.col(k).cast<double>()/xSum.col(k).norm(), ws(k));
    }
  
  for(uint32_t k=0; k<K; ++k)
    if (counts(k) > 100) {
      vmfs.push_back(OptRot::vMF<3>(xSum.col(k).cast<double>()/xSum.col(k).norm(),
            taus(k), pis(k)));
    }
  return true;
}

bool ComputeGMMfromPC(pcl::PointCloud<pcl::PointXYZRGBNormal>&
    pc, const CfgRtDDPvMF& cfg, std::vector<OptRot::Normal<3>>& gmm,
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
//  std::cout << "K: " << K << std::endl;
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
  
  for(uint32_t k=0; k<K; ++k)
//    if (pis(k) > 0.) {
    if (counts(k) > 100) {
      gmm.push_back(OptRot::Normal<3>(centroids.col(k).cast<double>(),
            Ss[k]/float(ws(k))+0.01*Eigen::Matrix3d::Identity(), pis(k)));
//        Ss[k]/float(ws(k)), pis(k)));
    }
  return true;
}

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
    ("display,d", "display results")
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

  std::vector<OptRot::vMF<3>> vmfsA;
  ComputevMFMMfromPC(pcA, cfg, vmfsA);
  for(uint32_t k=0; k<vmfsA.size(); ++k) vmfsA[k].Print(); 

  std::vector<OptRot::vMF<3>> vmfsB;
  ComputevMFMMfromPC(pcB, cfg, vmfsB);
  for(uint32_t k=0; k<vmfsB.size(); ++k) vmfsB[k].Print(); 

  if (egi_mode) {
    for(uint32_t k=0; k<vmfsA.size(); ++k) 
      vmfsA[k].SetPi(1./float(vmfsA.size()));
    for(uint32_t k=0; k<vmfsB.size(); ++k) 
      vmfsB[k].SetPi(1./float(vmfsB.size()));
  }

  cfg.lambda = lambdaT;
  std::vector<OptRot::Normal<3>> gmmA;
  ComputeGMMfromPC(pcA, cfg, gmmA, false);
//  for(uint32_t k=0; k<gmmA.size(); ++k) gmmA[k].Print(); 

  std::vector<OptRot::Normal<3>> gmmB;
  ComputeGMMfromPC(pcB, cfg, gmmB, false);
//  for(uint32_t k=0; k<gmmB.size(); ++k) gmmB[k].Print(); 


  OptRot::vMFMM<3> vmfmmA(vmfsA);
  OptRot::vMFMM<3> vmfmmB(vmfsB);
  std::cout << " Tessellate S3" << std::endl;
  std::list<OptRot::NodeS3> nodes = OptRot::GenerateNotesThatTessellateS3();
  std::cout << "# initial nodes: " << nodes.size() << std::endl;
  OptRot::LowerBoundS3 lower_bound(vmfmmA, vmfmmB);
  OptRot::UpperBoundIndepS3 upper_bound(vmfmmA, vmfmmB);
  OptRot::UpperBoundConvexS3 upper_bound_convex(vmfmmA, vmfmmB);

  if (output_init_bounds) {
    WriteBounds<OptRot::NodeS3>(lower_bound, upper_bound, upper_bound_convex,
        nodes);
  }
  
  double eps = 1e-7;
  uint32_t max_it = 15000;
  std::cout << " BB on S3 eps=" << eps << " max_it=" << max_it << std::endl;
//  OptRot::BranchAndBound<OptRot::NodeS3> bb(lower_bound, upper_bound);
  OptRot::BranchAndBound<OptRot::NodeS3> bb(lower_bound, upper_bound_convex);
  OptRot::NodeS3 node_star = bb.Compute(nodes, eps, max_it);
//  OptRot::CountBranchesInTree<OptRot::NodeS3>(nodes);
  Eigen::Quaterniond q_star = node_star.GetLbArgument();
  std::cout << "optimum BB quaternion: "  << q_star.coeffs().transpose()
      << " angle: " << 2.*acos(q_star.w()) * 180. / M_PI << std::endl
      << q_star.toRotationMatrix()
      << std::endl;
  std::cout << node_star.GetTetrahedron().GetCenterQuaternion().coeffs().transpose() << std::endl;
  Eigen::Quaterniond q = q_star;
  Eigen::Vector3d t_star; 

//  shared_ptr<Eigen::MatrixXd> qs(new MatrixXd(4, nodes.size()));
//  auto it=nodes.begin();
//  for (uint32_t i=0; i < nodes.size(); ++i, it++) {
//    qs->col(i) = it->GetTetrahedron().GetCenter();
//  }
//  double lambda_q = cos(2.*10.*M_PI/180.) - 1.; // TODO
//  dplv::DPMeans<double, dplv::Spherical<double>> dpvMF(qs, 0, lambda_q);
//  for (uint32_t t=0; t<20; ++t) {
//    dpvMF.updateLabels(); 
//    dpvMF.updateCenters(); 
//  }
//  std::cout << dpvMF.centroids() << std::endl; 
//
//  double lb_R3 = -1e40;
//  Eigen::Vector3d t_star; 
//  for (uint32_t k=0; k<dpvMF.K(); ++k) {
//    Eigen::Vector4d q_vec = dpvMF.centroids().col(k);
//    Eigen::Quaterniond q(q_vec(0), q_vec(1), q_vec(2), q_vec(3));
////    auto it = nodes.begin();
////    auto z = dpvMF.z();
////    double lb_max = 9999.;
////    for (uint32_t i=0; i<nodes.size(); ++i, it++) 
////      if (z(i) == k && lb_max > it->GetBoundGap()) {
////        lb_max = it->GetBoundGap();
////        q = it->GetTetrahedron().GetCenterQuaternion();
////      }
//    //  Eigen::Quaterniond q = node_star.GetTetrahedron().GetCenterQuaternion();
//    std::cout << "in cluster " << k << ": center = "  
//      << dpvMF.centroids().col(k).transpose() << std::endl;
//    std::cout << "optimum quaternion: "  << q.coeffs().transpose()
//      << " angle: " << 2.*acos(q.w()) * 180. / M_PI
//      << std::endl;
    // This q is the inverse of the rotation that brings B to A.
    q = q.inverse(); // A little ugly but this is because of the way we setup the problem...

    // To get all corners of the bounding box.j
    OptRot::Box box(minA, maxA);
    // Update the boundaries of the the rotated point cloud A to get
    // the full translation space later on.
    for (uint32_t i=0; i<8; ++i) {
      Eigen::Vector3d c;
      box.GetCorner(i,c);
      c = q._transformVector(c);
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

    std::list<OptRot::NodeR3> nodesR3 =
      OptRot::GenerateNotesThatTessellateR3(min, max, (max-min).norm()/10.);
    OptRot::LowerBoundR3 lower_bound_R3(gmmA, gmmB, q);
    OptRot::UpperBoundIndepR3 upper_bound_R3(gmmA, gmmB, q);
    OptRot::UpperBoundConvexR3 upper_bound_convex_R3(gmmA, gmmB, q);

    if (output_init_bounds) {
      WriteBounds<OptRot::NodeR3>(lower_bound_R3, upper_bound_R3,
          upper_bound_convex_R3, nodesR3);
    }

    std::cout << "# initial nodes: " << nodesR3.size() << std::endl;
    eps = 1e-10;
    max_it = 10000;
    OptRot::BranchAndBound<OptRot::NodeR3> bbR3(lower_bound_R3, upper_bound_convex_R3);
    std::cout << " BB on R3 eps=" << eps << " max_it=" << max_it << std::endl;
    OptRot::NodeR3 nodeR3_star = bbR3.Compute(nodesR3, eps, max_it);
    Eigen::Vector3d t =  nodeR3_star.GetLbArgument();
//    OptRot::CountBranchesInTree<OptRot::NodeR3>(nodesR3);
    std::cout << "with LB " << nodeR3_star.GetLB() << " optimum translation: " 
      << t.transpose() << std::endl;
//    if (lb_R3 < nodeR3_star.GetLB()) {
      std::cout << "Updating overall optimum transformation to: " << std::endl;
      t_star = t;
      q_star = q;
//      lb_R3 = nodeR3_star.GetLB();
      std::cout << "q: " << q_star.coeffs().transpose() 
        << " t: " << t_star.transpose() <<  std::endl;
//    }
    if(pathOut.size() > 1) {
      std::ofstream out(pathOut + std::string(".csv"));
      out << "q_w q_x q_y q_z t_x t_y t_z lb_S3 lb_R3 KvmfA KvmfB KgmmA KgmmB" << std::endl; 
      out << q_star.w() << " " << q_star.x() << " " 
        << q_star.y() << " " << q_star.z() << " " << t_star(0)
        << " " << t_star(1) << " " << t_star(2) 
        << " " << node_star.GetLB() << " " << nodeR3_star.GetLB()
        << " " << vmfsA.size() << " " << vmfsB.size() 
        << " " << gmmA.size() << " " << gmmB.size() 
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
