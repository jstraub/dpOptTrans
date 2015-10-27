/* Copyright (c) 2015, Julian Straub <jstraub@csail.mit.edu> Licensed
 * under the MIT license. See the license file LICENSE.
 */

#include <iostream>
#include <string>
#include <pcl/search/kdtree.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/transforms.h>

#include "rtDDPvMF/rtDDPvMF.hpp"
//#include "rtDDPvMF/realtimeDDPvMF_openni.hpp"
#include "optRot/lower_bound_R3.h"
#include "optRot/upper_bound_indep_R3.h"
#include "optRot/upper_bound_convex_R3.h"
#include "optRot/lower_bound_S3.h"
#include "optRot/upper_bound_indep_S3.h"
#include "optRot/upper_bound_convex_S3.h"
#include "optRot/branch_and_bound.h"
#include "optRot/vmf_mm.h"
#include "optRot/normal.h"
#include "dpvMFoptRot/dp_vmf_opt_rot.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

void UniformlySamplePc(const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcIn, 
  pcl::PointCloud<pcl::PointXYZRGBNormal>& pcOut) {
  
}

void ComputeAreaWeightsPc(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcIn,
    float scale) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcXYZ(new
      pcl::PointCloud<pcl::PointXYZ>(pcIn.size(),1));
  pcXYZ->getMatrixXfMap(3,4,0) = pcIn.getMatrixXfMap(3, 12, 0);

  pcl::search::KdTree<pcl::PointXYZ> kd; 
  kd.setInputCloud(pcXYZ);

  std::vector<int> k_ids;
  std::vector<float> k_sqr_dists;
//  double W = 0.; // Sum over all weights.
  for (uint32_t i=0; i<pcIn.size(); ++i) {
    int k_found = kd.nearestKSearch(pcXYZ->at(i), 9, k_ids, k_sqr_dists);
    std::sort(k_sqr_dists.begin(), k_sqr_dists.begin()+k_found);
    float median_sqr_dist = k_sqr_dists[k_found/2];
    if (k_found%2 == 0)
      median_sqr_dist = (k_sqr_dists[k_found/2-1]+k_sqr_dists[k_found/2])*0.5;
//    float scale_3d = median_sqr_dist*sqrt(median_sqr_dist)*4./3.*M_PI;
    float scale_2d = median_sqr_dist*M_PI*1e2*scale*scale; // in dm^2
    // Abuse curvature entry for the scale.
    pcIn.at(i).curvature = scale_2d;
//    W += scale;
//    if (i%10 == 0)
//      std::cout << k_found << ": "<< sqrt(median_sqr_dist) << " -> " << scale << std::endl;
  }
//  std::cout << "W: " << W << std::endl;
  // Normalize so that the sum over all weights will be 1.
//  for (uint32_t i=0; i<pcIn.size(); ++i) {
//    pcIn.at(i).curvature /= W;
////      if (i%100 == 0)
////        std::cout << pcIn.at(i).curvature << " " << pcIn.at(i).curvature/W << std::endl;
//  }
}

bool ComputevMFMMfromPC(const pcl::PointCloud<pcl::PointXYZRGBNormal>&
    pc, const CfgRtDDPvMF& cfg, std::vector<OptRot::vMF<3>>& vmfs) {
  // take 3 values (x,y,z of normal) with an offset of 4 values (x,y,z
  // and one float which is undefined) and the step is 12 (4 for xyz, 4
  // for normal xyz and 4 for curvature and rgb).
  auto n_map = pc.getMatrixXfMap(3, 12, 4); // this works for PointXYZRGBNormal
  boost::shared_ptr<Eigen::MatrixXf> n(new Eigen::MatrixXf(n_map));
  std::cout << "normals: " << std::endl << n->rows() 
    << "x" << n->cols() << std::endl;
//  auto d = pc.getMatrixXfMap(1, 12, 2); // this works for PointXYZRGBNormal
//  std::cout << "depth: " << std::endl << d.rows() 
//    << "x" << d.cols() << std::endl;

  // Setup the DPvMF clusterer.
  shared_ptr<jsc::ClDataGpuf> cld(new jsc::ClDataGpuf(n,0));
  dplv::DDPMeansCUDA<float,dplv::Spherical<float> > pddpvmf(cld,
      cfg.lambda, cfg.Q, cfg.beta);
  // Run the clustering algorithm.
  pddpvmf.nextTimeStep(n);
  for(uint32_t i=0; i<10; ++i)
  {
    cout<<"@"<<i<<" :"<<endl;
    pddpvmf.updateLabels();
    pddpvmf.updateCenters();
    if(pddpvmf.convergedCounts(n->cols()/100)) break;
  }
  pddpvmf.getZfromGpu(); // cache z_ back from gpu
  const VectorXu& z = pddpvmf.z();
  // Output results.
  MatrixXf centroids = pddpvmf.centroids();
  VectorXf counts = pddpvmf.counts().cast<float>();
  std::cout << counts.transpose() << std::endl;
  std::cout << centroids << std::endl;
  uint32_t K = counts.rows();
  std::cout << "K: " << K << std::endl;
  // Compute vMF statistics: area-weighted sum over surface normals
  // associated with respective cluster. 
  MatrixXd xSum = MatrixXd::Zero(3,K);
  counts.fill(0.);
  for (uint32_t i=0; i<n->cols(); ++i) 
    if(z(i) < K) {
      float w = pc.at(i).curvature; // curvature is used to store the weights.
//      if (i%100 == 0)
//        std::cout << w << std::endl;
//      double scale = d(0,i)/cfg.f_d;
      xSum.col(z(i)) += n->col(i).cast<double>()*w;
      counts(z(i)) += w;
    }
  // Fractions belonging to each cluster.
  Eigen::VectorXd pis = (counts.array() / counts.sum()).matrix().cast<double>();
  
  for(uint32_t k=0; k<K; ++k)
    if (pis(k) > 0.) {
//      vmfs.push_back(OptRot::vMF<3>(centroids.col(k).cast<double>(),
      vmfs.push_back(OptRot::vMF<3>(xSum.col(k).cast<double>()/xSum.col(k).norm(),
            xSum.col(k).norm(), pis(k)));
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
  std::cout << "xyz: " << std::endl << xyz->rows() 
    << "x" << xyz->cols() << std::endl;

  // Setup the DPvMF clusterer.
  shared_ptr<jsc::ClDataGpuf> cld(new jsc::ClDataGpuf(xyz,0));
  dplv::DDPMeansCUDA<float,dplv::Euclidean<float> > dpmeans(cld,
      cfg.lambda, cfg.Q, cfg.beta);
  // Run the clustering algorithm.
  dpmeans.nextTimeStep(xyz);
  for(uint32_t i=0; i<10; ++i)
  {
    cout<<"@"<<i<<" :"<<endl;
    dpmeans.updateLabels();
    dpmeans.updateCenters();
    if(dpmeans.convergedCounts(xyz->cols()/100)) break;
  }
  dpmeans.getZfromGpu(); // cache z_ back from gpu
  const VectorXu& z = dpmeans.z();
  // Output results.
  MatrixXf centroids = dpmeans.centroids();
  VectorXf counts = dpmeans.counts().cast<float>();
  std::cout << counts.transpose() << std::endl;
  std::cout << centroids << std::endl;
  uint32_t K = counts.rows();
  std::cout << "K: " << K << std::endl;
  // Compute Gaussian statistics: 
  std::vector<MatrixXd> Ss(K,Matrix3d::Zero());
  Eigen::MatrixXd xSum = Eigen::MatrixXd::Zero(3,K);
  counts.fill(0.);
  for (uint32_t i=0; i<xyz->cols(); ++i) 
    if(z(i) < K) {
      float w = pc.at(i).curvature; // curvature is used to store the weights.
      counts(z(i)) += w;
      xSum.col(z(i)) += xyz->col(i).cast<double>()*w;
    }
  for(uint32_t k=0; k<K; ++k)
    centroids.col(k) = xSum.col(k).cast<float>()/counts(k);

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
  Eigen::VectorXd pis = (counts.array() / counts.sum()).matrix().cast<double>();
  
  for(uint32_t k=0; k<K; ++k)
    if (pis(k) > 0.) {
      gmm.push_back(OptRot::Normal<3>(centroids.col(k).cast<double>(),
            Ss[k]/float(counts(k))+0.01*Eigen::Matrix3d::Identity(), pis(k)));
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

  string pathA = "";
  string pathB = "";
  //  string mode = "";
  //  if(vm.count("mode")) mode = vm["mode"].as<string>();
  if(vm.count("in_a")) pathA = vm["in_a"].as<string>();
  if(vm.count("in_b")) pathB = vm["in_b"].as<string>();
  float scale = 1.;
  if(vm.count("scale")) scale = vm["scale"].as<float>();

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

  ComputeAreaWeightsPc(pcA, scale);
  ComputeAreaWeightsPc(pcB, scale);

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

  cfg.lambda = lambdaT;
  std::vector<OptRot::Normal<3>> gmmA;
  ComputeGMMfromPC(pcA, cfg, gmmA, false);
  for(uint32_t k=0; k<gmmA.size(); ++k) gmmA[k].Print(); 

  std::vector<OptRot::Normal<3>> gmmB;
  ComputeGMMfromPC(pcB, cfg, gmmB, false);
  for(uint32_t k=0; k<gmmB.size(); ++k) gmmB[k].Print(); 

  OptRot::vMFMM<3> vmfmmA(vmfsA);
  OptRot::vMFMM<3> vmfmmB(vmfsB);
  std::cout << " Tessellate S3" << std::endl;
  std::list<OptRot::NodeS3> nodes = OptRot::GenerateNotesThatTessellateS3();
  std::cout << "# initial nodes: " << nodes.size() << std::endl;
  OptRot::LowerBoundS3 lower_bound(vmfmmA, vmfmmB);
  OptRot::UpperBoundIndepS3 upper_bound(vmfmmA, vmfmmB);
  OptRot::UpperBoundConvexS3 upper_bound_convex(vmfmmA, vmfmmB);
  
  double eps = 1e-8;
  uint32_t max_it = 3000;
  std::cout << " BB on S3 eps=" << eps << " max_it=" << max_it << std::endl;
  OptRot::BranchAndBound<OptRot::NodeS3> bb(lower_bound, upper_bound);
//  OptRot::BranchAndBound<OptRot::NodeS3> bb(lower_bound, upper_bound_convex);
  OptRot::NodeS3 node_star = bb.Compute(nodes, eps, max_it);
  OptRot::CountBranchesInTree<OptRot::NodeS3>(nodes);

  std::cout << "optimum quaternion: " 
    << node_star.GetTetrahedron().GetCenter().transpose()
    << std::endl;


  Eigen::Quaterniond q_star = node_star.GetTetrahedron().GetCenterQuaternion();
  // This q_star is the inverse of the rotation that brings B to A.
  q_star = q_star.inverse(); // A little ugly but this is because of the way we setup the problem...

  // Obtain the range for the translation.
  // TODO: this could be made more tight by getting min and max for the
  // rotated point-clouds.
  Eigen::Vector3d minA, minB, maxA, maxB, min, max;
  ComputePcBoundaries(pcA, minA, maxA);
  ComputePcBoundaries(pcB, minB, maxB);
  for (uint32_t d=0; d<3; ++d) {
//    Eigen::Matrix<double,8,1> dt;
    Eigen::Matrix<double,4,1> dt;
//    dt(0) = minA(d) - minB(d);
//    dt(1) = minA(d) - maxB(d);
//    dt(2) = maxA(d) - minB(d);
//    dt(3) = maxA(d) - maxB(d);
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
    OptRot::GenerateNotesThatTessellateR3(min, max, 10.);
  OptRot::LowerBoundR3 lower_bound_R3(gmmA, gmmB, q_star);
  OptRot::UpperBoundIndepR3 upper_bound_R3(gmmA, gmmB, q_star);
  OptRot::UpperBoundConvexR3 upper_bound_convex_R3(gmmA, gmmB, q_star);

  std::cout << "# initial nodes: " << nodesR3.size() << std::endl;
  eps = 1e-8;
  max_it = 3000;
  OptRot::BranchAndBound<OptRot::NodeR3> bbR3(lower_bound_R3, upper_bound_convex_R3);
  std::cout << " BB on R3 eps=" << eps << " max_it=" << max_it << std::endl;
  OptRot::NodeR3 nodeR3_star = bbR3.Compute(nodesR3, eps, max_it);
  OptRot::CountBranchesInTree<OptRot::NodeR3>(nodesR3);

  Eigen::Vector3d t_star = nodeR3_star.GetBox().GetCenter();
  std::cout << "optimum translation: " << t_star << std::endl;
  Eigen::Affine3f T = Eigen::Affine3f::Identity();
  T.translation() = q_star.inverse()._transformVector(t_star).cast<float>();
  T.rotate(q_star.inverse().cast<float>());
//  T.translation() = t_star.cast<float>();
//  T.rotate(q_star.cast<float>());
  std::cout << "optimal transformation: \n" << T.matrix() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcB_T_ptr(new
      pcl::PointCloud<pcl::PointXYZRGBNormal>());

  for (uint32_t i=0; i<pcB.size(); ++i) {
    Eigen::Map<Eigen::Vector3f> p(&(pcB.at(i).x));
    p = q_star.cast<float>().inverse()._transformVector( p - t_star.cast<float>());
    Eigen::Map<Eigen::Vector3f> n(pcB.at(i).normal);
    n = q_star.cast<float>().inverse()._transformVector(n);
  }

  // Display the loaded point clouds.
  if (vm.count("display")) {

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new
        pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    viewer->setBackgroundColor (1., 1., 1.);
    viewer->addCoordinateSystem (0.3);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcA_ptr = pcA.makeShared();
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcB_ptr = pcB.makeShared();

    pcA_ptr->sensor_origin_.fill(0.);
    pcA_ptr->sensor_origin_(3) = 1.;
    pcA_ptr->sensor_orientation_.setIdentity();

    pcB_ptr->sensor_origin_.topRows<3>() = t_star.cast<float>();
    pcB_ptr->sensor_origin_(3) = 1.;
    pcB_ptr->sensor_orientation_ = q_star.inverse().cast<float>();

    pcB_ptr->sensor_origin_.fill(0.);
    pcB_ptr->sensor_origin_(3) = 1.;
    pcB_ptr->sensor_orientation_.setIdentity();

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>
      rgbA(pcA_ptr);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>
      rgbB(pcB_ptr);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>
      rgbB_T(pcB_T_ptr);
    viewer->addPointCloud<pcl::PointXYZRGBNormal> (pcA_ptr, rgbA, "cloudA");
    viewer->addPointCloud<pcl::PointXYZRGBNormal> (pcB_ptr, rgbB, "cloudB");
//    viewer->addPointCloud<pcl::PointXYZRGBNormal> (pcB_T_ptr, rgbB_T,
//        "cloudB transformed");

    std::cout << "cloud origins for display:" <<   std::endl 
      << pcA_ptr->sensor_origin_.transpose()  << " q: " 
      << pcA_ptr->sensor_orientation_.coeffs().transpose() <<  std::endl
      << pcB_ptr->sensor_origin_.transpose()  << " q: " 
      << pcB_ptr->sensor_orientation_.coeffs().transpose() <<  std::endl;

    char label[10];
    for(uint32_t k=0; k<gmmA.size(); ++k) {
      pcl::PointXYZ p;
      p.x = gmmA[k].GetMu()(0);
      p.y = gmmA[k].GetMu()(1);
      p.z = gmmA[k].GetMu()(2);
      sprintf(label,"SA%d",k);
      viewer->addSphere(p, cfg.lambda/10., 1,0,0, label);
    }
    for(uint32_t k=0; k<gmmB.size(); ++k) {
      Eigen::Vector3d mu =
        q_star.inverse()._transformVector(gmmB[k].GetMu() -t_star);
      pcl::PointXYZ p;
      p.x = mu(0);
      p.y = mu(1);
      p.z = mu(2);
      sprintf(label,"SB%d",k);
      viewer->addSphere(p, cfg.lambda/10.,0,1,0, label);
    }
    while (!viewer->wasStopped ()) {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  }

//  pcl::PointCloud<pcl::PointXYZI>::Ptr pcA(DepthImageToPc(dIa, gIa, cfg.f_d, q0));
//  pcl::PointCloud<pcl::PointXYZI>::Ptr pcB(DepthImageToPc(dIb, gIb, cfg.f_d, qStar));
//  std::cout << pcA->width << " " << pcA->height << std::endl;
//  std::cout << pcB->width << " " << pcB->height << std::endl;
//
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  viewer->initCameraParameters ();
////  int v1(0);
////  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//  viewer->setBackgroundColor (0, 0, 0);
////  viewer_->setBackgroundColor (255, 255, 255);
//  viewer->addCoordinateSystem (1.0);
//  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> rgbA(pcA,"intensity"); 
//  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> rgbB(pcB,"intensity"); 
//  viewer->addPointCloud<pcl::PointXYZI> (pcA, rgbA, "sample cloud1");
//  viewer->addPointCloud<pcl::PointXYZI> (pcB, rgbB, "sample cloud2");
//
//  while (!viewer->wasStopped ())
//  {
//    viewer->spinOnce (100);
//    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//  }
//
//  if(vm.count("out"))
//  {
//    std::cout<<" writing out put to "<<std::endl
//      <<(vm["out"].as<string>()+"_rgbLabels.png")<<std::endl
//      <<vm["out"].as<string>()+"_cRmf.csv"<<std::endl;
////    cv::imwrite(vm["out"].as<string>()+"_rgbLabels.png",Iout);
////    ofstream out((vm["out"].as<string>()+"_cRmf.csv").data(),
////        ofstream::out);
////    for(uint32_t i=0; i<centroids.rows();++i) 
////    {
////      for(uint32_t j=0; j<centroids.cols()-1;++j) 
////        out << centroids(i,j)<<" ";
////      out << centroids(i,centroids.cols()-1)<<std::endl;
////    }
////    for(uint32_t i=0; i<concentrations.size()-1;++i) 
////      out << concentrations(i) << " ";
////    out << concentrations(concentrations.size()-1) << std::endl;
////    for(uint32_t i=0; i<proportions.size()-1;++i) 
////      out << proportions(i) << " ";
////    out << proportions(proportions.size()-1) << std::endl;
////    out.close();
//  }
  std::cout<<cudaDeviceReset()<<std::endl;
  return (0);
}
