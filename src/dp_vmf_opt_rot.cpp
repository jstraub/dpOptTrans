/* Copyright (c) 2015, Julian Straub <jstraub@csail.mit.edu> Licensed
 * under the MIT license. See the license file LICENSE.
 */

#include <iostream>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include "rtDDPvMF/rtDDPvMF.hpp"
//#include "rtDDPvMF/realtimeDDPvMF_openni.hpp"
#include "bbTrans/lower_bound_S3.h"
#include "bbTrans/upper_bound_indep_S3.h"
#include "bbTrans/upper_bound_convex_S3.h"
#include "bbTrans/branch_and_bound.h"
#include "bbTrans/vmf_mm.h"
#include "bbTrans/node.h"
#include "dpOptTrans/dp_vmf_opt_rot.h"
#include "dpOptTrans/pc_helper.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

bool ComputevMFMMfromDepth(
  shared_ptr<RtDDPvMF> pRtDDPvMF,
  const CfgRtDDPvMF& cfg,
    const std::string& path, bool display,
    cv::Mat *dI, 
    cv::Mat *gI, 
    std::vector<::bb::vMF<3>>* vmfs) {

  std::cout<<"reading depth image from "<<path<<std::endl;
  cv::Mat depth = cv::imread(path, CV_LOAD_IMAGE_ANYDEPTH);
  std::cout<<"type: "<<int(depth.type()) <<" != "<<int(CV_16U) <<std::endl;

  string pathRgb(path);
  pathRgb.replace(path.length()-5,1,"rgb");
  std::cout<<"reading rgb image from "<<pathRgb<<std::endl;
  cv::Mat gray = cv::imread(pathRgb, CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat rgb = cv::imread(pathRgb);
  *gI = gray;

  if (display) {
    cv::Mat dI(depth.rows,depth.cols,CV_8UC1);
    depth.convertTo(dI,CV_8UC1,255./4000.,-19.);
    cv::imshow("d",dI);
    cv::imshow("rgb",rgb);
    cv::waitKey(0);
  }
  cv::Mat nI;
  cv::Mat zI;
  cv::Mat Iout;
  MatrixXf centroids;
  VectorXf concentrations;
  VectorXf proportions;

//  uint32_t T = 10;
//  for(uint32_t i=0; i<T; ++i)
  pRtDDPvMF->compute(reinterpret_cast<uint16_t*>(depth.data),
      depth.cols,depth.rows);
  Iout = pRtDDPvMF->overlaySeg(rgb,false,true);
  *dI = pRtDDPvMF->smoothDepthImg();
  //      cv::Mat Iout = pRtDDPvMF->overlaySeg(gray);
  if(display) {
    nI = pRtDDPvMF->normalsImg();
    zI = pRtDDPvMF->labelsImg(true);
    cv::waitKey(0);
  }
  centroids = pRtDDPvMF->GetCentroids();
  const VectorXu z = pRtDDPvMF->labels();
  uint32_t K = pRtDDPvMF->GetK();
  proportions = pRtDDPvMF->GetCounts();
  proportions /= proportions.sum();
  std::cout << "counts and proportions: " << std::endl;
  std::cout << pRtDDPvMF->GetCounts().transpose() << " normalized: "
    << proportions.transpose() << std::endl;
  concentrations = VectorXf::Zero(K);

  cv::Mat normals = pRtDDPvMF->normalsImgRaw();
  depth = pRtDDPvMF->smoothDepth();
  MatrixXf xSum = MatrixXf::Zero(3,K);
  for (uint32_t i=0; i<normals.cols; ++i) 
    for (uint32_t j=0; j<normals.rows; ++j) 
      if(z(normals.cols*j +i) < K) {
        Eigen::Map<Matrix<float,3,1> > q(&(normals.at<cv::Vec3f>(j,i)[0]));
        float d = depth.at<float>(j,i);
        float scale = d/cfg.f_d;
        xSum.col(z(normals.cols*j +i)) += q*scale;
      }
  std::cout<< pRtDDPvMF->GetxSums() << " vs weighted: " << xSum << std::endl;
  for (uint32_t k = 0; k < K; ++k) {
    concentrations(k) = (xSum.col(k)).norm();
    std::cout << xSum.col(k).transpose() << " || " <<  xSum.col(k).norm() << std::endl;
  }

  if(display)  {
    cv::imshow("dS",*dI);
    cv::imshow("normals",nI);
    cv::imshow("zI",zI);
    cv::imshow("out",Iout);
    cv::waitKey(0);
  }
  *dI = depth;

  for(uint32_t k=0; k<centroids.cols(); ++k) {
    vmfs->push_back(::bb::vMF<3>(centroids.col(k).cast<double>(),
          concentrations(k), proportions(k)));
  }
  return true;
};


pcl::PointCloud<pcl::PointXYZI>::Ptr DepthImageToPc(const cv::Mat& dI, const cv::Mat& gI, float f_d, const Eigen::Quaterniond q) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new
      pcl::PointCloud<pcl::PointXYZI>(dI.cols, dI.rows));
  for (uint32_t i=0; i<dI.rows; ++i)
    for (uint32_t j=0; j<dI.cols; ++j) {
//      std::cout << dI.at<float>(i,j) << " ";
      pc->at(j,i).intensity = static_cast<float>(gI.at<uint8_t>(i,j))/255.;
//      std::cout << pc->at(j,i).intensity << " " << uint32_t(gI.at<uint8_t>(i,j)) << "; " ;
      pc->at(j,i).z = dI.at<float>(i,j);
      pc->at(j,i).x = (j-float(dI.cols-1.)*0.5) * pc->at(j,i).z / f_d;
      pc->at(j,i).y = (i-float(dI.rows-1.)*0.5) * pc->at(j,i).z / f_d;
      Eigen::Map<Matrix<float,3,1>> p(&(pc->at(j,i).x));
      p = q._transformVector(p.cast<double>()).cast<float>();
    }
  pc->is_dense = false;
  return pc;
}

int main(int argc, char** argv) {
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("lambdaDeg,l", po::value<double>(), "lambda in degree for dp and ddp")
    ("in_a,a", po::value<string>(), "path to first input file")
    ("in_b,b", po::value<string>(), "path to second input file")
    ("out,o", po::value<string>(), "path to output file")
    ("display,d", "display results")
    ("B,B", po::value<int>(), "B for guided filter")
    ("eps", po::value<float>(), "eps for guided filter")
    ("f_d,f", po::value<float>(), "focal length of depth camera")
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

  cfg.lambdaFromDeg(lambdaDeg);
  cfg.QfromFrames2Survive(cfg.nFramesSurvive_);

  string pathA = "";
  string pathB = "";
  //  string mode = "";
  cudaPcl::CfgSmoothNormals cfgNormals;
  cfgNormals.f_d = 540.;
  cfgNormals.eps = 0.2*0.2;
  cfgNormals.B = 9;
  cfgNormals.compress = true;
  //  if(vm.count("mode")) mode = vm["mode"].as<string>();
  if(vm.count("in_a")) pathA = vm["in_a"].as<string>();
  if(vm.count("in_b")) pathB = vm["in_b"].as<string>();
  if(vm.count("eps")) cfgNormals.eps = vm["eps"].as<float>();
  if(vm.count("f_d")) cfgNormals.f_d = vm["f_d"].as<float>();
  if(vm.count("B")) cfgNormals.B = uint32_t( vm["B"].as<int>());

  findCudaDevice(argc,(const char**)argv);

  std::cout<<"rtDDPvMFmeans lambdaDeg="<<cfg.lambdaDeg_<<" beta="<<cfg.beta
    <<"nFramesSurvive="<<cfg.nFramesSurvive_<<std::endl;
  std::cout<<"output path: "<<cfg.pathOut<<std::endl;

  std::vector<bb::vMF<3>> vmfs_A;

  cv::Mat dIa, gIa;
  shared_ptr<RtDDPvMF> pRtDDPvMF_A;
  pRtDDPvMF_A = shared_ptr<RtDDPvMF>(new RtDDPvMF(cfg,cfgNormals));
  ComputevMFMMfromDepth(pRtDDPvMF_A, cfg, pathA, vm.count("display"), &dIa, &gIa, &vmfs_A);
  bb::vMFMM<3> vmf_mm_A(vmfs_A);

  cv::Mat dIb, gIb;
  std::vector<bb::vMF<3>> vmfs_B;
  shared_ptr<RtDDPvMF> pRtDDPvMF_B;
  pRtDDPvMF_B = shared_ptr<RtDDPvMF>(new RtDDPvMF(cfg,cfgNormals));
  ComputevMFMMfromDepth(pRtDDPvMF_B, cfg, pathB, vm.count("display"), &dIb, &gIb, &vmfs_B);
  bb::vMFMM<3> vmf_mm_B(vmfs_B);

  std::cout << "A:" << std::endl;
  for (uint32_t i=0; i<vmf_mm_A.GetK(); ++i)
    std::cout << vmf_mm_A.Get(i).GetMu().transpose() 
      << " tau " << vmf_mm_A.Get(i).GetTau()
      << " pi " << vmf_mm_A.Get(i).GetPi() << std::endl;

  std::cout << "B:" << std::endl;
  for (uint32_t i=0; i<vmf_mm_B.GetK(); ++i)
    std::cout << vmf_mm_B.Get(i).GetMu().transpose() 
      << " tau " << vmf_mm_B.Get(i).GetTau()
      << " pi " << vmf_mm_B.Get(i).GetPi() << std::endl;

  std::list<bb::NodeS3> nodes = bb::GenerateNotesThatTessellateS3();
  bb::LowerBoundS3 lower_bound(vmf_mm_A, vmf_mm_B);
  bb::UpperBoundIndepS3 upper_bound(vmf_mm_A, vmf_mm_B);
  bb::UpperBoundConvexS3 upper_bound_convexity(vmf_mm_A, vmf_mm_B);
  
  bb::BranchAndBound<bb::NodeS3> bb(lower_bound, upper_bound_convexity);
  double eps = 1e-4 / 180. * M_PI;
  uint32_t max_lvl = 60;
  uint32_t max_it = 300;
  bb::NodeS3 node_star = bb.Compute(nodes, eps, max_lvl, max_it);

  std::cout << "optimum quaternion: " 
    << node_star.GetTetrahedron().GetCenter().transpose()
    << std::endl;

  Eigen::Quaterniond q0;
  Eigen::Quaterniond qStar = node_star.GetTetrahedron().GetCenterQuaternion();
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcA(DepthImageToPc(dIa, gIa, cfg.f_d, q0));
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcB(DepthImageToPc(dIb, gIb, cfg.f_d, qStar));
  std::cout << pcA->width << " " << pcA->height << std::endl;
  std::cout << pcB->width << " " << pcB->height << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();
//  int v1(0);
//  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0);
//  viewer_->setBackgroundColor (255, 255, 255);
  viewer->addCoordinateSystem (1.0);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> rgbA(pcA,"intensity"); 
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> rgbB(pcB,"intensity"); 
  viewer->addPointCloud<pcl::PointXYZI> (pcA, rgbA, "sample cloud1");
  viewer->addPointCloud<pcl::PointXYZI> (pcB, rgbB, "sample cloud2");

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  if(vm.count("out"))
  {
    std::cout<<" writing out put to "<<std::endl
      <<(vm["out"].as<string>()+"_rgbLabels.png")<<std::endl
      <<vm["out"].as<string>()+"_cRmf.csv"<<std::endl;
//    cv::imwrite(vm["out"].as<string>()+"_rgbLabels.png",Iout);
//    ofstream out((vm["out"].as<string>()+"_cRmf.csv").data(),
//        ofstream::out);
//    for(uint32_t i=0; i<centroids.rows();++i) 
//    {
//      for(uint32_t j=0; j<centroids.cols()-1;++j) 
//        out << centroids(i,j)<<" ";
//      out << centroids(i,centroids.cols()-1)<<std::endl;
//    }
//    for(uint32_t i=0; i<concentrations.size()-1;++i) 
//      out << concentrations(i) << " ";
//    out << concentrations(concentrations.size()-1) << std::endl;
//    for(uint32_t i=0; i<proportions.size()-1;++i) 
//      out << proportions(i) << " ";
//    out << proportions(proportions.size()-1) << std::endl;
//    out.close();
  }
  std::cout<<cudaDeviceReset()<<std::endl;
  return (0);
}
