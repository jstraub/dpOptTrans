/* Copyright (c) 2016, Julian Straub <jstraub@csail.mit.edu> Licensed
 * under the MIT license. See the license file LICENSE.
 */
#pragma once
#include <Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

#include <bbTrans/vmf.h>
#include <bbTrans/normal.h>

using namespace bb;

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

//bool ComputevMFMMfromPC(const pcl::PointCloud<pcl::PointXYZRGBNormal>&
//    pc, const CfgRtDDPvMF& cfg, std::vector<vMF<3>>& vmfs);
//
//bool ComputeGMMfromPC(pcl::PointCloud<pcl::PointXYZRGBNormal>&
//    pc, const CfgRtDDPvMF& cfg, std::vector<Normal<3>>& gmm,
//    bool colorLables);

void ShufflePc(pcl::PointCloud<pcl::PointXYZRGBNormal>& pc);
void ComputeAreaWeightsPc(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcIn);
void ComputeAreaWeightsPcRadius(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcIn,
    float scale);

void DisplayPcs(const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcA, 
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcB, 
  const Eigen::Quaterniond& q_star, const Eigen::Vector3d& t_star,
  float scale);

double ComputeClosestPointEucledianCost(
    pcl::PointCloud<pcl::PointXYZRGBNormal>& pcA,
    pcl::PointCloud<pcl::PointXYZRGBNormal>& pcB,
    const Eigen::Matrix3d* R, const Eigen::Vector3d* t);

