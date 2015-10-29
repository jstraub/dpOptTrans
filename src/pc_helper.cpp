/* Copyright (c) 2015, Julian Straub <jstraub@csail.mit.edu> Licensed
 * under the MIT license. See the license file LICENSE.
 */

#include "dpvMFoptRot/pc_helper.h"

void ComputeAreaWeightsPc(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcIn) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcXYZ(new
      pcl::PointCloud<pcl::PointXYZ>(pcIn.size(),1));
  pcXYZ->getMatrixXfMap(3,4,0) = pcIn.getMatrixXfMap(3, 12, 0);

  pcl::search::KdTree<pcl::PointXYZ> kd; 
  kd.setInputCloud(pcXYZ);

  std::vector<int> k_ids(9);
  std::vector<float> k_sqr_dists(9);
  for (uint32_t i=0; i<pcIn.size(); ++i) {
    int k_found = kd.nearestKSearch(pcXYZ->at(i), 9, k_ids, k_sqr_dists);
    std::sort(k_sqr_dists.begin(), k_sqr_dists.begin()+k_found);
    float median_sqr_dist = k_sqr_dists[k_found/2];
    if (k_found%2 == 0)
      median_sqr_dist = (k_sqr_dists[k_found/2-1]+k_sqr_dists[k_found/2])*0.5;
    float scale_2d = median_sqr_dist*M_PI; // in m^2
    // Abuse curvature entry for the scale.
    pcIn.at(i).curvature = scale_2d;
  }
}

void ShufflePc(pcl::PointCloud<pcl::PointXYZRGBNormal>& pc) {
  // https://en.wikipedia.org/wiki/Fisher%E2%80%93Yates_shuffle
  std::mt19937 gen;
  for (uint32_t i=pc.size()-1; i > 0; --i) {
    std::uniform_int_distribution<uint32_t> u(0,i);
    uint32_t j = u(gen);
    // swap i and j;
    pcl::PointXYZRGBNormal p = pc.at(i);
    pc.at(i) = pc.at(j);
    pc.at(j) = p;
  }
}

double ComputeClosestPointEucledianCost(
    pcl::PointCloud<pcl::PointXYZRGBNormal>& pcA,
    pcl::PointCloud<pcl::PointXYZRGBNormal>& pcB,
    const Eigen::Matrix3d* R, const Eigen::Vector3d* t
    ) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcAxyz(new
      pcl::PointCloud<pcl::PointXYZ>(pcA.size(),1));
  pcAxyz->getMatrixXfMap(3,4,0) = pcA.getMatrixXfMap(3, 12, 0);

  pcl::search::KdTree<pcl::PointXYZ> kd; 
  kd.setInputCloud(pcAxyz);

  std::vector<int> k_ids(1);
  std::vector<float> k_sqr_dists(1);
  double sum_sq_dists = 0.;
  double W = 0.;
  for (uint32_t i=0; i<pcB.size(); i+=100) {
    pcl::PointXYZ pS;
    Eigen::Map<Eigen::Vector3f> p(&(pS.x));
    Eigen::Map<Eigen::Vector3f> pB(&(pcB.at(i).x));
    p = pB;
    if (R && t) {
      p = R->cast<float>().transpose() * (p-t->cast<float>());
    }
    int k_found = kd.nearestKSearch(pS, 1, k_ids, k_sqr_dists);
    double w = pcB.at(i).curvature;
    sum_sq_dists += w*k_sqr_dists[0];
    W += w;
  }
  return sqrt(sum_sq_dists / W);
}

void DisplayPcs(const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcA, 
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcB, 
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
    viewerPc->addCoordinateSystem (scale);

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
