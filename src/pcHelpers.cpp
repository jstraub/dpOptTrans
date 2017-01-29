/* Copyright (c) 2016, Julian Straub <jstraub@csail.mit.edu> Licensed
 * under the MIT license. See the license file LICENSE.
 */
#include <dpOptTrans/pcHelpers.h>
//#include <dpMMlowVar/ddpmeansCUDA.hpp>
//
//bool ComputevMFMMfromPC(const pcl::PointCloud<pcl::PointXYZRGBNormal>&
//    pc, const CfgRtDDPvMF& cfg, std::vector<vMF<3>>& vmfs) {
//  // take 3 values (x,y,z of normal) with an offset of 4 values (x,y,z
//  // and one float which is undefined) and the step is 12 (4 for xyz, 4
//  // for normal xyz and 4 for curvature and rgb).
//  auto n_map = pc.getMatrixXfMap(3, 12, 4); // this works for PointXYZRGBNormal
//  boost::shared_ptr<Eigen::MatrixXf> n(new Eigen::MatrixXf(n_map));
////  std::cout << "normals: " << std::endl << n->rows() 
////    << "x" << n->cols() << std::endl;
//  // Setup the DPvMF clusterer.
//  shared_ptr<jsc::ClDataGpuf> cld(new jsc::ClDataGpuf(n,0));
//  dplv::DDPMeansCUDA<float,dplv::Spherical<float>> pddpvmf(cld,
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
//    if (counts(k) > 10) {
//      taus(k) = bb::vMF<3>::MLEstimateTau(xSum.col(k),
//          xSum.col(k).cast<double>()/xSum.col(k).norm(), ws(k));
//    }
//  
//  for(uint32_t k=0; k<K; ++k)
//    if (counts(k) > 10) {
//      vmfs.push_back(bb::vMF<3>(xSum.col(k).cast<double>()/xSum.col(k).norm(),
//            taus(k), pis(k)));
//    }
//  return true;
//}
//
//bool ComputeGMMfromPC(pcl::PointCloud<pcl::PointXYZRGBNormal>&
//    pc, const CfgRtDDPvMF& cfg, std::vector<Normal<3>>& gmm,
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
//  std::cout << "lambda=" << cfg.lambda << " K=" << K << std::endl;
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
//  const double maxEvFactor = 1e-2;
//  for(uint32_t k=0; k<K; ++k)
////    if (pis(k) > 0.) {
//    if (counts(k) > 10) {
//      Matrix3d cov = Ss[k]/float(ws(k));
//      Eigen::SelfAdjointEigenSolver<Matrix3d> eig(cov);
//      Eigen::Vector3d e = eig.eigenvalues();
//      uint32_t iMax = 0;
//      double eMax = e.maxCoeff(&iMax);
////      std::cout << cov << std::endl;
//      bool regularized = false;
//      for (uint32_t i=0; i<3; ++i)
//        if (i!=iMax && eMax*maxEvFactor > e(i)) {
//          std::cout << "small eigenvalue: " << e(i) << " replaced by " 
//            << eMax*maxEvFactor << std::endl;
//          e(i) = eMax*maxEvFactor;
//          regularized = true;
//        }
//      if (regularized) {
//        Eigen::Matrix3d V = eig.eigenvectors();
//        cov = V*e.asDiagonal()*V.inverse();
//      }
////      std::cout << cov << std::endl;
//      gmm.push_back(bb::Normal<3>(centroids.col(k).cast<double>(),
//            cov, pis(k)));
////        Ss[k]/float(ws(k)), pis(k)));
//    }
//  return true;
//}

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

void ComputeAreaWeightsPcRadius(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcIn, 
    float scale) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcXYZ(new
      pcl::PointCloud<pcl::PointXYZ>(pcIn.size(),1));
  pcXYZ->getMatrixXfMap(3,4,0) = pcIn.getMatrixXfMap(3, 12, 0);

  pcl::search::KdTree<pcl::PointXYZ> kd; 
  kd.setInputCloud(pcXYZ);

  std::vector<int> k_ids(9);
  std::vector<float> k_sqr_dists(9);
  for (uint32_t i=0; i<pcIn.size(); ++i) {
    int k_found = kd.radiusSearch(pcXYZ->at(i), scale, k_ids, k_sqr_dists);
    if (k_found > 0) {
      std::sort(k_sqr_dists.begin(), k_sqr_dists.begin()+k_found);
      float median_sqr_dist = k_sqr_dists[k_found/2];
      if (k_found%2 == 0)
        median_sqr_dist = (k_sqr_dists[k_found/2-1]+k_sqr_dists[k_found/2])*0.5;
      float scale_2d = median_sqr_dist*M_PI; // in m^2
      // Abuse curvature entry for the scale.
      pcIn.at(i).curvature = scale_2d;
    } else{
      pcIn.at(i).curvature = scale*0.01;
    }
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
