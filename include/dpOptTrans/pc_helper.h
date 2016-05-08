/* Copyright (c) 2015, Julian Straub <jstraub@csail.mit.edu> Licensed
 * under the MIT license. See the license file LICENSE.
 */

#pragma once 
#include <random>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

void ComputeAreaWeightsPc(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcIn);

void DisplayPcs(const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcA, 
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcB, 
  const Eigen::Quaterniond& q_star, const Eigen::Vector3d& t_star,
  float scale);


void ShufflePc(pcl::PointCloud<pcl::PointXYZRGBNormal>& pc);
