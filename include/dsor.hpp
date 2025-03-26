#ifndef INCLUDE_DSOR_HPP_
#define INCLUDE_DSOR_HPP_

#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

using PointT = pcl::PointXYZ;

pcl::PointCloud<PointT>::Ptr dsor(pcl::PointCloud<PointT>::Ptr &input_cloud,
                                  int mean_k, float std_mul, float range_mul,
                                  bool negative) {
  pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr negative_cloud(new pcl::PointCloud<PointT>);

  pcl::KdTreeFLANN<PointT> kd_tree;
  kd_tree.setInputCloud(input_cloud);

  std::vector<int> pointIdxNKNSearch(mean_k);
  std::vector<float> pointNKNSquaredDistance(mean_k);
  std::vector<float> mean_distances;

  for (auto it = input_cloud->begin(); it != input_cloud->end(); ++it) {
    kd_tree.nearestKSearch(*it, mean_k, pointIdxNKNSearch, pointNKNSquaredDistance);
    double dist_sum = 0;
    for (int j = 1; j < mean_k; ++j) {
      dist_sum += sqrt(pointNKNSquaredDistance[j]);
    }
    mean_distances.push_back(static_cast<float>(dist_sum / (mean_k - 1)));
  }

  double sum = 0, sq_sum = 0;
  for (auto dist : mean_distances) {
    sum += dist;
    sq_sum += dist * dist;
  }
  double mean = sum / mean_distances.size();
  double variance = (sq_sum - sum * sum / mean_distances.size()) / (mean_distances.size() - 1);
  double stddev = sqrt(variance);
  double distance_threshold = (mean + std_mul * stddev);

  int i = 0;
  for (auto it = input_cloud->begin(); it != input_cloud->end(); ++it, ++i) {
    float range = sqrt(it->x * it->x + it->y * it->y + it->z * it->z);
    double dynamic_threshold = distance_threshold * range_mul * range;
    if (mean_distances[i] < dynamic_threshold) {
      filtered_cloud->push_back(*it);
    } else {
      negative_cloud->push_back(*it);
    }
  }

  return negative ? negative_cloud : filtered_cloud;
}

#endif  // INCLUDE_DSOR_HPP_
