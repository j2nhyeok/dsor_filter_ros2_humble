#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "dsor.hpp"

using PointT = pcl::PointXYZ;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "Usage: ./pcd_dsor_filter input.pcd output.pcd [mean_k] [std_mul] [range_mul]" << std::endl;
    return -1;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());

  if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1) {
    std::cerr << "Couldn't read input PCD file " << argv[1] << std::endl;
    return -1;
  }

  std::cout << "Loaded " << cloud->size() << " points from " << argv[1] << std::endl;

  // 기본값 설정
  int mean_k = 3;
  float std_mul = 0.05;
  float range_mul = 0.05;

  // 추가 인자가 주어지면 파라미터로 사용
  if (argc >= 6) {
    mean_k = std::stoi(argv[3]);
    std_mul = std::stof(argv[4]);
    range_mul = std::stof(argv[5]);
  } else {
    std::cout << "Using default parameters: mean_k=" << mean_k 
              << ", std_mul=" << std_mul 
              << ", range_mul=" << range_mul << std::endl;
  }

  filtered_cloud = dsor(cloud, mean_k, std_mul, range_mul, false);

  std::cout << "Filtered cloud has " << filtered_cloud->size() << " points" << std::endl;

  pcl::io::savePCDFileASCII(argv[2], *filtered_cloud);
  std::cout << "Saved filtered point cloud to " << argv[2] << std::endl;

  return 0;
}
