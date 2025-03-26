#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "dsor.hpp"

using PointT = pcl::PointXYZ;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "Usage: ./pcd_dsor_filter input.pcd output.pcd" << std::endl;
    return -1;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());

  if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1) {
    std::cerr << "Couldn't read input PCD file " << argv[1] << std::endl;
    return -1;
  }

  std::cout << "Loaded " << cloud->size() << " points from " << argv[1] << std::endl;

  // "Adjust the DSOR parameter values as needed."
  int mean_k = 3;
  float std_mul = 0.08;
  float range_mul = 0.08;


  filtered_cloud = dsor(cloud, mean_k, std_mul, range_mul, false);

  std::cout << "Filtered cloud has " << filtered_cloud->size() << " points" << std::endl;

  pcl::io::savePCDFileASCII(argv[2], *filtered_cloud);
  std::cout << "Saved filtered point cloud to " << argv[2] << std::endl;

  return 0;
}
