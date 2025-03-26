#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "dsor.hpp"

using std::placeholders::_1;
using PointT = pcl::PointXYZ;


class DSORFilter : public rclcpp::Node {
public:
  // Adjust the DSOR parameter values as needed. 
  DSORFilter() : Node("dsor_filter_node") {
    this->declare_parameter("k", 5);
    this->declare_parameter("std", 0.01);
    this->declare_parameter("range_mul", 0.05);

    this->get_parameter("k", k_);
    this->get_parameter("std", std_);
    this->get_parameter("range_mul", range_mul_);

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_in", 10, std::bind(&DSORFilter::cloud_callback, this, _1));
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<PointT>::Ptr input_cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *input_cloud);

    auto cloud_filtered = dsor(input_cloud, k_, std_, range_mul_, false);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header = msg->header;
    cloud_pub_->publish(output);
  }

  int k_;
  float std_, range_mul_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DSORFilter>());
  rclcpp::shutdown();
  return 0;
}
