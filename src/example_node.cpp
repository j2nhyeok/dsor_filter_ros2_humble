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
  DSORFilter() : Node("dsor_filter_node") {
    // 기본 파라미터 선언 및 기본값 설정
    this->declare_parameter("k", 5);
    this->declare_parameter("std", 0.2);
    this->declare_parameter("range_mul", 0.2);

    // 기본값으로 파라미터 읽기
    this->get_parameter("k", k_);
    this->get_parameter("std", std_);
    this->get_parameter("range_mul", range_mul_);

    RCLCPP_INFO(this->get_logger(), "Initial parameters: k=%d, std=%.2f, range_mul=%.2f", k_, std_, range_mul_);

    // Publisher와 Subscriber 생성
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_in", 10, std::bind(&DSORFilter::cloud_callback, this, _1));
  }

  // 파라미터 오버라이드를 위한 public setter 함수 (옵션)
  void override_parameters(int k, float s, float r) {
    k_ = k;
    std_ = s;
    range_mul_ = r;
    this->set_parameter(rclcpp::Parameter("k", k_));
    this->set_parameter(rclcpp::Parameter("std", std_));
    this->set_parameter(rclcpp::Parameter("range_mul", range_mul_));
    RCLCPP_INFO(this->get_logger(), "Overridden parameters: k=%d, std=%.2f, range_mul=%.2f", k_, std_, range_mul_);
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<PointT>::Ptr input_cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *input_cloud);

    auto cloud_filtered = dsor(input_cloud, k_, std_, range_mul_, false);

    if (cloud_filtered->empty()) {
      RCLCPP_WARN(this->get_logger(), "Filtered cloud is empty. Skipping publish.");
      return;
    }
    
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
  auto node = std::make_shared<DSORFilter>();

  // 커맨드라인 인자가 제공되면 파라미터 오버라이드 (argv[1]=k, argv[2]=std, argv[3]=range_mul)
  if (argc >= 4) {
    try {
      int new_k = std::stoi(argv[1]);
      float new_std = std::stof(argv[2]);
      float new_range_mul = std::stof(argv[3]);
      node->override_parameters(new_k, new_std, new_range_mul);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node->get_logger(), "Failed to parse command line arguments: %s", e.what());
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "Using default parameters.");
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
