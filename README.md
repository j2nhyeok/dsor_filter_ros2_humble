## Overview
![dsor_filter](https://github.com/j2nhyeok/dsor_filter_ros2_humble/blob/main/image.png)   

This repository is a ROS 2 Humble port of the original dsor_filter developed for ROS 1. The dsor_filter is designed to **remove snow points from LiDAR point cloud** data using a dynamic statistical outlier removal (DSOR) approach.

This package offers both:
- A command-line utility for processing .pcd files.
- A ROS 2 node that subscribes to point cloud topics and publishes filtered results.*(ongoing)*

For the original implementation and theory, please refer to the [original dsor_filter repository](https://github.com/assasinXL/dsor_filter)  .

Special thanks to the original developers for their contributions.

## Key Parameters
- **mean_k**: Number of nearest neighbors considered for each point when computing the mean distance.
- **std_mul**: Standard deviation multiplier used to determine the filtering threshold. Lower values result in stricter filtering.
- **range_mul**: Unique to DSOR – adjusts filtering aggressiveness based on the point's range from the sensor. Lower values remove distant points more easily.



## Setup

```
# Clone the repository
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/j2nhyeok/dsor_filter_ros2_humble.git

# Build the workspace
cd ~/colcon_ws
colcon build --packages-select dsor_filter

# Source the workspace
source install/setup.bash
```

## Usage
### 1. Filter a Single PCD File
The test_pcd executable applies the DSOR filter to a specified `.pcd` file.</br>
**usage:**<br/>
**Input**: input.pcd</br>
**Output**: output.pcd</br>
```
# Run with default parameters (k=3, std=0.05, range_mul=0.05)
ros2 run dsor_filter test_pcd <input.pcd> <output.pcd>
# Run with custom parameters
ros2 run dsor_filter test_pcd <input.pcd> <output.pcd> <mean_k> <std_mul> <range_mul>
# Example:
ros2 run dsor_filter test_pcd test.pcd test_filtered.pcd 3 0.01 0.01
```

### 2. Run as a ROS 2 Node
The DSOR filter can also be used as a ROS 2 node (example_node) to filter incoming point clouds in real time.

**Node behavior:**<br/>
- Subscribes to: cloud_in (sensor_msgs/msg/PointCloud2)<br/>
- Publishes to: filtered_cloud<br/>

**Run the Node:**

```
# Run with default parameters (k=5, std=0.01, range_mul=0.05)
ros2 run dsor_filter example_node

# Run with custom parameters
ros2 run dsor_filter example_node 3 0.02 0.02
```
**Bag File Integration & Visualization**:
<br/>To test the DSOR node with a recorded bag file:

```
ros2 bag play my_lidar --remap /points_raw:=cloud_in

```
You can then visualize the filtered result by subscribing to the filtered_cloud topic in RViz2.
