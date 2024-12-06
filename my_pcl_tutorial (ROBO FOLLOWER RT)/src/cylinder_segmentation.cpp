#include <ros/ros.h> // ROS core functionalities
// PCL (Point Cloud Library) specific includes
#include <sensor_msgs/PointCloud2.h> // ROS message type for point cloud data
#include <pcl_conversions/pcl_conversions.h> // Conversions between PCL and ROS data types
// #include <pcl/ros/conversions.h> // Deprecated; replaced with pcl_conversions
#include <pcl/conversions.h> // Additional conversions within PCL
#include <pcl/point_cloud.h> // Core data type for point clouds
#include <pcl/point_types.h> // Common point types like pcl::PointXYZ
#include <pcl_ros/point_cloud.h> // Integration of PCL with ROS
#include <pcl/sample_consensus/model_types.h> // Model types for segmentation
#include <pcl/sample_consensus/method_types.h> // Methods for segmentation (e.g., RANSAC)
#include <pcl/segmentation/sac_segmentation.h> // Segmentation algorithms
#include <iostream> // Standard input/output library

// Specific to cylinder segmentation
#include <pcl/ModelCoefficients.h> // Represents model coefficients
#include <pcl/io/pcd_io.h> // Input/output operations for PCD files
#include <pcl/filters/extract_indices.h> // Extract points based on indices
#include <pcl/filters/passthrough.h> // Filter points based on a range of values
#include <pcl/features/normal_3d.h> // Compute surface normals

// Define PointT as pcl::PointXYZ for simplicity
typedef pcl::PointXYZ PointT;

using namespace std;

ros::Publisher pub; // ROS publisher to publish processed data

// Callback function to process incoming point cloud data
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the ROS PointCloud2 message to a PCL point cloud
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*input, *cloud);

  // Define variables for filtering and segmentation
  pcl::PassThrough<PointT> pass; // Pass-through filter
  pcl::NormalEstimation<PointT, pcl::Normal> ne; // Normal estimation
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; // Segmentation from normals
  pcl::ExtractIndices<PointT> extract; // Extract indices of filtered points
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients); // Plane coefficients
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices); // Plane inliers

  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Apply a pass-through filter to remove unwanted points (e.g., outside a range)
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z"); // Filter based on the Z-axis
  pass.setFilterLimits(0, 2); // Keep points with Z between 0 and 2 meters
  pass.filter(*cloud_filtered);
  std::cerr << "Filtered cloud has " << cloud_filtered->points.size() << " points." << std::endl;

  // Estimate normals for the filtered cloud
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>); // KD-tree for neighbor search
  ne.setInputCloud(cloud_filtered);
  ne.setSearchMethod(tree);
  ne.setKSearch(50); // Use 50 nearest neighbors for normal estimation
  ne.compute(*cloud_normals);

  // Perform planar segmentation using the estimated normals
  seg.setOptimizeCoefficients(true); // Optimize the coefficients
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE); // Model: plane with normals
  seg.setMethodType(pcl::SAC_RANSAC); // Method: RANSAC
  seg.setNormalDistanceWeight(0.1); // Weight of normals in distance computation
  seg.setMaxIterations(100); // Maximum iterations for RANSAC
  seg.setDistanceThreshold(0.03); // Distance threshold for inliers
  seg.setInputCloud(cloud_filtered);
  seg.setInputNormals(cloud_normals);
  seg.segment(*inliers_plane, *coefficients_plane); // Segment the plane

  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the inliers corresponding to the plane
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers_plane);
  extract.setNegative(false); // Keep the inliers
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
  extract.filter(*cloud_plane);
  std::cerr << "Plane cloud has " << cloud_plane->points.size() << " points." << std::endl;

  // Convert the plane cloud to ROS data type and publish it
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(*cloud_plane, output);
  pub.publish(output);
}

// Main function
int main (int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "cylinder"); // Node name: "cylinder"
  ros::NodeHandle nh; // Create a node handle for communication

  // Subscribe to a topic to receive point cloud data
  ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_cb);

  // Advertise a topic to publish the processed point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Keep the node running and responsive to callbacks
  ros::spin();
}
