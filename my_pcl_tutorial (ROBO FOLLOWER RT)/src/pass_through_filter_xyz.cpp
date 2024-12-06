#include <ros/ros.h> // ROS core functionalities
#include <ros/console.h> // ROS logging
// PCL (Point Cloud Library) specific includes
#include <sensor_msgs/PointCloud2.h> // ROS message type for point cloud data
#include <pcl_conversions/pcl_conversions.h> // Conversions between PCL and ROS data types
#include <pcl/point_cloud.h> // Core data type for point clouds
#include <pcl/point_types.h> // Common point types like pcl::PointXYZ
#include <pcl/conversions.h> // Additional conversions within PCL
#include <pcl_ros/point_cloud.h> // Integration of PCL with ROS
#include <iostream> // Standard input/output library
#include <pcl/filters/passthrough.h> // For pass-through filtering
#include <pcl/io/pcd_io.h> // Input/output operations for PCD files

ros::Publisher pub; // ROS publisher to publish filtered point cloud data

// Callback function to process incoming point cloud data
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Container for the original and filtered point cloud data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; // Original point cloud
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud); // Constant pointer to the original cloud
  pcl::PCLPointCloud2 cloud_filtered; // Z-axis filtered point cloud

  // Convert the incoming ROS PointCloud2 message to a PCL PointCloud2 object
  pcl_conversions::toPCL(*input, *cloud); // Conversion from ROS to PCL format

  // Apply a pass-through filter on the Z-axis
  pcl::PassThrough<pcl::PCLPointCloud2> pass; // Z-axis filter
  pass.setInputCloud(cloudPtr); // Set the input cloud
  pass.setFilterFieldName("z"); // Filter along the Z-axis
  pass.setFilterLimits(0, 2); // Keep points with Z values between 0 and 2 meters
  pass.filter(cloud_filtered); // Apply the filter

  // Apply a pass-through filter on the X-axis
  pcl::PCLPointCloud2* cloud_2 = new pcl::PCLPointCloud2(cloud_filtered); // Create a new container for X-axis filtering
  pcl::PCLPointCloud2ConstPtr cloudPtr_2(cloud_2); // Constant pointer to the X-axis filtered cloud
  pcl::PCLPointCloud2 cloud_filtered_2; // X-axis filtered point cloud
  pcl::PassThrough<pcl::PCLPointCloud2> pass_2; // X-axis filter
  pass_2.setInputCloud(cloudPtr_2); // Set the input cloud
  pass_2.setFilterFieldName("x"); // Filter along the X-axis
  pass_2.setFilterLimits(-0.5, 0.5); // Keep points with X values between -0.5 and 0.5 meters
  pass_2.filter(cloud_filtered_2); // Apply the filter

  // Apply a pass-through filter on the Y-axis
  pcl::PCLPointCloud2* cloud_3 = new pcl::PCLPointCloud2(cloud_filtered_2); // Create a new container for Y-axis filtering
  pcl::PCLPointCloud2ConstPtr cloudPtr_3(cloud_3); // Constant pointer to the Y-axis filtered cloud
  pcl::PCLPointCloud2 cloud_filtered_3; // Y-axis filtered point cloud
  pcl::PassThrough<pcl::PCLPointCloud2> pass_3; // Y-axis filter
  pass_3.setInputCloud(cloudPtr_3); // Set the input cloud
  pass_3.setFilterFieldName("y"); // Filter along the Y-axis
  pass_3.setFilterLimits(-2, 2); // Keep points with Y values between -2 and 2 meters
  pass_3.filter(cloud_filtered_3); // Apply the filter

  // Convert the filtered PCL PointCloud2 object back to a ROS PointCloud2 message
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered_3, output); // Conversion from PCL to ROS format

  // Publish the filtered point cloud data
  pub.publish(output);
}

int main (int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "xyzpass"); // Node name: "xyzpass"
  ros::NodeHandle nh; // Create a node handle for communication

  // Subscribe to a topic to receive point cloud data
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 1, cloud_cb);

  // Advertise a topic to publish the filtered point cloud data
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Keep the node running and responsive to callbacks
  ros::spin();

  return 0; // Exit the program (this line won't be reached due to `ros::spin()`)
}
