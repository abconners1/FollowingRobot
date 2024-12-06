#include <ros/ros.h> // ROS core functionalities
// PCL (Point Cloud Library) specific includes
#include <sensor_msgs/PointCloud2.h> // ROS message type for point cloud data
#include <pcl_conversions/pcl_conversions.h> // Conversions between PCL and ROS data types
#include <pcl/point_cloud.h> // Core data type for point clouds
#include <pcl/point_types.h> // Common point types like pcl::PointXYZ
#include <pcl/conversions.h> // Additional conversions within PCL
#include <pcl/filters/voxel_grid.h> // For voxel grid filtering
#include <pcl_ros/point_cloud.h> // Integration of PCL with ROS
#include <iostream> // Standard input/output library
#include <pcl/filters/passthrough.h> // For pass-through filtering
#include <pcl/io/pcd_io.h> // Input/output operations for PCD files

ros::Publisher pub; // ROS publisher to publish filtered point cloud data

// Callback function to process incoming point cloud data
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Containers for the original and filtered point cloud data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; // Original point cloud
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud); // Constant pointer to the original point cloud
  pcl::PCLPointCloud2 cloud_filtered; // Filtered point cloud

  // Convert the incoming ROS PointCloud2 message to a PCL PointCloud2 object
  pcl_conversions::toPCL(*input, *cloud); // Conversion from ROS to PCL format

  // Create and configure a pass-through filter
  pcl::PassThrough<pcl::PCLPointCloud2> pass; // Pass-through filter object
  pass.setInputCloud(cloudPtr); // Set the input cloud
  pass.setFilterFieldName("z"); // Filter along the Z-axis
  pass.setFilterLimits(0.0, 0.5); // Keep points with Z values between 0.0 and 0.5 meters
  // pass.setFilterLimitsNegative(true); // Uncomment to invert the filter and remove points in the range
  pass.filter(cloud_filtered); // Apply the filter and store the result in `cloud_filtered`

  // Convert the filtered PCL PointCloud2 object back to a ROS PointCloud2 message
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output); // Conversion from PCL to ROS format

  // Publish the filtered point cloud data
  pub.publish(output);
}

int main (int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "zpass"); // Node name: "zpass"
  ros::NodeHandle nh; // Create a node handle for communication

  // Subscribe to a topic to receive point cloud data
  // The `cloud_cb` function will be called whenever new data is received
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud_in", 1, cloud_cb);

  // Advertise a topic to publish the filtered point cloud data
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Keep the node running and responsive to callbacks
  ros::spin();

  return 0; // Exit the program (this line won't be reached due to `ros::spin()`)
}
