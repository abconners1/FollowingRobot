#include <ros/ros.h> // ROS core functionalities
// PCL (Point Cloud Library) specific includes
#include <sensor_msgs/PointCloud2.h> // ROS message type for point cloud data
#include <pcl_conversions/pcl_conversions.h> // Conversions between PCL and ROS data types
#include <pcl/point_cloud.h> // Core data type for point clouds
#include <pcl/point_types.h> // Common point types like pcl::PointXYZ

ros::Publisher pub; // ROS publisher to publish processed point cloud data

// Callback function to process incoming point cloud data
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the output point cloud
  sensor_msgs::PointCloud2 output;

  // Do data processing here (currently, no modifications are made)
  output = *input; // Simply copy the input point cloud to the output container

  // Publish the processed (or unmodified) point cloud data
  pub.publish(output);
}

int main (int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "example"); // Node name: "example"
  ros::NodeHandle nh; // Create a node handle for communication

  // Subscribe to the "cloud_in" topic to receive point cloud data
  // The callback function `cloud_cb` will be invoked whenever new data is received
  ros::Subscriber sub = nh.subscribe("cloud_in", 1, cloud_cb);

  // Advertise a topic "output" to publish processed point cloud data
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Keep the node running and responsive to callbacks
  ros::spin();

  return 0; // Exit the program (this line won't be reached due to ros::spin())
}
