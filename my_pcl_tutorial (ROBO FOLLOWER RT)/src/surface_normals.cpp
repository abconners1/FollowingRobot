#include <ros/ros.h> // ROS core functionalities
#include <ros/console.h> // ROS logging utilities
#include <iostream> // Standard input/output
#include <sensor_msgs/PointCloud2.h> // ROS message type for point cloud data
// PCL (Point Cloud Library) specific includes
#include <pcl_conversions/pcl_conversions.h> // Conversions between ROS and PCL
#include <pcl/point_cloud.h> // Core data type for PCL point clouds
#include <pcl/point_types.h> // Common PCL point types like pcl::PointXYZ
#include <pcl/conversions.h> // Additional conversions within PCL
#include <pcl/filters/voxel_grid.h> // Voxel grid filtering (not used in this code)
#include <pcl_ros/point_cloud.h> // Integration of PCL with ROS
#include <pcl/filters/passthrough.h> // Pass-through filtering (not used here)
#include <pcl/io/pcd_io.h> // Input/output operations for PCD files
#include <pcl/features/normal_3d.h> // To estimate normals of a point cloud
// #include <pcl/visualization/pcl_visualizer.h> // For visualization (commented out)

ros::Publisher pub; // ROS publisher for processed point cloud data

/**
 * Callback function to process incoming point cloud data.
 * Computes the centroid and normals of the input point cloud.
 *
 * @param input The input point cloud message of type sensor_msgs::PointCloud2.
 */
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the ROS PointCloud2 message to a PCL PointCloud<pcl::PointXYZ>
  pcl::PointCloud<pcl::PointXYZ> cloud; 
  pcl::fromROSMsg(*input, cloud); // Conversion from ROS message to PCL format

  // Placeholder for the centroid (X, Y, Z)
  Eigen::Vector4f xyz_centroid;

  // Compute the centroid of the point cloud
  compute3DCentroid(cloud, xyz_centroid);

  // Log the centroid for debugging
  ROS_INFO_STREAM("The Centroid is " << xyz_centroid);

  // Create an object for normal estimation and pass the input cloud to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud.makeShared()); // Use shared pointer for compatibility with PCL

  // Create a KD-tree structure for neighborhood searches
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree); // Set the search method to the KD-tree

  // Container for the computed normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  // Set the radius for neighborhood searches (3 cm)
  ne.setRadiusSearch(0.03);

  // Compute the normals for the point cloud
  ne.compute(*cloud_normals);

  // Uncomment below for debugging normals (if needed)
  /*
  std::cerr << "normals: " << cloud_normals->points[0].normal_x << " " 
                            << cloud_normals->points[0].normal_y << " "
                            << cloud_normals->points[0].normal_z << std::endl;
  */

  // Note: cloud_normals->points.size() should match the size of input cloud->points.size()
}

int main (int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "norms"); // Node name: "norms"
  ros::NodeHandle nh; // Create a node handle for communication

  // Subscribe to the input point cloud topic
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 1, cloud_cb);
  // Alternative subscriptions (commented out)
  // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, cloud_cb);
  // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud_in", 1, cloud_cb);

  // Advertise an output topic for processed point clouds (not used in this code)
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Enter a loop and process callbacks as new data arrives
  // ROS_INFO("Before spin");
  ros::spin(); // Blocks until the node is shut down
  // ROS_INFO("After spin");
}
