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

using namespace std; // Simplifies standard library usage

ros::Publisher pub; // ROS publisher to publish processed data

// Callback function to process incoming point cloud data
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the ROS PointCloud2 message to a PCL point cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*input, cloud);

  // Variables for storing segmentation results
  pcl::ModelCoefficients coefficients; // Coefficients of the segmented model
  pcl::PointIndices inliers; // Indices of points that belong to the model

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // Configure segmentation for plane detection
  seg.setOptimizeCoefficients(true); // Optimize the model coefficients
  seg.setModelType(pcl::SACMODEL_PLANE); // Model type: Plane
  seg.setMethodType(pcl::SAC_RANSAC); // Segmentation method: RANSAC
  seg.setDistanceThreshold(0.01); // Maximum allowable distance for a point to be considered an inlier

  // Set the input cloud for segmentation
  seg.setInputCloud(cloud.makeShared()); // Convert to shared pointer for compatibility
  seg.segment(inliers, coefficients); // Perform segmentation and store results in `inliers` and `coefficients`

  // Publish the model coefficients
  coefficients.values[1] = 45; // Example modification to the coefficients
  cout << "First coefficient is " << coefficients.values[1] << "\n";

  // Convert the coefficients to ROS message format
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);

  // Publish the coefficients
  pub.publish(ros_coefficients);
}

int main (int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "plane"); // Node name: "plane"
  ros::NodeHandle nh; // Create a node handle for communication

  // Subscribe to a topic to receive point cloud data
  ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_cb);

  // Advertise a topic to publish the segmentation results
  pub = nh.advertise<pcl_msgs::ModelCoefficients>("output", 1);

  // Keep the node running and responsive to callbacks
  ros::spin();

  return 0; // Exit the program (this line won’t be reached due to `ros::spin()`)
}
