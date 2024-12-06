#include <ros/ros.h> // ROS core functionalities
#include <ros/console.h> // ROS console logging
#include <iostream> // Standard input/output library
#include <sensor_msgs/PointCloud2.h> // ROS message type for point cloud data

// PCL (Point Cloud Library) specific includes
#include <pcl_conversions/pcl_conversions.h> // Conversions between PCL and ROS data types
#include <pcl/point_cloud.h> // Core data type for point clouds
#include <pcl/point_types.h> // Common point types like pcl::PointXYZ
#include <pcl/conversions.h> // Additional conversions within PCL
#include <pcl/filters/voxel_grid.h> // For voxel grid filtering
#include <pcl_ros/point_cloud.h> // For PCL and ROS integration
#include <pcl/filters/passthrough.h> // For pass-through filtering
#include <pcl/io/pcd_io.h> // Input/output operations for PCD files
#include <pcl/features/normal_3d.h> // To estimate surface normals
// #include <pcl/visualization/pcl_visualizer.h> // Optional visualization (commented out)

ros::Publisher pub; // ROS publisher to publish processed point cloud data

// Callback function to process incoming point cloud data
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the ROS PointCloud2 message to a PCL point cloud
  pcl::PointCloud<pcl::PointXYZ> cloud; 
  pcl::fromROSMsg(*input, cloud);

  // Placeholder for the centroid (X, Y, Z) of the point cloud
  Eigen::Vector4f xyz_centroid;

  // Compute the centroid of the point cloud
  compute3DCentroid(cloud, xyz_centroid);

  // Log the centroid values to the console
  ROS_INFO_STREAM("The Centroid is " << xyz_centroid);

  // Uncomment the following block for normal estimation if needed
  /*
  // Create the normal estimation class and pass the input dataset
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud.makeShared());

  // Create an empty KD-tree representation for neighbor search
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);

  // Output dataset for normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors within a 3 cm radius
  ne.setRadiusSearch(0.03);

  // Compute the surface normals
  ne.compute(*cloud_normals);

  // Optionally log computed normals (example placeholder code)
  std::cerr << "normals: " << cloud_normals->points[0].normal_x << " " 
                            << cloud_normals->points[0].normal_y << " "
                            << cloud_normals->points[0].normal_z << std::endl;
  */
}

// Main function
int
main (int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "centroid"); // Node name: "centroid"
  ros::NodeHandle nh; // Create a node handle for communication

  // Subscribe to a topic to receive point cloud data
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("output", 1, cloud_cb);

  // Uncomment one of the following if using a different topic
  // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 1, cloud_cb);
  // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, cloud_cb);
  // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud_in", 1, cloud_cb);

  // Publisher setup (optional in this case, but uncomment if needed for output)
  // pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Keep the node running and responsive to callbacks
  ros::spin();
}
