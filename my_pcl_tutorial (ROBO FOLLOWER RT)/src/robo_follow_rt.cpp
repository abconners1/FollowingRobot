#include <ros/ros.h> // ROS core functionalities
#include <ros/console.h> // ROS logging
#include <iostream> // Standard input/output
#include <sensor_msgs/PointCloud2.h> // ROS PointCloud2 message type
// PCL (Point Cloud Library) specific includes
#include <pcl_conversions/pcl_conversions.h> // Conversions between PCL and ROS
#include <pcl/point_cloud.h> // Core data type for point clouds
#include <pcl/point_types.h> // Common point types like pcl::PointXYZ
#include <pcl/conversions.h> // Additional conversions within PCL
#include <pcl/filters/voxel_grid.h> // Voxel grid filtering
#include <pcl_ros/point_cloud.h> // Integration of PCL with ROS
#include <pcl/filters/passthrough.h> // Pass-through filtering
#include <pcl/io/pcd_io.h> // Input/output for PCD files
#include <pcl/features/normal_3d.h> // Compute surface normals

#include <geometry_msgs/Twist.h> // Message type for velocity commands

/*#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <kobuki_msgs/BumperEvent.h> // Message type in: Bumper
#include <geometry_msgs/Twist.h> // Message type out: Velocity
#include <ctime> // Needed for time() in random number generator
#include <unistd.h> // For sleep() functions
*/

using namespace std;

//--------------Class------------------------------
/**
 * Class `followCentroid` defines the robot follower behavior. It subscribes to
 * a point cloud, computes the centroid of points, and publishes velocity commands
 * to align and follow the target.
 */
class followCentroid
{
  ros::Subscriber sb_cloud; /**< Subscriber for the input point cloud. */
  ros::Publisher pb_velocity; /**< Publisher for velocity commands. */
  ros::NodeHandle nh; /**< ROS node handle for communication. */
  double lat_dead_band; /**< Minimum lateral displacement of the target for robot to rotate. */
  double follow_dist_max; /**< Maximum allowable follow distance. */
  double follow_dist_min; /**< Minimum allowable follow distance. */

  // Callback function for processing incoming point cloud data
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr input);

public:
    followCentroid(); // Constructor
    // void pub_vel(); // Uncommented to potentially add a velocity publisher
};

//----------------Constructor----------------------------
/**
 * Constructor initializes default values for the follower's parameters and sets up
 * publishers and subscribers for velocity and point cloud data.
 */
followCentroid::followCentroid()
{
  lat_dead_band = 0.25; // Minimum lateral displacement of the target for robot rotation
  follow_dist_max = 2.0; // Maximum allowable follow distance
  follow_dist_min = 1.0; // Minimum allowable follow distance

  // Advertise velocity commands topic
  pb_velocity = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

  // Subscribe to point cloud topic
  sb_cloud = nh.subscribe("output", 1, &followCentroid::cloud_cb, this);
}

//------------ Callback Function to Adjust Robot Position---------------
/**
 * Processes the incoming point cloud to compute the centroid and generates velocity
 * commands for aligning and following the target.
 */
void followCentroid::cloud_cb (const sensor_msgs::PointCloud2ConstPtr input)
{
  ROS_INFO("Entered the callback function!");

  geometry_msgs::Twist vel; // Initialize velocity command message

  // Convert ROS PointCloud2 message to PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*input, cloud);

  // Placeholder for the XYZ centroid of the target
  Eigen::Vector4f xyz_centroid;

  // Compute the centroid of the point cloud
  compute3DCentroid(cloud, xyz_centroid);

  // Log the centroid values for debugging
  ROS_INFO_STREAM("Centroid: " << xyz_centroid);
  ROS_INFO_STREAM("X: " << xyz_centroid[0]);
  ROS_INFO_STREAM("Y: " << xyz_centroid[1]);
  ROS_INFO_STREAM("Z: " << xyz_centroid[2]);

  // Generate rotational velocity based on the lateral displacement of the centroid
  if (xyz_centroid[0] > 0.5 * lat_dead_band) 
  {
    vel.angular.z = -0.5; // Rotate left
  }
  else if (xyz_centroid[0] < -0.5 * lat_dead_band)
  {
    vel.angular.z = 0.5; // Rotate right
  }
  else
  {
    vel.angular.z = 0.0; // No rotation
  }

  // Generate linear velocity based on the depth (Z-axis) of the centroid
  if (xyz_centroid[2] > follow_dist_max)
  {
    vel.linear.x = 0.0; // Stop moving forward
  }
  else if (xyz_centroid[2] > follow_dist_min)
  {
    vel.linear.x = 0.25; // Move forward at a constant speed
  }
  else 
  {
    vel.linear.x = 0.0; // Stop moving forward
  }

  // Publish the velocity command
  pb_velocity.publish(vel);

  ROS_INFO("Exiting the callback function.");
}

//----------------Publishing Function-----------------------
/*void followCentroid::pub_vel()
{
  // geometry_msgs::Twist vel;
  vel.linear.x = command_velocity_l;
  vel.angular.z = command_velocity_a;
  pb_velocity.publish(vel);
  ROS_INFO("Publishing velocity command!");  
}
*/

int main (int argc, char** argv)
{
  ROS_INFO("Initializing the robot follower node...");
  ros::init(argc, argv, "follow"); // Initialize ROS node
  followCentroid robot; // Instantiate the follower object

  ros::Rate loop_rate(10); // Set the loop rate to 10 Hz

  while (ros::ok) // Main loop
  {
    ROS_INFO("Executing spinOnce...");
    ros::spinOnce(); // Process incoming messages
    loop_rate.sleep(); // Sleep to maintain the loop rate
    ROS_INFO("Loop iteration complete.");
  }
}

/*// Create a ROS subscriber for the input point cloud
ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("output", 1, cloud_cb);
// ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("camera/depth_registered/points", 1, cloud_cb);
// ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("velodyne_points", 1, cloud_cb);
// ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("cloud_in", 1, cloud_cb);

// Create a ROS publisher for the output point cloud
pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

// Spin
// ROS_INFO("Before spin");
ros::spin();
// ROS_INFO("After spin");
*/
