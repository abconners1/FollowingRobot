#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
// ROS_INFO( "callback start" );
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; // vs. pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud); // needed for "sor.setInputCloud (cloudPtr);" because it needs a pointer
  pcl::PCLPointCloud2 cloud_filtered; //vs.  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  
   // Convert to PCL data type
  pcl_conversions::toPCL(*input, *cloud); //"toPCL" muct copy entire point cloud because its input is a "const"

  // Create the z-filtering object
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 2); 
  // pass.setFilterLimits (-2, 2); 
  //pass.setFilterLimitsNegative (true);
  pass.filter (cloud_filtered);


  // Create the x-filtering object
  pcl::PCLPointCloud2* cloud_2 = new pcl::PCLPointCloud2(cloud_filtered); 
  pcl::PCLPointCloud2ConstPtr cloudPtr_2(cloud_2);
  pcl::PCLPointCloud2 cloud_filtered_2;   
  pcl::PassThrough<pcl::PCLPointCloud2> pass_2;
  pass_2.setInputCloud (cloudPtr_2);
  pass_2.setFilterFieldName ("x");
  pass_2.setFilterLimits (-.5, 0.5); 
  // pass_2.setFilterLimits (-.5, 1.5); 
  //pass.setFilterLimitsNegative (true);
  pass_2.filter (cloud_filtered_2);


 // Create the y-filtering object
  pcl::PCLPointCloud2* cloud_3 = new pcl::PCLPointCloud2(cloud_filtered_2); 
  pcl::PCLPointCloud2ConstPtr cloudPtr_3(cloud_3);
  pcl::PCLPointCloud2 cloud_filtered_3;   
  pcl::PassThrough<pcl::PCLPointCloud2> pass_3;
  pass_3.setInputCloud (cloudPtr_3);
  pass_3.setFilterFieldName ("y");
  pass_3.setFilterLimits (-2, 2); 
  // pass_3.setFilterLimits (-3, 0); 
  //pass.setFilterLimitsNegative (true);
  pass_2.filter (cloud_filtered_3);


  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered_3, output); // "fromPCL" can be used but it will copy entire pointcloud

  // Publish the data
  pub.publish (output);
  // ROS_INFO( "callback done" );
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "xyzpass");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("velodyne_points", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("camera/depth_registered/points", 1, cloud_cb);
    // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("cloud_in", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  // ROS_INFO( "before spin" );
  ros::spin ();
  // ROS_INFO( "after spin" );

}