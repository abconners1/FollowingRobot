#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
//specific to cylinder segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>


using namespace std;

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud; // vs. pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, cloud);

  pcl::ModelCoefficients coefficients; // vs. pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices inliers; // vs   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  //pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; //another version of cylinder tutorial --doesnt work
  
  //=============PLANE================================
  // Optional
  seg.setOptimizeCoefficients (true);
  // seg.setRadiusLimits(.5,1);
    // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  //=====================================================

  //===========CYLINDER, DOESNT WORK======================
 // Create the segmentation object for cylinder segmentation and set all the parameters
/*  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);*/
  //======================================================

  seg.setInputCloud (cloud.makeShared ()); // vs. seg.setInputCloud (cloud); I wonder why ROS likes this as a shared pointer?
  seg.segment (inliers, coefficients); // vs. seg.segment (*inliers, *coefficients);
  
  // Publish the model coefficients
  coefficients.values[1] = 45;
  cout << " first coeff is " << coefficients.values[1] << "\n";


  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub.publish (ros_coefficients);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "plane");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe ("cloud_in", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

  // Spin
  ros::spin ();
}