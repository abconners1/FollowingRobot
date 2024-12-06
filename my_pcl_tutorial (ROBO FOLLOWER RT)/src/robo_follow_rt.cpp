#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <geometry_msgs/Twist.h> //Message Type out : Velocity

/*#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <kobuki_msgs/BumperEvent.h> //Message type in : Bumper
#include <geometry_msgs/Twist.h> //Message Type out : Velocity
#include <ctime> //needed for time() in random number generator
#include <unistd.h>*/


using namespace std;

//--------------Class------------------------------
class followCentroid
{
  ros::Subscriber sb_cloud;
  ros::Publisher pb_velocity;
  ros::NodeHandle nh;
  // double command_velocity_l;
  // double command_velocity_a; 
  double lat_dead_band; //minimun lateral displacement of a person for robot to move
  double follow_dist_max;
  double follow_dist_min;

  //class callback function
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr input);

  public:
    followCentroid();
    //void pub_vel();
 
};
//----------------Constructor----------------------------
followCentroid::followCentroid()
{
  // command_velocity_l = 0;
  // command_velocity_a = 0;
  lat_dead_band = .25; //minimun lateral displacement of a person for robot to move
  follow_dist_max = 2;
  follow_dist_min = 1;

  // nh.subscribe(topic_name,queue_size, pointer_to_callback_function);
  
  pb_velocity = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  
  sb_cloud = nh.subscribe("output", 1, &followCentroid::cloud_cb,this);

}

//------------ callback function to Correct Mobot Position---------------

void followCentroid::cloud_cb (const sensor_msgs::PointCloud2ConstPtr input)
{
  ROS_INFO("Hey Im in the Callback!");  
  geometry_msgs::Twist vel;
  
   // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud; 
  pcl::fromROSMsg (*input, cloud);

 // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
  Eigen::Vector4f xyz_centroid;

  // Estimate the XYZ centroid
  compute3DCentroid (cloud, xyz_centroid);

  ROS_INFO_STREAM("The Centroid is " << xyz_centroid);
  ROS_INFO_STREAM("X Centroid is " << xyz_centroid[0]);
  ROS_INFO_STREAM("Y Centroid is " << xyz_centroid[1]);
  ROS_INFO_STREAM("Z Centroid is " << xyz_centroid[2]);

  if(xyz_centroid[0] > 0.5*lat_dead_band) 
  {vel.angular.z = -0.5;}
  else if(xyz_centroid[0] < -0.5*lat_dead_band)
  {vel.angular.z = 0.5;}
  else
  {vel.angular.z = 0;}

  if (xyz_centroid[2] > follow_dist_max)
  {vel.linear.x = 0;}
  else if (xyz_centroid[2] > follow_dist_min)
  {vel.linear.x = 0.25;}
  else 
  {vel.linear.x = 0;}

  pb_velocity.publish(vel);

  ROS_INFO("end of call back");
}

//----------------Publishing Function-----------------------
/*void followCentroid::pub_vel()
{
  // geometry_msgs::Twist vel;
  vel.linear.x = command_velocity_l;
  vel.angular.z = command_velocity_a;
  pb_velocity.publish(vel);
  ROS_INFO("Im in pub_vel");  
}
*/

int main (int argc, char** argv)
{
  ROS_INFO(" initializing robot follow node");
  ros::init (argc, argv, "follow");
  followCentroid robot;

  ros::Rate loop_rate(10); //send message at 10 hz

  while(ros::ok)
    {
      // robot.pub_vel(); //call function that sets path to forward
      ROS_INFO("before spinOnce");
      //while(ros::ok)
      //{
      ros::spinOnce();  //takes your newest message when cloud msg comes in
      //sleep(1);
      //}
      
      //ros::spin();
      ROS_INFO("After spinOnce");
      
      ROS_INFO("start sleep");
      loop_rate.sleep();
      ROS_INFO("end sleep");
    }
}  

  /*// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("output", 1, cloud_cb);
  // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("camera/depth_registered/points", 1, cloud_cb);
    // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("velodyne_points", 1, cloud_cb);
  // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("cloud_in", 1, cloud_cb);

  Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  // ROS_INFO( "before spin" );
  ros::spin ();
  // ROS_INFO( "after spin" );
*/
