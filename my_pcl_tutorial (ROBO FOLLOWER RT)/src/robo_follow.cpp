/*
 * LICENSE:
 * This code is licensed under Willow Garage, Inc., allowing redistribution and modification under specific conditions.
 * DISCLAIMERS:
 * - Provided "AS IS" without warranty of any kind.
 * - No liability for damages resulting from its use.
 */

#include <ros/ros.h> // ROS core functionalities
#include <pluginlib/class_list_macros.h> // Pluginlib for dynamic nodelet loading
#include <nodelet/nodelet.h> // Base class for nodelets
#include <geometry_msgs/Twist.h> // Message type for velocity commands
#include <sensor_msgs/Image.h> // Image message type for depth images
#include <visualization_msgs/Marker.h> // Marker visualization for RViz
#include <turtlebot_msgs/SetFollowState.h> // Custom service for enabling/disabling the follower
#include "dynamic_reconfigure/server.h" // Support for dynamic reconfiguration
#include "turtlebot_follower/FollowerConfig.h" // Custom configuration options
#include <depth_image_proc/depth_traits.h> // Helper utilities for depth image processing

namespace turtlebot_follower
{

// The Turtlebot follower nodelet class
/**
 * Processes depth images from a sensor, calculates the centroid of points within
 * a specified region, and publishes robot velocity commands to follow the target.
 */
class TurtlebotFollower : public nodelet::Nodelet
{
public:
  // Constructor initializes bounding box and speed scaling parameters
  TurtlebotFollower() : min_y_(0.1), max_y_(0.5), 
                        min_x_(-0.2), max_x_(0.2), 
                        max_z_(0.8), goal_z_(0.6), 
                        z_scale_(1.0), x_scale_(5.0) {}

  // Destructor cleans up the dynamic reconfigure server
  ~TurtlebotFollower()
  {
    delete config_srv_; // Prevent memory leaks
  }

private:
  // Parameters for bounding box dimensions and control
  double min_y_, max_y_; /**< Y-axis limits for the bounding box. */
  double min_x_, max_x_; /**< X-axis limits for the bounding box. */
  double max_z_; /**< Maximum depth (Z-axis) limit for detected points. */
  double goal_z_; /**< Target depth for maintaining distance from the centroid. */
  double z_scale_, x_scale_; /**< Scaling factors for translational and rotational speeds. */
  bool enabled_; /**< Whether the follower is actively publishing velocity commands. */

  // ROS Communication
  ros::ServiceServer switch_srv_; /**< Service to enable or disable the follower. */
  dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>* config_srv_; /**< Dynamic reconfiguration server. */
  ros::Subscriber sub_; /**< Subscriber for depth image input. */
  ros::Publisher cmdpub_; /**< Publisher for robot velocity commands. */
  ros::Publisher markerpub_; /**< Publisher for RViz markers to visualize the centroid. */
  ros::Publisher bboxpub_; /**< Publisher for RViz bounding box visualization. */

  // Initialization method for the nodelet
  /**
   * Initializes ROS parameters, sets up publishers and subscribers,
   * and configures dynamic reconfiguration callbacks.
   */
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    // Retrieve parameters from the parameter server
    private_nh.getParam("min_y", min_y_);
    private_nh.getParam("max_y", max_y_);
    private_nh.getParam("min_x", min_x_);
    private_nh.getParam("max_x", max_x_);
    private_nh.getParam("max_z", max_z_);
    private_nh.getParam("goal_z", goal_z_);
    private_nh.getParam("z_scale", z_scale_);
    private_nh.getParam("x_scale", x_scale_);
    private_nh.getParam("enabled", enabled_);

    // Setup publishers and subscribers
    cmdpub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); // Velocity command topic
    markerpub_ = private_nh.advertise<visualization_msgs::Marker>("marker", 1); // Centroid marker topic
    bboxpub_ = private_nh.advertise<visualization_msgs::Marker>("bbox", 1); // Bounding box marker topic
    sub_ = nh.subscribe<sensor_msgs::Image>("depth/image_rect", 1, &TurtlebotFollower::imagecb, this); // Depth image subscriber

    // Setup service for enabling/disabling the follower
    switch_srv_ = private_nh.advertiseService("change_state", &TurtlebotFollower::changeModeSrvCb, this);

    // Setup dynamic reconfigure server and callback
    config_srv_ = new dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>(private_nh);
    dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>::CallbackType f = 
        boost::bind(&TurtlebotFollower::reconfigure, this, _1, _2);
    config_srv_->setCallback(f);
  }

  /**
   * Updates configuration parameters in real-time using dynamic reconfiguration.
   * 
   * @param config Reference to the configuration object.
   * @param level Bitmask of configuration level (unused here).
   */
  void reconfigure(turtlebot_follower::FollowerConfig &config, uint32_t level)
  {
    min_y_ = config.min_y;
    max_y_ = config.max_y;
    min_x_ = config.min_x;
    max_x_ = config.max_x;
    max_z_ = config.max_z;
    goal_z_ = config.goal_z;
    z_scale_ = config.z_scale;
    x_scale_ = config.x_scale;
  }

  /**
   * Callback to process depth images, find the centroid of target points, and publish velocity commands.
   * 
   * @param depth_msg Depth image message.
   */
  void imagecb(const sensor_msgs::ImageConstPtr& depth_msg)
  {
    // Precompute trigonometric values for converting pixel positions to 3D coordinates
    uint32_t image_width = depth_msg->width;
    uint32_t image_height = depth_msg->height;
    float sin_pixel_x[image_width], sin_pixel_y[image_height];

    float x_radians_per_pixel = (60.0 / 57.0) / image_width; // Horizontal field of view
    float y_radians_per_pixel = (45.0 / 57.0) / image_width; // Vertical field of view

    for (int x = 0; x < image_width; ++x)
      sin_pixel_x[x] = sin((x - image_width / 2.0) * x_radians_per_pixel);

    for (int y = 0; y < image_height; ++y)
      sin_pixel_y[y] = sin((image_height / 2.0 - y) * y_radians_per_pixel);

    // Initialize centroid variables
    float x = 0.0, y = 0.0, z = 1e6; // Z initialized to a high value
    unsigned int n = 0; // Number of valid points in the bounding box

    // Process depth data row by row
    const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(float);

    for (int v = 0; v < static_cast<int>(image_height); ++v, depth_row += row_step)
    {
      for (int u = 0; u < static_cast<int>(image_width); ++u)
      {
        float depth = depth_image_proc::DepthTraits<float>::toMeters(depth_row[u]);

        // Skip invalid or out-of-range points
        if (!depth_image_proc::DepthTraits<float>::valid(depth) || depth > max_z_) continue;

        float y_val = sin_pixel_y[v] * depth;
        float x_val = sin_pixel_x[u] * depth;

        if (y_val > min_y_ && y_val < max_y_ && x_val > min_x_ && x_val < max_x_)
        {
          x += x_val;
          y += y_val;
          z = std::min(z, depth); // Update closest depth
          n++; // Increment valid point count
        }
      }
    }

    // Publish velocity commands or stop robot based on detected points
    if (n > 4000) // Threshold for sufficient points
    {
      x /= n;
      y /= n;

      if (z > max_z_) // Target too far, stop
      {
        ROS_INFO_THROTTLE(1, "Centroid too far: stopping.");
        if (enabled_) cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
        return;
      }

      geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
      cmd->linear.x = (z - goal_z_) * z_scale_;
      cmd->angular.z = -x * x_scale_;
      cmdpub_.publish(cmd);
    }
    else
    {
      ROS_INFO_THROTTLE(1, "Not enough points: stopping.");
      if (enabled_) cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
    }

    publishBbox(); // Update bounding box visualization
  }

  /**
   * Handles requests to enable or disable the follower.
   */
  bool changeModeSrvCb(turtlebot_msgs::SetFollowState::Request& request,
                       turtlebot_msgs::SetFollowState::Response& response)
  {
    if (enabled_ && request.state == request.STOPPED)
    {
      ROS_INFO("Following stopped.");
      cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist())); // Stop robot
      enabled_ = false;
    }
    else if (!enabled_ && request.state == request.FOLLOW)
    {
      ROS_INFO("Following started.");
      enabled_ = true;
    }

    response.result = response.OK;
    return true;
  }

  /**
   * Publishes a marker at the centroid location for visualization in RViz.
   */
  void publishMarker(double x, double y, double z)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.ns = "target_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.2; // Set marker size
    marker.color.a = 1.0; // Fully opaque
    marker.color.r = 1.0; // Red
    markerpub_.publish(marker);
  }

  /**
   * Publishes a visualization of the bounding box in RViz.
   */
  void publishBbox()
  {
    double x = (min_x_ + max_x_) / 2.0;
    double y = (min_y_ + max_y_) / 2.0;
    double z = max_z_ / 2.0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.ns = "bounding_box";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.scale.x = max_x_ - min_x_;
    marker.scale.y = max_y_ - min_y_;
    marker.scale.z = max_z_;
    marker.color.a = 0.5; // Transparent
    marker.color.g = 1.0; // Green
    bboxpub_.publish(marker);
  }
};

// Macro to register the nodelet with ROS
PLUGINLIB_DECLARE_CLASS(turtlebot_follower, TurtlebotFollower, turtlebot_follower::TurtlebotFollower, nodelet::Nodelet);

}
