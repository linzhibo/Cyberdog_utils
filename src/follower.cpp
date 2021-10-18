#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "motion_msgs/msg/se3_velocity_cmd.hpp"
#include "depth_traits.h"

#define ROS_WARN RCUTILS_LOG_WARN
#define ROS_ERROR RCUTILS_LOG_ERROR
#define ROS_INFO_THROTTLE(sec, ...) RCUTILS_LOG_INFO_THROTTLE(RCUTILS_STEADY_TIME, sec, __VA_ARGS__)

using std::placeholders::_1;

class RS_Follower : public rclcpp::Node
{
public:

  RS_Follower() : Node("rs_follower")
  {
    this->declare_parameter<std::string>("depth_topic", "/camera/depth/image_rect_raw");
    this->declare_parameter<std::string>("cmd_topic", "/cmd_vel");
    this->declare_parameter<double>("min_y", 0.1);
    this->declare_parameter<double>("max_y", 0.5);
    this->declare_parameter<double>("min_x", -0.3);
    this->declare_parameter<double>("max_x", 0.3);
    this->declare_parameter<double>("max_z", 1.5);
    this->declare_parameter<double>("goal_z", 0.6);
    this->declare_parameter<double>("z_scale", 1.0);
    this->declare_parameter<double>("x_scale", 5.0);
    this->declare_parameter<bool>("enabled", true);

    initialize();
  }

  ~RS_Follower()
  {
  }

private:
  
  
  void initialize()
  {
    this->get_parameter("depth_topic", depth_topic_);
    this->get_parameter("cmd_topic", cmd_topic_);
    this->get_parameter("min_y", min_y_);
    this->get_parameter("max_y", max_y_);
    this->get_parameter("min_x", min_x_);
    this->get_parameter("max_x", max_x_);
    this->get_parameter("max_z", max_z_);
    this->get_parameter("goal_z", goal_z_);
    this->get_parameter("z_scale", z_scale_);
    this->get_parameter("x_scale", x_scale_);
    this->get_parameter("enabled", enabled_);

    // auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(10));
    RCLCPP_INFO(this->get_logger(), "Publishing to topic '%s'", cmd_topic_.c_str());
    // cmdpub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, qos_cmd);

    cmdpub_ = this->create_publisher<motion_msgs::msg::SE3VelocityCMD>(cmd_topic_, rclcpp::SystemDefaultsQoS());

    size_t depth_ = rmw_qos_profile_default.depth;
    rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
    rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;

    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        history_policy_,
        depth_
    ));

    qos.reliability(reliability_policy_);
    auto callback = [this](const sensor_msgs::msg::Image::SharedPtr msg)
      {
        process_image(msg);
      };

    RCLCPP_INFO(this->get_logger(), "Subscribing to topic '%s'", depth_topic_.c_str());
    sub_ = create_subscription<sensor_msgs::msg::Image>(depth_topic_, rclcpp::SensorDataQoS(), callback);
    // imageDepthSub_.subscribe(&node, "depth/image", hints.getTransport(), rmw_qos_profile_sensor_data);
  }

  void process_image(const sensor_msgs::msg::Image::SharedPtr depth_msg)
  {
    std::cout<<"got depth image: " << std::endl;
    if(depth_msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1)
    {
      std::cout<<"received depth image with unsupported encoding: " << depth_msg->encoding.c_str() << std::endl;
      return;
    }

    // Precompute the sin function for each row and column
    uint32_t image_width = depth_msg->width;
    float x_radians_per_pixel = 60.0/57.0/image_width;
    std::vector<float> sin_pixel_x(image_width);
    for (uint32_t x = 0; x < image_width; ++x) {
      sin_pixel_x[x] = sin((x - image_width/ 2.0)  * x_radians_per_pixel);
    }

    uint32_t image_height = depth_msg->height;
    float y_radians_per_pixel = 45.0/57.0/image_width;
    std::vector<float> sin_pixel_y(image_height);
    for (uint32_t y = 0; y < image_height; ++y) {
      // Sign opposite x for y up values
      sin_pixel_y[y] = sin((image_height/ 2.0 - y)  * y_radians_per_pixel);
    }

    //X,Y,Z of the centroid
    float x = 0.0;
    float y = 0.0;
    float z = 1e6;
    //Number of points observed
    unsigned int n = 0;
    float max_depth = 0;

    //Iterate through all the points in the region and find the average of the position
    const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(float);
    for (int v = 0; v < (int)depth_msg->height; ++v, depth_row += row_step)
    {
     for (int u = 0; u < (int)depth_msg->width; ++u)
     {
       float depth = depth_image_proc::DepthTraits<float>::toMeters(depth_row[u]);
       if(depth > max_depth)
        {
          max_depth = depth;
        }
       if (!depth_image_proc::DepthTraits<float>::valid(depth) || depth > max_z_) continue;
       float y_val = sin_pixel_y[v] * depth;
       float x_val = sin_pixel_x[u] * depth;
       if ( y_val > min_y_ && y_val < max_y_ &&
            x_val > min_x_ && x_val < max_x_)
       {
         x += x_val;
         y += y_val;
         z = std::min(z, depth); //approximate depth as forward.
         
         n++;
       }
     }
    }
    std::cout<< "max depth: "<< max_depth << std::endl;

    // auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
    motion_msgs::msg::SE3VelocityCMD cmd_vel_msg;

    cmd_vel_msg.sourceid = motion_msgs::msg::SE3VelocityCMD::REMOTEC;
    // cmd_vel_msg.velocity.frameid.id = motion_msgs::msg::Frameid::BODY_FRAME;
    // cmd_vel_msg.velocity.timestamp = decision_->get_clock()->now();
    cmd_vel_msg.velocity.linear_x = 0;
    cmd_vel_msg.velocity.linear_y = 0;
    cmd_vel_msg.velocity.linear_z = 0;

    cmd_vel_msg.velocity.angular_x = 0;
    cmd_vel_msg.velocity.angular_y = 0;
    cmd_vel_msg.velocity.angular_z = 0;

    // cmd_vel_msg->linear.x = 0.0;
    // cmd_vel_msg->linear.y = 0.0;
    // cmd_vel_msg->linear.z = 0.0;

    // cmd_vel_msg->angular.x = 0.0;
    // cmd_vel_msg->angular.y = 0.0;
    // cmd_vel_msg->angular.z = 0.0;

    //If there are points, find the centroid and calculate the command goal.
    //If there are no points, simply publish a stop goal.
    if (n>4000)
    {
      x /= n;
      y /= n;
      if(z > max_z_){
        ROS_INFO_THROTTLE(1, "Centroid too far away %f, stopping the robot", z);
        if (enabled_)
        {
          // cmdpub_->publish(std::move(cmd_vel_msg));
        }
        return;
      }

      ROS_INFO_THROTTLE(1, "Centroid at %f %f %f with %d points", x, y, z, n);

      if (enabled_)
      {
        // cmd_vel_msg->linear.x = (z - goal_z_) * z_scale_;
        // cmd_vel_msg->angular.z = -x * x_scale_;
        // cmdpub_->publish(std::move(cmd_vel_msg));
        cmd_vel_msg.velocity.linear_x = (z - goal_z_) * z_scale_;
        cmd_vel_msg.velocity.angular_z = -x * x_scale_;
        cmdpub_->publish(cmd_vel_msg);
      }
    }
    else
    {
      ROS_INFO_THROTTLE(1, "Not enough points(%d) detected, stopping the robot", n);

      if (enabled_)
      {
        // cmdpub_->publish(std::move(cmd_vel_msg));
        cmdpub_->publish(cmd_vel_msg);
      }
    }

  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<motion_msgs::msg::SE3VelocityCMD>::SharedPtr cmdpub_;

  size_t depth_ = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
  double min_y_ = 0.1; /**< The minimum y position of the points in the box. */
  double max_y_ = 0.5; /**< The maximum y position of the points in the box. */
  double min_x_ = -0.3; /**< The minimum x position of the points in the box. */
  double max_x_ = 0.3; /**< The maximum x position of the points in the box. */
  double max_z_ = 1.5; /**< The maximum z position of the points in the box. */
  double goal_z_ = 0.6; /**< The distance away from the robot to hold the centroid */
  double z_scale_ = 1.0; /**< The scaling factor for translational robot speed */
  double x_scale_ = 5.0; /**< The scaling factor for rotational robot speed */
  bool   enabled_ = true; /**< Enable/disable following; just prevents motor commands */
  std::string depth_topic_, cmd_topic_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RS_Follower>());
  rclcpp::shutdown();
  return 0;
}