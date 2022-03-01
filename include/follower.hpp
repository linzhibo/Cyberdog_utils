/*
 * Author: Zhibo Lin
 */

#ifndef DEPTH_IMAGE_FOLLOWER
#define DEPTH_IMAGE_FOLLOWER

#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <std_srvs/srv/set_bool.hpp>

#include "motion_msgs/msg/se3_velocity_cmd.hpp"
#include "motion_msgs/msg/parameters.hpp"
#include "motion_msgs/action/change_gait.hpp"
#include "motion_msgs/msg/gait.hpp"
#include "depth_traits.h"

class DepthFollower : public rclcpp::Node
{
public:

    DepthFollower();
    ~DepthFollower();

private:
    sensor_msgs::msg::CameraInfo::SharedPtr cam_info_;
    image_geometry::PinholeCameraModel cam_model_;

    void depthCb(const sensor_msgs::msg::Image::SharedPtr image);
    void infoCb(sensor_msgs::msg::CameraInfo::SharedPtr info);
    void paramCb(const motion_msgs::msg::Parameters::SharedPtr param);
    void camera_service_call(std::string service_name, bool process);
    void checkGaitFbCb(const motion_msgs::action::ChangeGait_FeedbackMessage::SharedPtr msg);
    void discover_dogs_ns();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<motion_msgs::msg::Parameters>::SharedPtr param_sub_;
    rclcpp::Subscription<motion_msgs::action::ChangeGait_FeedbackMessage>::SharedPtr check_gait_sub_;
    rclcpp::Publisher<motion_msgs::msg::SE3VelocityCMD>::SharedPtr cmdpub_;

    double min_y_ = 0.1; /**< The minimum y position of the points in the box. */
    double max_y_ = 0.5; /**< The maximum y position of the points in the box. */
    double min_x_ = -0.3; /**< The minimum x position of the points in the box. */
    double max_x_ = 0.3; /**< The maximum x position of the points in the box. */
    double max_z_ = 1.5; /**< The maximum z position of the points in the box. */
    double goal_z_ = 0.6; /**< The distance away from the robot to hold the centroid */
    double z_scale_ = 1.0; /**< The scaling factor for translational robot speed */
    double x_scale_ = 5.0; /**< The scaling factor for rotational robot speed */
    bool   enabled_ = true; /**< Enable/disable following; just prevents motor commands */
    uint    thresh_pcl_number_ = 4000; /**< Threshold for minimum available pointcloud number*/
    std::string depth_topic_, cmd_topic_, depth_topic_cam_info_, namespace_;
    bool camera_enabled_;
    double body_height_ = 0.0;
    int current_gait_ = 0;
    std::string dogs_namespace_ = "/mi123456789/";
};
#endif