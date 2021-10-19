#include "follower.hpp"

#define ROS_INFO_THROTTLE(sec, ...) RCUTILS_LOG_INFO_THROTTLE(RCUTILS_STEADY_TIME, sec, __VA_ARGS__)

DepthFollower::DepthFollower(): rclcpp::Node("depth_follower")
{
this->declare_parameter<std::string>("depth_topic", "/camera/depth/image_rect_raw");
this->declare_parameter<std::string>("depth_topic_cam_info", "/camera/depth/camera_info");
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

this->get_parameter("depth_topic", depth_topic_);
this->get_parameter("depth_topic_cam_info", depth_topic_cam_info_);
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

RCLCPP_INFO(this->get_logger(), "Publishing to topic '%s'", cmd_topic_.c_str());
RCLCPP_INFO(this->get_logger(), "Subscribing to topic '%s', '%s'", depth_topic_.c_str(), depth_topic_cam_info_.c_str());

auto qos = rclcpp:: SystemDefaultsQoS();

depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(depth_topic_, 
                                                                rclcpp::SensorDataQoS(), 
                                                                std::bind(&DepthFollower::depthCb, this, std::placeholders::_1));
cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(depth_topic_cam_info_, 
                                                                  qos,
                                                                  std::bind(&DepthFollower::infoCb, this,std::placeholders::_1));

cmdpub_ = create_publisher<motion_msgs::msg::SE3VelocityCMD>(cmd_topic_, qos);
}

DepthFollower::~DepthFollower()
{
}

void DepthFollower::depthCb(const sensor_msgs::msg::Image::SharedPtr image)
{
    if (nullptr == cam_info_)
    {
    RCLCPP_INFO(get_logger(), "No camera info, skipping point cloud squash");
    return;
    }
    cam_model_.fromCameraInfo(cam_info_);

    // std::cout<<"got depth image: " << std::endl;
    
    if(image->encoding != sensor_msgs::image_encodings::TYPE_16UC1)
    {
      std::cout<<"received depth image with unsupported encoding: " << image->encoding.c_str() << std::endl;
      return;
    }

    //X,Y,Z of the centroid
    float x = 0.0;
    float y = 0.0;
    float z = 1e6;
    //Number of points observed
    unsigned int n = 0;
    float max_depth = 0;

    float constant_x = 1.0 / cam_model_.fx();
    float constant_y = 1.0 / cam_model_.fy();

    const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&image->data[0]);
    int row_step = image->step / sizeof(uint16_t);
    for (int v = 0; v < (int)image->height; v++, depth_row += row_step)
    {
        for (int u = 0; u < (int)image->width; u++)
        {
            // float depth = depth_image_proc::DepthTraits<float>::toMeters(depth_row[u]);
            float depth = depth_row[u] * 0.001;
            // std::cout<< depth << " "<< depth_row[u] << std::endl;
            if(depth > max_depth)
            {
                max_depth = depth;
            }
            if (!depth_image_proc::DepthTraits<float>::valid(depth) || depth > max_z_) continue;
            float y_val = constant_y * v * depth;
            float x_val = constant_x * u * depth;
            // std::cout<< y_val << " " << x_val << std::endl;
            if ( y_val > min_y_ && y_val < max_y_ && x_val > min_x_ && x_val < max_x_)
            {
                x += x_val;
                y += y_val;
                z = std::min(z, depth); //approximate depth as forward.
                n++;
            }
        }
    }

    motion_msgs::msg::SE3VelocityCMD cmd_vel_msg;
    cmd_vel_msg.sourceid = motion_msgs::msg::SE3VelocityCMD::REMOTEC;
    cmd_vel_msg.velocity.linear_x = 0;
    cmd_vel_msg.velocity.linear_y = 0;
    cmd_vel_msg.velocity.linear_z = 0;

    cmd_vel_msg.velocity.angular_x = 0;
    cmd_vel_msg.velocity.angular_y = 0;
    cmd_vel_msg.velocity.angular_z = 0;

    if (n>4000)
    {
        x /= n;
        y /= n;
        if(z > max_z_)
        {
            ROS_INFO_THROTTLE(1, "Centroid too far away %f, stopping the robot", z);
            if (enabled_)
            {
            cmdpub_->publish(cmd_vel_msg);
            }
            return;
        }

        ROS_INFO_THROTTLE(1, "Centroid at %f %f %f with %d points", x, y, z, n);

        if (enabled_)
        {
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
            cmdpub_->publish(cmd_vel_msg);
        }
    }
}

void DepthFollower::infoCb(sensor_msgs::msg::CameraInfo::SharedPtr info)
{
  cam_info_ = info;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthFollower>());
    rclcpp::shutdown();
    return 0;
}