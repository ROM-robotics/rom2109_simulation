#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rom2109_race/rom_aeb.hpp"
#include "rom2109_race/rom_double.hpp"
#include <cmath>

std::string odom_topic = "/diff_cont/odom";
std::string scan_topic = "/scan";
std::string twist_topic= "/linear_vel";

using namespace std::chrono_literals;
using namespace rom_dynamics::math;

class LinearPublisher : public rclcpp::Node
{
public:
    LinearPublisher();
    //bool should_drive(double dist, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg );

private:
    void timer_callback();
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    int count_;
    geometry_msgs::msg::Twist msg;
    geometry_msgs::msg::Twist::SharedPtr msg_ptr_;

    double odom_velocity_x_;
    double odom_velocity_y_;

    double min_angle_;
    double max_angle_;

    sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;

    double ttc_final_threshold_;
    rom_dynamics::vehicle::Aeb rom_aeb_;

    double linear_speed_;
};

LinearPublisher::LinearPublisher() : Node("linear_publisher"), count_(0)
{
    if (!this->has_parameter("linear_speed")) { this->declare_parameter("linear_speed", 1.0); }
    if (!this->has_parameter("min_angle")) { this->declare_parameter("min_angle", -90.0); }
    if (!this->has_parameter("max_angle")) { this->declare_parameter("max_angle", 90.0); }
    if (!this->has_parameter("ttc_final")) { this->declare_parameter("ttc_final", 1.0); }

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(twist_topic, 1);
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 10, std::bind(&LinearPublisher::scan_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&LinearPublisher::odom_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(100ms, std::bind(&LinearPublisher::timer_callback, this));

    msg.linear.x = 0.0;
    msg.angular.z = 0.0;

    odom_velocity_x_ = 0.0;
    odom_velocity_y_ = 0.0;

    linear_speed_ = this->get_parameter("linear_speed").as_double();
    msg.linear.x = linear_speed_;

    ttc_final_threshold_ = this->get_parameter("ttc_final").as_double(); // seconds // please try default value
    rom_aeb_.setTtcFinalThreshold(ttc_final_threshold_);

    min_angle_ = this->get_parameter("min_angle").as_double();
    max_angle_ = this->get_parameter("max_angle").as_double();
}

void LinearPublisher::timer_callback()
{
    if ( scan_msg_ == nullptr || odom_msg_ == nullptr )
    {   // Ensure both messages have been received before processing
        return; 
    }

    bool flag = rom_aeb_.should_brake(scan_msg_, odom_msg_, min_angle_, max_angle_);

    if (flag == true)
    {
        msg.linear.x -= 0.300; if (msg.linear.x <= 0.000 ){ msg.linear.x = 0.000;}
        RCLCPP_INFO(rclcpp::get_logger("\033[1;31mBraking\033[1;0m"), ": \033[1;31m%.4f m/s\033[1;0m", msg.linear.x);

        // check should_drive
        //if( should_drive( rom_aeb_.getDistance(), odom_msg_) ) { msg.linear.x = linear_speed_; }
    }
    else
    {
        msg.linear.x = linear_speed_;
        RCLCPP_INFO(rclcpp::get_logger("\033[1;36mLinear Velocity\033[1;0m"), ": \033[1;36m%.4f m/s\033[1;0m", msg.linear.x);
    }
    twist_publisher_->publish(msg);
}

void LinearPublisher::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    scan_msg_ = std::move(msg);
}

void LinearPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_msg_ = std::move(msg);
    odom_velocity_x_ = odom_msg_->twist.twist.linear.x;
    odom_velocity_y_ = odom_msg_->twist.twist.linear.y;
}

/*bool LinearPublisher::should_drive(double dist, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg )
{
    if(rom_dynamics::math::areDoublesEqual(odom_msg->twist.twist.linear.x, 0.00000, 1e-5)==true && rom_dynamics::math::isDoubleGreater(dist, 0.50000, 1e-5)==true)
    {
        return true;
    } else { return false; }
}*/

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LinearPublisher>());
    rclcpp::shutdown();
    return 0;
}