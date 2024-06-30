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

std::string odom_topic = "/diff_cont/odom";
std::string scan_topic = "/scan";
std::string twist_topic= "/linear_vel";

using namespace std::chrono_literals;

class LinearPublisher : public rclcpp::Node
{
public:
    LinearPublisher();

private:
    void timer_callback();
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    int count_;
    geometry_msgs::msg::Twist msg_;
    geometry_msgs::msg::Twist::SharedPtr msg_ptr_;

    double odom_velocity_x_;
    double odom_velocity_y_;

    double min_angle_;
    double max_angle_;

    sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;
};

LinearPublisher::LinearPublisher() : Node("linear_publisher"), count_(0)
{
    if (!this->has_parameter("linear_speed"))
    {
        this->declare_parameter("linear_speed", 1.0); // 1 m/s
    }
    this->declare_parameter("min_angle", -90.0); // degree
    this->declare_parameter("max_angle", 90.0);
    this->declare_parameter("ttc_final", 1.0);

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(twist_topic, 1);
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 10, std::bind(&LinearPublisher::scan_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&LinearPublisher::odom_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(500ms, std::bind(&LinearPublisher::timer_callback, this));

    msg_.linear.x = 0.0;
    msg_.angular.z = 0.0;

    odom_velocity_x_ = 0.0;
    odom_velocity_y_ = 0.0;
    double linear_speed = this->get_parameter("linear_speed").as_double();
    msg.linear.x = linear_speed;
}

void LinearPublisher::timer_callback()
{
    if (!scan_msg_ || !odom_msg_)
    {
        return; // Ensure both messages have been received before processing
    }

    // auto input_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan_msg_);
    // auto input_odom = std::make_shared<nav_msgs::msg::Odometry>(*odom_msg_);

    //double min_angle = this->get_parameter("min_angle").as_double();
    //ouble max_angle = this->get_parameter("max_angle").as_double();

    
    //RCLCPP_INFO(rclcpp::get_logger("\033[1;34mTTC\033[1;0m"), ": \033[1;34m%.4f second\033[1;0m", rom_aeb.getMinTtc());
    //RCLCPP_INFO(rclcpp::get_logger("\033[1;37mdistance\033[1;0m"), ": \033[1;37m%.4f second\033[1;0m", rom_aeb.getDistance());
    //RCLCPP_INFO(rclcpp::get_logger("\033[1;38mvelocity\033[1;0m"), ": \033[1;38m%.4f second\033[1;0m", rom_aeb.getVelocity());
}

void LinearPublisher::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    scan_msg_ = msg;
    if (!odom_msg_) { return; }

    double ttc_threshold = this->get_parameter("ttc_final").as_double(); // seconds // please try default value

    rom_dynamics::vechicle::Aeb rom_aeb(ttc_threshold);

    bool flag = rom_aeb.should_brake(scan_msg_, odom_msg_, min_angle_, max_angle_);

    if (flag == true)
    {
        msg_.linear.x -= 0.01; if (msg_.linear.x <= 0 ){ msg_.linear.x = 0;}
        twist_publisher_->publish(msg_);
        RCLCPP_INFO(rclcpp::get_logger("\033[1;36mBraking\033[1;0m"), ": \033[1;36m%.4f m/s\033[1;0m", msg.linear.x);
    }
    else
    {
        double linear_speed = this->get_parameter("linear_speed").as_double();
        msg_.linear.x = linear_speed;
        twist_publisher_->publish(msg_);
        RCLCPP_INFO(rclcpp::get_logger("\033[1;36mLinear Velocity\033[1;0m"), ": \033[1;36m%.4f m/s\033[1;0m", msg.linear.x);
    }

}

void LinearPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_velocity_x_ = msg->twist.twist.linear.x;
    odom_velocity_y_ = msg->twist.twist.linear.y;
    odom_msg_ = msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LinearPublisher>());
    rclcpp::shutdown();
    return 0;
}