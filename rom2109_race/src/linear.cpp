#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

std::string odom_topic = "/diff_cont/odom";
std::string scan_topic = "/scan";

using namespace std::chrono_literals;

class LinearPublisher : public rclcpp::Node
{
  public:
    LinearPublisher();

  private:
    void timer_callback();
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;   // subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    int count_;
    geometry_msgs::msg::Twist msg;
    geometry_msgs::msg::Twist::SharedPtr msg_ptr;
    nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_;
    double odom_velocity_x_;
    double odom_velocity_y_ ;

    sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg_;
    double min_angle_;
    double max_angle_;

    romrobotics::Vehicle::AEB aeb_;
    romrobotics::Control::PID pid_;
};

LinearPublisher::LinearPublisher() : Node("linear_publisher"), count_(60)
{
      if( !this->has_parameter("linear_speed") )
        {
            this->declare_parameter("linear_speed", 1.0); // 1 ms
        }
      this->declare_parameter("min_angle", -90.0);  // degree
      this->declare_parameter("max_angle", 90.0);

      twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/linear_vel", 10);
      laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 10, std::bind(&LinearPublisher::scan_callback, this, std::placeholders::_1));
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&LinearPublisher::odom_callback, this, std::placeholders::_1));

      timer_ = this->create_wall_timer(100ms, std::bind(&LinearPublisher::timer_callback, this));

      msg.linear.x = 0.0;
      msg.angular.z = 0.0;

      odom_velocity_x_ = 0.0;
      odom_velocity_y_ = 0.0;
}

void LinearPublisher::timer_callback()
{ 
      /* 
      int timer_second = count_/10;

      if(count_ % 10 == 0 && count_ >= 0)
      { // တစက်ကန့် တခါ timer ပြဖို့ ပါ။
        RCLCPP_INFO(this->get_logger(), "Ready to go !: %d", timer_second);
      }
      count_--;

      if(count_ < 0)
      { // ၆ စက်ကန့် ပြီး ရင် မောင်းမယ်။
        double linear_speed = this->get_parameter("linear_speed").as_double();
        msg.linear.x = linear_speed;
        twist_publisher_->publish(msg);

        if(count_ % -5 == 0)
        { // ၆ စက်ကန့်ပြီးရင် တစက်ကန့် ကို ၂ ခါ display ပြရန်။
          RCLCPP_INFO(rclcpp::get_logger("\033[1;36mLinear Velocity\033[1;0m"), ": \033[1;36m%.4f m/s\033[1;0m", msg.linear.x);
        }
      }
      */
      double linear_speed = calculatePID(aeb_speed); // accel , decel  = this->get_parameter("linear_speed").as_double();
      bool flag = shouldBrake(scan,);

      double pid_speed;
      if(flag) 
      {
        // // deceleration
        pid_speed = calculatePID(linear_speed);
        // checkHeading(); // VFF angular velocity
      } 
      else
      {
        pid_speed = calculatePID(linear_speed); // accel 
      }
      
      
      msg.linear.x = pid_speed;

      
      twist_publisher_->publish(msg);
     
}

void LinearPublisher::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) 
{
    min_angle_ = this->get_parameter("min_angle").as_double();   //laser rays ရဲ့ angle ကို dynamically ချိန်ညှိနိုင်ရန် နှင့် parameter သတ်မှတ်ရလွယ်ကူစေရန်
    max_angle_ = this->get_parameter("max_angle").as_double();
    scan_msg_ = msg;
}

void LinearPublisher::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) 
{
    this->odom_velocity_x_ = msg->twist.twist.linear.x;
    this->odom_velocity_y_ = msg->twist.twist.linear.y;
    odom_msg_ = msg;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinearPublisher>());
  rclcpp::shutdown();
  return 0;
}