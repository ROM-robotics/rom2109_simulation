#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rom_pid.hpp"

using namespace std::chrono_literals;


class LinearPublisher : public rclcpp::Node
{
  public:
    LinearPublisher()
    : Node("pid_tester"), count_(60)
    {
        if( !this->has_parameter("desire_value") )
        {
            this->declare_parameter("desire_value", 1.0);
        }
        if( !this->has_parameter("Kp") )
        {
            this->declare_parameter("Kp", 1.0);
        }
        if( !this->has_parameter("Ki") )
        {
            this->declare_parameter("Ki", 0.0);
        }
        if( !this->has_parameter("Kd") )
        {
            this->declare_parameter("Kd", 0.0);
        }
        if( !this->has_parameter("min") )
        {
            this->declare_parameter("min", 0.0);
        }
        if( !this->has_parameter("max") )
        {
            this->declare_parameter("max", 100.0);
        }
      

      desire_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/desire", 100);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&LinearPublisher::timer_callback1, this));
      pid_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/pid", 100);

      desire_vel_msg.linear.x = 0.0;
      desire_vel_msg.angular.z = 0.0;
      pid_vel_msg.linear.x = 0.0;
      pid_vel_msg.angular.z = 0.0;

      pid.setPID(1.0, 0.0, 0.0);

      previous_time_ = this->now();
    }
    

  private:
    void timer_callback1()
    {
        desire_vel_msg.linear.x = this->get_parameter("desire_value").as_double();
        desire_publisher_->publish(desire_vel_msg);

        rclcpp::Time current_time_ = this->now();
            rclcpp::Duration time_diff = current_time_ - previous_time_;
            double dt = time_diff.seconds();

            double kp = this->get_parameter("Kp").as_double();
            double ki = this->get_parameter("Ki").as_double();
            double kd = this->get_parameter("Kd").as_double();
            double min = this->get_parameter("min").as_double();
            double max = this->get_parameter("max").as_double();
        pid.setPID(kp, ki, kd);

        pid_vel_msg.linear.x = pid.computePID(desire_vel_msg.linear.x, pid_vel_msg.linear.x, dt, min, max);
        
        pid_publisher_->publish(pid_vel_msg);

        previous_time_ = current_time_;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr desire_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pid_publisher_;
    int count_;
    geometry_msgs::msg::Twist desire_vel_msg;
    geometry_msgs::msg::Twist pid_vel_msg;
    rclcpp::Time previous_time_;

    rom_dynamics::control::PID pid;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinearPublisher>());
  rclcpp::shutdown();
  return 0;
}

// ros2 run rom2109_race test_code --ros-args -p desire_value:=1.5