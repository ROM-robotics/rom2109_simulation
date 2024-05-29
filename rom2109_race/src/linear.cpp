#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class LinearPublisher : public rclcpp::Node
{
  public:
    LinearPublisher()
    : Node("linear_publisher"), count_(60)
    {
      if( !this->has_parameter("linear_speed") )
        {
            this->declare_parameter("linear_speed", 1.0);
        }
      

      twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/linear_vel", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&LinearPublisher::timer_callback, this));

      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
    }

  private:
    void timer_callback()
    {
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
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    int count_;
    geometry_msgs::msg::Twist msg;
    geometry_msgs::msg::Twist::SharedPtr msg_ptr;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinearPublisher>());
  rclcpp::shutdown();
  return 0;
}