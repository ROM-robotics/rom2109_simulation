#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

std::string odom_topic = "/diff_cont/odom";
std::string scan_topic = "/scan";
std::string publish_topic = "/aeb/cmd_vel";
std::string aeb_topic = "aeb_status";



class Safety : public rclcpp::Node {

public:
    Safety() : Node("rom_safety_node")
    {
        if( !this->has_parameter("ttc_final") )
        {
            this->declare_parameter("ttc_final", 1.0);
        }
        TTC_final_threshold = this->get_parameter("ttc_final").as_double(); // second

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 10, std::bind(&Safety::scan_callback, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&Safety::odom_callback, this, _1));

        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(publish_topic, 10);
        brake_pub_ = this->create_publisher<std_msgs::msg::Bool>(aeb_topic, 10); //brake topic 

        timer_ = this->create_wall_timer( 100ms, std::bind(&Safety::timer_callback, this) );

        brake_msg_.linear.x = 0.0000;
        brake_msg_.angular.z = 0.0000;

        RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;33mTTC_Final : %.4f\033[1;0m", TTC_final_threshold);
    }

private:
    geometry_msgs::msg::Twist brake_msg_;
    bool should_brake_ = false;
    double TTC_final_threshold;
    double odom_velocity_x_ = 0.0;
    double odom_velocity_y_ = 0.0;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brake_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    

    void timer_callback()
    {
      if(should_brake_ == true)
      { 
          std_msgs::msg::Bool brake_status_;
          brake_status_.data = should_brake_;
          brake_pub_->publish(brake_status_);
        // twist_pub_->publish(brake_msg_);
        RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;31mBreaking\033[1;0m");
      }
    //   else 
    //   { 
    //     RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;32mReleased\033[1;0m"); 
    //     RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;33mTTC_Final : %.4f\033[1;0m", TTC_final_threshold);
    //     }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        // လက်တွေ့မှာ param read က topic ကို မလိုက်နိုင်လောက်ဘူး။ dynamic reconfigure သုံးရင်သုံး
        // ဒီလိုင်းကို ဖျက်လို့ ရလောက်တယ်။
        TTC_final_threshold = this->get_parameter("ttc_final").as_double(); // seconds
        
        double min_TTC = 1000000.0;
        double velocity_x = this->odom_velocity_x_;
        
        // 180 ပဲယူမယ်။ ဘာလို့ဆို lidar က 360 ိ ဖြစ်နေလို့ပါ။
        // for (unsigned int i = 90; i < 270; i++) 
        // {
            //RCLCPP_INFO(this->get_logger(), "ranges i : %d", i);

            //if (!std::isinf(scan_msg->ranges[i]) && !std::isnan(scan_msg->ranges[i]))  

            // lidar angle filter ထည့်ရန်

            for (std::size_t i = 0; i < scan_msg->ranges.size(); i++) {
            {
                double distance = scan_msg->ranges[i];

                if (std::isnan(distance) || std::isinf(distance) || distance < scan_msg->range_min) {
                continue;
            }
            
                double angle = scan_msg->angle_min + scan_msg->angle_increment * i;
                double distance_derivative = cos(angle) * velocity_x;    

                // RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;distance_derivative: %.4f\033[1;0m", distance_derivative);
                
                if ( distance_derivative > 0 && (distance/distance_derivative) < min_TTC ) 
                {
                    min_TTC = distance / std::max(distance_derivative, 0.001);
                    // RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;33mmin_TTC : %.4f\033[1;0m", min_TTC);
                
                }
            }
        }
        if (min_TTC <= TTC_final_threshold) {
            should_brake_ = true;
        } else {
            should_brake_ = false;
        }
    }
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
    {
        this->odom_velocity_x_ = odom_msg->twist.twist.linear.x;
        this->odom_velocity_y_ = odom_msg->twist.twist.linear.y;
    }

};


// rom_dynamics::vechicle::Aeb_diff_drive::Aeb_diff_drive(int argc, char ** argv){
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<Safety>());
//     rclcpp::shutdown();
// }



// int main(int argc, char ** argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<Safety>());
//     rclcpp::shutdown();
//     return 0;
// }