#include "rom2109_race/rom_aeb.hpp"
#include "rom2109_race/laser_angle_filter.hpp"

using namespace std::chrono_literals;


std::string publish_topic = "/diff_cont/cmd_vel_unstamped";
std::string aeb_topic = "/aeb_status";
std::string laser_filtered_topic = "/filtered_scan";
std::string input_velocity_topic = "/aeb/input_vel";



rom_dynamics::vechicle::AEB::AEB() : Node("Rom_AEB_Node") {
    if (!this->has_parameter("ttc_final")) {
        this->declare_parameter("ttc_final", 1.0);
    }

    if( !this->has_parameter("min_angle") && !this->has_parameter("max_angle") )
        {
            this->declare_parameter("min_angle", -30.0);
            this->declare_parameter("max_angle", 30.0);
        }

    // bool should_brake_ = false;

    // double min_angle = this->get_parameter("min_angle").as_double();   //laser rays ရဲ့ angle ကို dynamically ချိန်ညှိနိုင်ရန် နှင့် parameter သတ်မှတ်ရလွယ်ကူစေရန်
    // double max_angle = this->get_parameter("max_angle").as_double();

    // TTC_final_threshold = this->get_parameter("ttc_final").as_double(); // second

    

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(publish_topic, 10);
    vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            input_velocity_topic, 10, std::bind(&rom_dynamics::vechicle::AEB::velocity_subscription, this, std::placeholders::_1));


    brake_pub_ = this->create_publisher<std_msgs::msg::Bool>(aeb_topic, 10); //brake topic 
    laser_filtered_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(laser_filtered_topic, 10);     // laser_angle_filter အတွက် publisher 

    timer_ = this->create_wall_timer(100ms, std::bind(&AEB::timer_callback, this));

    brake_msg_.linear.x = 0.0000;
    brake_msg_.angular.z = 0.0000;

    RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;33mTTC_Final : %.4f\033[1;0m", TTC_final_threshold);
}

void rom_dynamics::vechicle::AEB::timer_callback() {
    if (should_brake_ == true) { 
       
        brake_status_.data = should_brake_;
        brake_pub_->publish(brake_status_);
        twist_pub_->publish(brake_msg_);
        RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;31mBreaking\033[1;0m");
    }
    else { 
        RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;32mReleased\033[1;0m"); 
        RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;33mTTC_Final : %.4f\033[1;0m", TTC_final_threshold);
        brake_status_.data = should_brake_;
        brake_pub_->publish(brake_status_);
        twist_pub_->publish(input_vel_);
    }
}



void rom_dynamics::vechicle::AEB::velocity_subscription(const geometry_msgs::msg::Twist::ConstSharedPtr vel_msg){

    this->input_vel_.linear = vel_msg->linear;
    this->input_vel_.angular = vel_msg->angular;

}