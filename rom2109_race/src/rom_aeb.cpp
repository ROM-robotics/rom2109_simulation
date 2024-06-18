#include "rom2109_race/rom_aeb.hpp"
#include "rom2109_race/laser_angle_filter.hpp"

using namespace std::chrono_literals;

std::string odom_topic = "/diff_cont/odom";
std::string scan_topic = "/scan";
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

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 10, std::bind(&rom_dynamics::vechicle::AEB::scan_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&rom_dynamics::vechicle::AEB::odom_callback, this, std::placeholders::_1));

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

void rom_dynamics::vechicle::AEB::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // Actual implementation here
    TTC_final_threshold = this->get_parameter("ttc_final").as_double(); // seconds

    double min_angle = this->get_parameter("min_angle").as_double();   //laser rays ရဲ့ angle ကို dynamically ချိန်ညှိနိုင်ရန် နှင့် parameter သတ်မှတ်ရလွယ်ကူစေရန်
    double max_angle = this->get_parameter("max_angle").as_double();

    
    double min_TTC = 1000000.0;
    double velocity_x = this->odom_velocity_x_;


    // 180 ပဲယူမယ်။ ဘာလို့ဆို lidar က 360 ိ ဖြစ်နေလို့ပါ။                  // Lidar angle ကို index နည်းလမ်းဖြင့်ယူခြင်း
    // for (unsigned int i = 90; i < 270; i++) 
    // {
        //RCLCPP_INFO(this->get_logger(), "ranges i : %d", i);

        //if (!std::isinf(scan_msg->ranges[i]) && !std::isnan(scan_msg->ranges[i]))  

    auto input_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan_msg); 
    //std::make_shared<sensor_msgs::msg::LaserScan> laser_filter;

    LaserAngleFilter laser_filter; //custom object
    
    sensor_msgs::msg::LaserScan::SharedPtr filtered_scan = laser_filter.filter_angle(input_scan, min_angle, max_angle);

    // <laser_filter object ထဲ က filter_angle အလုပ်လုပ်ပုံ>
    
    // double angle_min = min_angle * M_PI / 180.0;  // degree to radian
    // double angle_max = max_angle * M_PI / 180.0; 
    // double current_angle = scan_msg->angle_min;

    // auto const filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan_msg);
    //     filtered_scan->ranges.clear();



    // for (const auto &range : scan_msg->ranges)           // laser data အသစ်ဖန်တီးခြင်း
    //     {
    //         if (current_angle >= angle_min && current_angle <= angle_max)
    //         {
    //             filtered_scan->ranges.push_back(range);
    //         }
    //         else
    //         {
    //             filtered_scan->ranges.push_back(std::numeric_limits<float>::infinity());
    //         }
    //         current_angle += scan_msg->angle_increment;
    //     }
    // laser_filtered_pub_->publish(*filtered_scan);  // laser_angle_filter အတွက် publisher 


    for (std::size_t i = 0; i < filtered_scan->ranges.size(); i++) {

        double distance = filtered_scan->ranges[i];
        if (std::isnan(distance) || std::isinf(distance) || distance < filtered_scan->range_min) {  // containue if laser data were non and inf and < min_range 
            continue;
        }
        
        double angle = filtered_scan->angle_min + filtered_scan->angle_increment * (double)i; // 
        double distance_derivative = cos(angle) * velocity_x;    // base frame ရဲ့ velocity x နဲ့ အတူ laser_range ကို velocity အပြိုင်ပေးခြင်း
        
        if (distance_derivative > 0 && (distance/distance_derivative) < min_TTC) {   //  min_TTC ကို လျော့
            min_TTC = distance / std::max(distance_derivative, 0.001); // max() သုံးတာက  0 နဲ့ divide တဲ့အခြေအနေရှောင်ဖို့
        }

        //<OR>
        // double ttc = 1.0;
        // if (distance/std::max(velocity_x * cos(filtered_scan->angle_min + (double)i * scan_msg->angle_increment),0.001) < ttc;{

        // }


    }
    // should_brake_ = (min_TTC <= TTC_final_threshold);
    if (min_TTC <= TTC_final_threshold){
        should_brake_ = true;
    }
}

void rom_dynamics::vechicle::AEB::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    this->odom_velocity_x_ = odom_msg->twist.twist.linear.x;
    this->odom_velocity_y_ = odom_msg->twist.twist.linear.y;
}

void rom_dynamics::vechicle::AEB::velocity_subscription(const geometry_msgs::msg::Twist::ConstSharedPtr vel_msg){

    this->input_vel_.linear = vel_msg->linear;
    this->input_vel_.angular = vel_msg->angular;

}
