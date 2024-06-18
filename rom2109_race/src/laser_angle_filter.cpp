#include "rom2109_race/laser_angle_filter.hpp"


LaserAngleFilter::LaserAngleFilter() : Node("lidar_filter_node"){
    
        

        // subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        //     "/scan", 10, std::bind(&LaserAngleFilter::filter_angle, this, std::placeholders::_1));   if we leave that uncommand, that cause type error.(from rclcpp::any_back.cpp)(to read)

        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan", 10);     // laser_angle_filter အတွက် publisher 
    }


    sensor_msgs::msg::LaserScan::SharedPtr LaserAngleFilter::filter_angle(const sensor_msgs::msg::LaserScan::SharedPtr msg , double min_angle=-60.0 , double max_angle=60.0){

        // double min_angle = this->get_parameter("min_angle").as_double();   //laser rays ရဲ့ angle ကို dynamically ချိန်ညှိနိုင်ရန် နှင့် parameter သတ်မှတ်ရလွယ်ကူစေရန်
        // double max_angle = this->get_parameter("max_angle").as_double();

        // if( !this->has_parameter("min_angle") && !this->has_parameter("max_angle") )
        // {
        //     this->declare_parameter("min_angle", -90.0);
        //     this->declare_parameter("max_angle", 90.0);
        // }

        // double min_angle = this->get_parameter("min_angle").as_double();   //laser rays ရဲ့ angle ကို dynamically ချိန်ညှိနိုင်ရန် နှင့် parameter သတ်မှတ်ရလွယ်ကူစေရန်
        // double max_angle = this->get_parameter("max_angle").as_double();

        

        auto const filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
        filtered_scan->ranges.clear();

        double angle_min = min_angle * M_PI / 180.0;  // -60 degrees in radians
        double angle_max = max_angle * M_PI / 180.0;   // 60 degrees in radians
        double current_angle = msg->angle_min;

        for (const auto &range : msg->ranges)
        {
            if (current_angle >= angle_min && current_angle <= angle_max)
            {
                filtered_scan->ranges.push_back(range);
            }
            else
            {
                filtered_scan->ranges.push_back(std::numeric_limits<double>::infinity());
            }
            current_angle += msg->angle_increment;
        }

        publisher_->publish(*filtered_scan);  // laser_angle_filter အတွက် publisher 

        return filtered_scan;
    }

    

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<LaserAngleFilter>());
//     rclcpp::shutdown();
//     return 0;
// }
