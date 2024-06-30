#ifndef ROM_DYNAMICS_AEB
#define ROM_DYNAMICS_AEB
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace rom_dynamics
{
    namespace vehicle
    {
        class Aeb
        {
            public:
                Aeb();
                Aeb(double ttc_final);
                Aeb(double ttc_final, double ttc_mid, double ttc_start);
                // double TTC_final_threshold, min_angle , max_angle;
                bool should_brake(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg , double min_angle , double max_angle);
                void setTtcFinalThreshold(double value)
                {
                    ttc_final_threshold_ = value;
                }
                double getTtcFinalThreshold()
                {
                    return ttc_final_threshold_;
                }
                double getMinTtc()
                {
                    return min_ttc_;
                }
                double getDistance() { return distance_; }
                double getVelocity() { return velocity_; }
                
            private:
                geometry_msgs::msg::Twist brake_msg_;
                geometry_msgs::msg::Twist input_vel_;

                std_msgs::msg::Bool brake_status_;
                double ttc_final_threshold_;
                double min_ttc_;
                double ttc_mid_threshold_;
                double ttc_start_threshold_;
                double odom_velocity_x_ = 0.0;
                double odom_velocity_y_ = 0.0;
                bool should_brake_;
                double distance_;
                double velocity_;
       };

    };

};
#endif // ROM_DYNAMICS_AEB