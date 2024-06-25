// Copyright 2024 ROM DYNAMICS TEAM
//
#ifndef ROM_DYNAMIC_AEB_CLASS
#define ROM_DYNAMIC_AEB_CLASS

#include "rclcpp/rclcpp.hpp"

namespace rom_dynamics
{
    namespace vechicle
    {

        class AEB
        {
        public:
            AEB();
            
            bool shouldBerak(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
            {
                // laser filtering
                computeAEB();
            }
            double computeAEB(double desire, double actual, double dt, double min, double max);
            ~AEB();
        private:
            double odom_x;
            double odom_y;
            scan
            bool should_break_;
            
        };
        AEB::AEB() : 
        {
            //RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKp : %.4f\033[1;0m", this->kp_); 
            //RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKi : %.4f\033[1;0m", this->ki_); 
            //RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKd : %.4f\033[1;0m", this->kd_); 
        }
        double AEB::computeAEB()
        {
            
            return pidTerm;
        }
        AEB::~AEB() {}
    };
}
#endif 