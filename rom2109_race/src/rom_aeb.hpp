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
            
            
            double computeAEB(double desire, double actual, double dt, double min, double max);
            ~AEB();
        private:
            
        };
        AEB::AEB() : 
        {
            //RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKp : %.4f\033[1;0m", this->kp_); 
            //RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKi : %.4f\033[1;0m", this->ki_); 
            //RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKd : %.4f\033[1;0m", this->kd_); 
        }
        double AEB::computeAEB(double desire, double actual, double dt, double min, double max)
        {
            
            return pidTerm;
        }
        AEB::~AEB() {}
    };
}
#endif 