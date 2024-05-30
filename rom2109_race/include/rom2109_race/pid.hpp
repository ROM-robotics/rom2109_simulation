// Copyright 2024 ROM DYNAMICS TEAM
//
#ifndef ROM_DYNAMIC_PID_CLASS
#define ROM_DYNAMIC_PID_CLASS

#include "rclcpp/rclcpp.hpp"

namespace rom_dynamics
{
    namespace control
    {

        class PID
        {
        public:
            PID();
            void setPID(double kp, double ki, double kd) 
            { 
                kp_ = kp; ki_ = ki; kd_ = kd; 
                RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKp : %.4f\033[1;0m", this->kp_); 
                RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKi : %.4f\033[1;0m", this->ki_); 
                RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKd : %.4f\033[1;0m", this->kd_);
            }
            void setMinMax(double min, double max) 
            { 
                min_ = min; max_ = max; 
                RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32m(Min,Max) : (%.4f,%.4f)\033[1;0m", this->min_, this->max_); 
            }
            void setKp(double kp) 
            { 
                kp_ = kp; 
                RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKp : %.4f\033[1;0m", this->kp_); 
            }
            void setKi(double ki) 
            { 
                ki_ = ki; 
                RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKi : %.4f\033[1;0m", this->ki_); 
            }
            void setKd(double kd) 
            { 
                kd_ = kd; 
                RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKd : %.4f\033[1;0m", this->kd_); 
            }

            double getKp() { return kp_; }
            double getKi() { return ki_; }
            double getKd() { return kd_; }

            double control(double error, double dt);
            double pidControl(double desire, double actual, double dt, double min, double max);
            ~PID();
        private:
            double kp_;
            double ki_;
            double kd_;
            double min_;
            double max_;
            double error_;
        };
        PID::PID() : kp_(1.0), ki_(0), kd_(0), min_(-100000), max_(100000) 
        {
            RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKp : %.4f\033[1;0m", this->kp_); 
            RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKi : %.4f\033[1;0m", this->ki_); 
            RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32mKd : %.4f\033[1;0m", this->kd_); 
            RCLCPP_INFO(rclcpp::get_logger("\033[1;33mPID\033[1;0m"), ": \033[1;32m(Min,Max) : (%.4f,%.4f)\033[1;0m", this->min_, this->max_); 
        }
        double PID::pidControl(double desire, double actual, double dt, double min, double max)
        {
            error_ = desire - actual;
            return control(error_, dt);
        }
        double control(double error, double dt)
        {
            RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;31mBreaking\033[1;0m");
        }
    };
}
#endif 