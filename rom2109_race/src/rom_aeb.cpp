#include "rom2109_race/rom_aeb.hpp"

using namespace std::chrono_literals;

rom_dynamics::vechicle::Aeb::Aeb()
{
    ttc_final_threshold_ = 1.0;
}
rom_dynamics::vechicle::Aeb::Aeb(double ttc_final)
{
    ttc_final_threshold_ = ttc_final;
}

bool rom_dynamics::vechicle::Aeb::should_brake(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg , double min_angle , double max_angle)
{
    double min_TTC = 1000000.0;
    double velocity_x = odom_msg->twist.twist.linear.x;

    //LASER ANGLE FILTER 
    auto const filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan_msg);
    filtered_scan->ranges.clear();

    double angle_min = min_angle * M_PI / 180.0;  //degree to radian
    double angle_max = max_angle * M_PI / 180.0;  //degree to radian
    double current_angle = scan_msg->angle_min;

    for (const auto &range : scan_msg->ranges)
    {
        if (current_angle >= angle_min && current_angle <= angle_max)
        {
            filtered_scan->ranges.push_back(range);
        }
        else
        {
            filtered_scan->ranges.push_back(std::numeric_limits<double>::infinity());
        }
        current_angle += scan_msg->angle_increment;
    }

    //CALCULATE AEB 
    for (std::size_t i = 0; i < filtered_scan->ranges.size(); i++) 
    {

        double distance = filtered_scan->ranges[i];
        if (std::isnan(distance) || std::isinf(distance) || distance < filtered_scan->range_min) 
        {  // containue if laser data were non and inf and < min_range 
            continue;
        }

        double angle = filtered_scan->angle_min + filtered_scan->angle_increment * (double)i; 
        // base frame ရဲ့ velocity x နဲ့ အတူ laser_range ကို velocity အပြိုင်ပေးခြင်း
        // distance ကို ရှိတ်တဲ့ velocity တန်ဖိုးကို cos တွက်ပြီး weight အနေနဲ့သုံးတယ်။
        double distance_derivative = cos(angle) * velocity_x;    
        
        // v = s/t , t = s/v
        // velocity 0 ထက်ကြီးပြီး time( obstacle ဆီရောက်မည့်အချိန် ) က minttc ထက်ငယ်ရင် 
        if (distance_derivative > 0 && (distance/distance_derivative) < min_TTC) 
        {   //  min_TTC ကို လျော့
            //min_TTC = distance / std::max(distance_derivative, 0.001); // max() သုံးတာက  0.001 ထက်မငယ်စေဖို့
            min_TTC = distance / distance_derivative;
        }

        // တကြောင်းထဲတွက်
        // if (distance/std::max(velocity_x * cos(filtered_scan->angle_min + (double)i * scan_msg->angle_increment),0.001) < ttc;


    }
    if (min_TTC <= ttc_final_threshold_)
    {
        should_brake_ = true;
    }

    return should_brake_;

}