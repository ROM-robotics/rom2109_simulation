#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <geometry_msgs/msg/point.hpp>
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class RomRobotics : public rclcpp::Node
{
    public:
        RomRobotics() : Node("RomRobotics"), count_(0)
        {
            r_pub = this->create_publisher<visualization_msgs::msg::Marker>("r_marker",10);
            g_pub = this->create_publisher<visualization_msgs::msg::Marker>("g_marker",10);
            b_pub = this->create_publisher<visualization_msgs::msg::Marker>("b_marker",10);
            
            init_marker_values(r_marker); 
            g_marker = r_marker; b_marker = r_marker;

            r_marker.color.r = 1.0; // red
            g_marker.color.g = 1.0; // green

            b_marker.color.r = 1.0; // red+green=yellow
            b_marker.color.g = 1.0;

            push_data();

            timer_ = this->create_wall_timer(1000ms, std::bind(&RomRobotics::timer_callback, this));
        }

        void init_marker_values(visualization_msgs::msg::Marker &marker) 
        {
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock().get()->now();
            marker.ns = "ROM ROBOTICS";
            marker.id = 0;
            
            marker.type = visualization_msgs::msg::Marker::SPHERE_LIST; //SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;

            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            marker.points.clear();

            
        }
        void push_data()
        {
            for(double x=7.0;x<x_max;x+=density){
	        		for(double y=y_min;y<y_max;y+=density){
	        			for(double z=z_min;z<(z_min+0.66);z+=density){
	        				my_point.x = x;
	        				my_point.y = y;
	        				my_point.z = z;
	        				r_marker.points.push_back(my_point);
	        			}
	        		}
	        	}  
                for(double x=7.0;x<x_max;x+=density){
                    for(double y=y_min;y<y_max;y+=density){
                        for(double z=z_min+0.66;z<(z_min+1.33);z+=density){
                            my_point.x = x;
                            my_point.y = y;
                            my_point.z = z;
                            g_marker.points.push_back(my_point);
                        }
                    }
                }
                for(double x=7.0;x<x_max;x+=density){
                    for(double y=y_min;y<y_max;y+=density){
                        for(double z=z_min+1.33;z<z_max;z+=density){
                            my_point.x = x;
                            my_point.y = y;
                            my_point.z = z;
                            b_marker.points.push_back(my_point);
                        }
                    }
                }
        }
    private:
        void timer_callback()
        {
            count_++;
            if(count_ > 20) 
            {
                auto marker_time = this->get_clock().get()->now();
                r_marker.header.stamp = marker_time;
                g_marker.header.stamp = marker_time;
                b_marker.header.stamp = marker_time;

                r_pub->publish(r_marker);
                g_pub->publish(g_marker);
                b_pub->publish(b_marker);
            }
            
        }

        rclcpp::TimerBase::SharedPtr timer_;

        visualization_msgs::msg::Marker r_marker; // instantiate a marker object
        visualization_msgs::msg::Marker g_marker;
        visualization_msgs::msg::Marker b_marker;

        geometry_msgs::msg::Point my_point;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr r_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr g_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr b_pub;

        size_t count_;

        double x_min = 7.0;
        double x_max = 9.0;
        double y_min = 0.0;
        double y_max = 2.0;
        double density = 0.05;
        double z_min = 8.0;
        double z_max = 10.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RomRobotics>());
  rclcpp::shutdown();
  return 0;
}