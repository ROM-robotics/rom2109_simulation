#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class DeepBlue : public rclcpp::Node
{
    public:
        DeepBlue() : Node("DeepBlue"), count_(0)
        {
            deepblue_publisher = this->create_publisher<visualization_msgs::msg::Marker>("deepblue_marker",10);

            marker_db.header.frame_id = "map";
            marker_db.header.stamp = this->get_clock().get()->now();
            marker_db.id = 0;
            marker_db.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker_db.action = visualization_msgs::msg::Marker::ADD;

            marker_db.pose.position.x = 3.5; marker_db.pose.position.y = 0; marker_db.pose.position.z = 3;
            marker_db.pose.orientation.x = 0; marker_db.pose.orientation.y = 0; marker_db.pose.orientation.z = 0; marker_db.pose.orientation.w = 1;
            marker_db.text = "Deep Blue AI Lab";
            marker_db.scale.x = 1; marker_db.scale.y = 1; marker_db.scale.z = 1;
            marker_db.color.r = 0.54; marker_db.color.g = 0.95; marker_db.color.b = 0.92; marker_db.color.a = 0.5;

            timer_ = this->create_wall_timer(1000ms, std::bind(&DeepBlue::timer_callback, this));
        }
    private:
        void timer_callback()
        {
            marker_db.action = visualization_msgs::msg::Marker::MODIFY;
            marker_db.header.stamp = this->get_clock().get()->now();
            deepblue_publisher->publish(marker_db);
        }

        visualization_msgs::msg::Marker marker_db;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr deepblue_publisher;

        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeepBlue>());
    rclcpp::shutdown();
    return 0;
}