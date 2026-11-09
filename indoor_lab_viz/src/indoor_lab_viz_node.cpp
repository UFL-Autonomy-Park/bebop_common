// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <chrono>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

using namespace std::chrono_literals;

class IndoorLabViz : public rclcpp::Node {
public:
    IndoorLabViz() : Node("indoor_lab_viz_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Indoor Lab Visualization Node");

        std::vector<double> temp;
        this->declare_parameter("border_x", temp);
        this->declare_parameter("border_z", temp);

        std::vector<double> border_x, border_z;
        this->get_parameter("border_x", border_x);
        this->get_parameter("border_z", border_z);

        if (border_x.size() != 4 || border_z.size() != 4) {
            RCLCPP_ERROR(this->get_logger(), "(IndoorLabViz) Border X and Y must be length 4.");
        }

        low_border_poly_msg_.header.frame_id = high_border_poly_msg_.header.frame_id = "odom";

        rclcpp::Time now = this->get_clock()->now();
        low_border_poly_msg_.header.stamp = now;
        high_border_poly_msg_.header.stamp = now;

        for (size_t i = 0; i < border_x.size(); i++) {
            geometry_msgs::msg::Point32 p_low, p_high;

            p_low.x = p_high.x = border_x[i];
            p_low.z = p_high.z = border_z[i];
            p_low.y = 0.0;
            p_high.y = 2.7432;

            low_border_poly_msg_.polygon.points.push_back(p_low);
            high_border_poly_msg_.polygon.points.push_back(p_high);
        }

        low_border_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("low_border", 1);
        high_border_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("high_border", 1);

        timer_ = this->create_wall_timer(1s, std::bind(&IndoorLabViz::timer_callback, this));
    }

private:
    geometry_msgs::msg::PolygonStamped low_border_poly_msg_, high_border_poly_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr low_border_publisher_, high_border_publisher_;

    void timer_callback() {
        this->low_border_publisher_->publish(low_border_poly_msg_);
        this->high_border_publisher_->publish(high_border_poly_msg_);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IndoorLabViz>());
    rclcpp::shutdown();
    return 0;
}
