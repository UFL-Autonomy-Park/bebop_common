// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

#include "rho_filter/RhoFilter.hpp"

using namespace std::chrono_literals;

using PoseMsg = geometry_msgs::msg::PoseStamped;
using OdomMsg = nav_msgs::msg::Odometry;

#define STALE_MOCAP_TIMEOUT_SEC 0.5
#define LOG_THROTTLE_DURATION_MS 500
#define MIN_DT_SEC 1.0e-9

class RhoFilterNode : public rclcpp::Node {
public:
    RhoFilterNode() : Node("rho_filter_node"), data_state_(IDLE) {
        parent_frame = this->declare_parameter<std::string>("frames.parent");
        child_frame = this->declare_parameter<std::string>("frames.child");
        input_topic = this->declare_parameter<std::string>("topics.input_pose");
        k1 = this->declare_parameter<double>("rho_filter_gains.k1");
        k2 = this->declare_parameter<double>("rho_filter_gains.k2");
        k3 = this->declare_parameter<double>("rho_filter_gains.k3");
        k4 = this->declare_parameter<double>("rho_filter_gains.k4");

        rho_ = std::make_unique<RhoFilter>(3, k1, k2, k3, k4);

        pub_rho_ = this->create_publisher<OdomMsg>("filtered_odom", 10);

        sub_pose_ = this->create_subscription<PoseMsg>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&RhoFilterNode::propagate_filters_callback, this, std::placeholders::_1));
        
        odom_msg_template_.header.frame_id = parent_frame;
        odom_msg_template_.child_frame_id = child_frame;
    }

private:
    std::string parent_frame;
    std::string child_frame;
    std::string input_topic;
    double k1;
    double k2;
    double k3;
    double k4;

    enum DataState { IDLE, INITIALIZED, READY };
    DataState data_state_;
    rclcpp::Time last_msg_time_;
    OdomMsg odom_msg_template_;

    std::unique_ptr<RhoFilter> rho_;

    rclcpp::Publisher<OdomMsg>::SharedPtr pub_rho_;
    rclcpp::Subscription<PoseMsg>::SharedPtr sub_pose_;

    void publish_odom(rclcpp::Publisher<OdomMsg>::SharedPtr& pub,
                      const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                      const Eigen::Vector3d& w, const Eigen::Quaterniond& q) 
    {
        OdomMsg odom = odom_msg_template_;
        odom.header.stamp = this->now();
        odom.pose.pose.position = tf2::toMsg(p);
        odom.pose.pose.orientation = tf2::toMsg(q); 

        Eigen::Vector3d v_body = q.inverse() * v;
        odom.twist.twist.linear.x = v_body.x();
        odom.twist.twist.linear.y = v_body.y();
        odom.twist.twist.linear.z = v_body.z();
        odom.twist.twist.angular.x = w.x();
        odom.twist.twist.angular.y = w.y();
        odom.twist.twist.angular.z = w.z();

        pub->publish(odom);
    }

    void propagate_filters_callback(const PoseMsg::SharedPtr msg) {
        rclcpp::Time current_time = this->now();
        
        if (data_state_ != IDLE) {
            if ((current_time - last_msg_time_) > rclcpp::Duration::from_seconds(STALE_MOCAP_TIMEOUT_SEC)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROTTLE_DURATION_MS, "Mocap data stopped publishing.");
            }
        }

        double dt = 0.0;
        if (data_state_ == IDLE) {
            // First message ever: Initialize time, but cannot calculate dt or filter yet
            last_msg_time_ = current_time;
            data_state_ = INITIALIZED;
            return;
        } 
        else {
            // Subsequent messages: Calculate dynamic dt
            dt = (current_time - last_msg_time_).seconds();
            last_msg_time_ = current_time;
            
            // Protect against zero or negative dt (duplicate messages or clock jumps)
            if (dt <= MIN_DT_SEC) {
                return; 
            }

            data_state_ = READY;
        }

        // Filter Processing
        Eigen::Vector3d p_raw;
        Eigen::Quaterniond q_raw;
        tf2::fromMsg(msg->pose.position, p_raw);
        tf2::fromMsg(msg->pose.orientation, q_raw);

        rho_->propagate_filter(p_raw, dt);
        Eigen::Vector3d w_zero = Eigen::Vector3d::Zero();
        publish_odom(
            pub_rho_,
            rho_->get_position_estimate(),
            rho_->get_velocity_estimate(),
            w_zero,
            q_raw
        );
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<RhoFilterNode>());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Node exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}