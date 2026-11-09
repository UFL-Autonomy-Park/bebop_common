// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

class BebopTeleop : public rclcpp::Node {
public:
    BebopTeleop() : Node("bebop_teleop_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Bebop Teleop Node");

        this->declare_parameter("x_axis", -1);
        this->declare_parameter("y_axis", -1);
        this->declare_parameter("z_axis", -1);
        this->declare_parameter("yaw_axis", -1);
        this->declare_parameter("takeoff_button", -1);
        this->declare_parameter("land_button", -1);
        this->declare_parameter("mode_button", -1);

        this->declare_parameter("x_vel_max", 1.0);
        this->declare_parameter("y_vel_max", 1.0);
        this->declare_parameter("z_vel_max", 1.0);
        this->declare_parameter("yaw_vel_max", 1.0);

        this->get_parameter("x_axis", axes_.x.axis);
        this->get_parameter("y_axis", axes_.y.axis);
        this->get_parameter("z_axis", axes_.z.axis);
        this->get_parameter("yaw_axis", axes_.yaw.axis);

        this->get_parameter("takeoff_button", buttons_.takeoff.button);
        this->get_parameter("land_button", buttons_.land.button);
        this->get_parameter("mode_button", buttons_.mode.button); 

        this->get_parameter("x_vel_max", axes_.x.factor);
        this->get_parameter("y_vel_max", axes_.y.factor);
        this->get_parameter("z_vel_max", axes_.z.factor);
        this->get_parameter("yaw_vel_max", axes_.yaw.factor);

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        takeoff_pub_ = this->create_publisher<std_msgs::msg::Empty>("takeoff", 10);
        land_pub_ = this->create_publisher<std_msgs::msg::Empty>("land", 10);
        mode_pub_ = this->create_publisher<std_msgs::msg::Int32>("mode", 10);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&BebopTeleop::joy_callback, this, _1));

        setpoint_timer_ = this->create_wall_timer(20ms, std::bind(&BebopTeleop::main_callback, this));

        RCLCPP_INFO(this->get_logger(), "Loaded controller parameters:\nX: %d, Y: %d, Z: %d, Yaw: %d, Takeoff: %d, Land: %d, Mode %d", 
            axes_.x.axis, axes_.y.axis, axes_.z.axis, axes_.yaw.axis, buttons_.takeoff.button, buttons_.land.button, buttons_.mode.button);
        
        std::string mode_str = (flight_state_ == FlightState::TELEOP) ? "TELEOP" : "OFFBOARD";
        RCLCPP_INFO_STREAM(this->get_logger(), "Control mode set to " << mode_str);
    }

private:
    struct Axis {
        Axis() : axis(0), factor(0.0), offset(0.0) {}
        int axis;
        double factor;
        double offset;
    };

    struct {
        Axis x;
        Axis y;
        Axis z;
        Axis yaw;
    } axes_;

    struct Button {
        Button() : button(0) {}
        int button;
    };

    struct {
        Button takeoff;
        Button land;
        Button mode;
    } buttons_;

    // Button state for debouncing
    struct ButtonState {
        ButtonState() : state(0) {}
        int state;
    };

    struct {
        ButtonState takeoff;
        ButtonState land;
        ButtonState mode;
    } button_state_;

    enum class FlightState {
        TELEOP = 0,
        OFFBOARD = 1,
    };
    
    FlightState flight_state_ = FlightState::TELEOP;

    geometry_msgs::msg::Twist setpoint_vel_desired_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeoff_pub_, land_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr setpoint_timer_;

    double get_joy_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Axis &axis) {
        if (axis.axis < 0 || std::abs(axis.axis) > (int)joy_msg->axes.size()-1) {
            RCLCPP_ERROR(this->get_logger(), "Axis %d out of range, joy has %d axes", axis.axis, (int)joy_msg->axes.size());
            return -1;
        }
        double output = joy_msg->axes[std::abs(axis.axis)] * axis.factor + axis.offset;
        return output;
    }

    int get_joy_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Button &button) {
        if (button.button < 0 || button.button > (int)joy_msg->buttons.size()-1) {
            RCLCPP_ERROR(this->get_logger(), "Button %d out of range, joy has %d buttons", button.button, (int)joy_msg->buttons.size());
            return -1;
        }
        return joy_msg->buttons[button.button];
    }

    void request_takeoff() {
        if (this->flight_state_ == FlightState::TELEOP) {
            RCLCPP_WARN(this->get_logger(), "Takeoff requested");
            std_msgs::msg::Empty takeoff_msg;
            takeoff_pub_->publish(takeoff_msg);
        } else if (this->flight_state_ == FlightState::OFFBOARD) {
            RCLCPP_WARN(this->get_logger(), "Cannot take off in OFFBOARD mode");
        }
    }

    void request_landing() {
        RCLCPP_WARN(this->get_logger(), "Landing requested");
        if (this->flight_state_ == FlightState::OFFBOARD) {
            mode_switch();
        }

        geometry_msgs::msg::Twist cmd_vel_zero;
        cmd_vel_zero.linear.x = 0.0;
        cmd_vel_zero.linear.y = 0.0;
        cmd_vel_zero.linear.z = 0.0;
        cmd_vel_zero.angular.z = 0.0;
        for(int i=0; i<5; i++) {
            vel_pub_->publish(cmd_vel_zero);
            rclcpp::sleep_for(20ms);
        }
        std_msgs::msg::Empty land_msg;
        land_pub_->publish(land_msg);
    }

    void mode_switch() {
        if (this->flight_state_ == FlightState::TELEOP) {
            RCLCPP_WARN(this->get_logger(), "Switching to OFFBOARD");
            this->flight_state_ = FlightState::OFFBOARD;
        } else {
            RCLCPP_WARN(this->get_logger(), "Switching to TELEOP");
            this->flight_state_ = FlightState::TELEOP;
            reset_velocity_setpoints();
        }
    }

    void reset_velocity_setpoints() {
        setpoint_vel_desired_.linear.x = 0.0;
        setpoint_vel_desired_.linear.y = 0.0;
        setpoint_vel_desired_.linear.z = 0.0;
        setpoint_vel_desired_.angular.z = 0.0;
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        int takeoff_button_state = get_joy_button(joy_msg, buttons_.takeoff);
        if (takeoff_button_state != button_state_.takeoff.state) {
            if (takeoff_button_state == 1) {
                RCLCPP_INFO(this->get_logger(), "Takeoff button pressed");
                request_takeoff();
            }
            button_state_.takeoff.state = takeoff_button_state;
        }

        int land_button_state = get_joy_button(joy_msg, buttons_.land);
        if (land_button_state != button_state_.land.state) {
            if (land_button_state == 1) {
                RCLCPP_INFO(this->get_logger(), "Land button pressed");
                request_landing();
            }
            button_state_.land.state = land_button_state;
        }

        int mode_button_state = get_joy_button(joy_msg, buttons_.mode);
        if (mode_button_state != button_state_.mode.state) {
            if (mode_button_state == 1) {
                RCLCPP_INFO(this->get_logger(), "Mode button pressed");
                mode_switch();
            }
            button_state_.mode.state = mode_button_state;
        }

        setpoint_vel_desired_.linear.x = get_joy_axis(joy_msg, axes_.x);
        setpoint_vel_desired_.linear.y = get_joy_axis(joy_msg, axes_.y);
        setpoint_vel_desired_.linear.z = get_joy_axis(joy_msg, axes_.z);
        setpoint_vel_desired_.angular.z = get_joy_axis(joy_msg, axes_.yaw);
    }

    void main_callback() {
        std_msgs::msg::Int32 mode_msg;
        mode_msg.data = static_cast<int>(this->flight_state_);
        mode_pub_->publish(mode_msg);

        if (this->flight_state_ == FlightState::TELEOP) {
            vel_pub_->publish(setpoint_vel_desired_);        
        }       
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BebopTeleop>());
    rclcpp::shutdown();
    return 0;
}
