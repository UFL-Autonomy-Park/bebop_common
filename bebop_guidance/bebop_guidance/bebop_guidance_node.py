# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32


def _desired_trajectory_pos_(t):
    # Trajectory in Mocap frame: X, Y (Up), Z
    return np.array([-1.0 * np.sin(2 * t), 1.0, 3.0 * np.sin(t)])

def _desired_trajectory_vel(t):
    # Analytic derivative of _desired_trajectory_pos_ with respect to t
    return np.array([-2.0 * np.cos(2 * t), 0.0, 3.0 * np.cos(t)])

class BebopGuidance(Node):
    def __init__(self):
        super().__init__('bebop_guidance_node')

        self.control_freq_hz = self._get_double('control_freq_hz')
        self.k1 = self._get_double('control_gains.k1')
        self.k2 = self._get_double('control_gains.k2')
        self.beta = self._get_double('control_gains.beta')
        
        self.k1_y = self._get_double('control_gains.k1_y')
        self.k2_y = self._get_double('control_gains.k2_y')
        self.beta_y = self._get_double('control_gains.beta_y')
        
        self.kp_lat = self._get_double('control_gains.kp_lat')
        self.kp_vert = self._get_double('control_gains.kp_vert')
        self.kp_yaw = self._get_double('control_gains.kp_yaw')
        self.vel_max_mps = self._get_double('control_gains.vel_max_mps')

        # --- Selectors ---
        self.artificial_drift_on = self._get_bool('artificial_drift_on')
        self.controller_type = self._get_str('controller_type')
        self.valid_controller_types = ["proportional", "baseline_rise"]
        if self.controller_type not in self.valid_controller_types:
            raise ValueError(f"Not a valid controller type. Choose from: {self.valid_controller_types}.")

        self.d_out = 3

        self.pose_sub_ = self.create_subscription(PoseStamped, 'pose', self.pose_callback, qos_profile_sensor_data)
        self.odom_sub_ = self.create_subscription(Odometry, 'odom', self.odom_callback, qos_profile_sensor_data)
        self.mode_sub_ = self.create_subscription(Int32, 'mode', self.mode_callback, 10)

        self.control_pub_    = self.create_publisher(Twist, 'cmd_vel_des', 10)
        self.trajectory_pub_ = self.create_publisher(PoseStamped, 'trajectory_setpoint', 10)
        self.tracking_error_pub_ = self.create_publisher(Twist, 'tracking_error', 10)

        self.bebop_sphere_pub_      = self.create_publisher(Marker, 'marker', 10)
        self.trajectory_buffer_pub_ = self.create_publisher(Marker, 'trajectory_buffer', 10)
        self.pose_buffer_pub_       = self.create_publisher(Marker, 'pose_buffer', 10)
        
        visualizer_buffer_size = self._get_int('visualizer_buffer_size')
        self.trajectory_buffer = deque(maxlen=visualizer_buffer_size)
        self.pose_buffer       = deque(maxlen=visualizer_buffer_size)

        self.control_period_seconds = 1.0 / self.control_freq_hz
        self.dt_ = self.control_period_seconds
        self.timer = self.create_timer(self.control_period_seconds, self.timer_callback)

        self.bebop_mode_dict = {0: "teleop", 1: "offboard"}
        self.bebop_mode   = 0
        self.last_pose_time = self.get_clock().now()

        self.traj_period_seconds = 30.0
        self.trajectory_time = 0.0
        
        self.bebop_vel = np.zeros(self.d_out)
        
        self.offboard_active = False
        self._reset_integral_terms()

        self.control_input = Twist()
        self.bebop_pose    = PoseStamped()

        self.get_logger().info('Guidance node initialized.')

    def _get_double(self, name):
        self.declare_parameter(name, rclpy.Parameter.Type.DOUBLE)
        return self.get_parameter(name).get_parameter_value().double_value
    
    def _get_int(self, name):
        self.declare_parameter(name, rclpy.Parameter.Type.INTEGER)
        return self.get_parameter(name).get_parameter_value().integer_value

    def _get_bool(self, name):
        self.declare_parameter(name, rclpy.Parameter.Type.BOOL)
        return self.get_parameter(name).get_parameter_value().bool_value

    def _get_str(self, name):
        self.declare_parameter(name, rclpy.Parameter.Type.STRING)
        return self.get_parameter(name).get_parameter_value().string_value
    
    def _make_point(self, pos) -> Point:
        p = Point()
        p.x, p.y, p.z = float(pos[0]), float(pos[1]), float(pos[2])
        return p

    def _make_line_strip_marker(self, frame_id, points, r, g, b, a=0.3) -> Marker:
        m = Marker()
        m.header.frame_id, m.header.stamp = frame_id, self.get_clock().now().to_msg()
        m.type, m.action, m.scale.x = Marker.LINE_STRIP, Marker.ADD, 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        m.points = points
        return m

    def _make_sphere_marker(self, frame_id, pos) -> Marker:
        m = Marker()
        m.header.frame_id, m.header.stamp = frame_id, self.get_clock().now().to_msg()
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(pos[0]), float(pos[1]), float(pos[2])
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
        return m

    def pose_callback(self, msg):
        self.last_pose_time = self.get_clock().now()
        self.bebop_pose = msg

    def odom_callback(self, msg):
        v = msg.twist.twist.linear
        # Mocap world frame velocity
        self.bebop_vel = np.array([v.x, v.y, v.z])

    def mode_callback(self, msg):
        self.bebop_mode = msg.data
    
    def _reset_integral_terms(self):
        self.integral_term_lat_ = np.zeros(2)
        self.integral_term_vert_ = np.zeros(1)

    def timer_callback(self):
        if not self.bebop_pose.header.frame_id:
            return

        now = self.get_clock().now()
        if (now - self.last_pose_time).nanoseconds >= 1e9:
            self.control_pub_.publish(Twist())
            return

        pos = self.bebop_pose.pose.position
        frame_id = self.bebop_pose.header.frame_id

        self.bebop_sphere_pub_.publish(self._make_sphere_marker(frame_id, [pos.x, pos.y, pos.z]))
        self.pose_buffer.append(self._make_point([pos.x, pos.y, pos.z]))
        self.pose_buffer_pub_.publish(self._make_line_strip_marker(frame_id, list(self.pose_buffer), 0.0, 1.0, 0.0))

        if self.bebop_mode_dict.get(self.bebop_mode, "teleop") == "teleop":
            self.offboard_active = False
            return

        if not self.offboard_active:
            self._reset_integral_terms()
            self.offboard_active = True
            self.get_logger().info("Offboard tracking (re)started. Resuming trajectory clock.")

        # --- Desired Trajectory ---
        self.trajectory_time += self.dt_  # Only ticks forward while offboard is active
        elapsed_seconds = self.trajectory_time
        scale_factor = (2.0 * np.pi / self.traj_period_seconds)
        time_scaled = scale_factor * elapsed_seconds

        pos_setpoint = np.array(_desired_trajectory_pos_(time_scaled))
        traj_deriv   = np.array(_desired_trajectory_vel(time_scaled)) * scale_factor

        # --- Errors (e, e_dot, r) ---
        x_3d = np.array([pos.x, pos.y, pos.z])
        e_3d = np.array(pos_setpoint) - x_3d
        e_dot_3d = np.array(traj_deriv) - self.bebop_vel

        e_lat  = np.array([e_3d[0], e_3d[2]])
        e_vert = np.array([e_3d[1]])

        r_lat  = np.array([e_dot_3d[0], e_dot_3d[2]]) + self.k1 * e_lat
        r_vert = np.array([e_dot_3d[1]]) + self.k1_y * e_vert
        r_3d   = np.array([r_lat[0], r_vert[0], r_lat[1]])

        traj_deriv_lat  = np.array([traj_deriv[0], traj_deriv[2]])
        traj_deriv_vert = np.array([traj_deriv[1]])

        # --- Control Law & Integral Updates ---
        if self.controller_type == "proportional":
            cmd_vel_lat  = self.kp_lat * e_lat #+ traj_deriv_lat
            cmd_vel_vert = self.kp_vert * e_vert #+ traj_deriv_vert
        else:
            int_dot_lat  = self.beta * np.sign(e_lat) + (1 + self.k1 * self.k2) * e_lat
            int_dot_vert = self.beta_y * np.sign(e_vert) + (1 + self.k1_y * self.k2_y) * e_vert

            self.integral_term_lat_  += int_dot_lat * self.dt_
            self.integral_term_vert_ += int_dot_vert * self.dt_

            cmd_vel_lat  = (self.k1 + self.k2) * e_lat + self.integral_term_lat_ #+ traj_deriv_lat
            cmd_vel_vert = (self.k1_y + self.k2_y) * e_vert + self.integral_term_vert_ #+ traj_deriv_vert

        # Artificial drift
        cmd_vel = np.array([cmd_vel_lat[0], cmd_vel_vert[0], cmd_vel_lat[1]])
        if self.artificial_drift_on:
            cmd_vel += self.artificial_drift(drone_position=x_3d)

        # --- Saturation & Publishing ---
        norm = np.linalg.norm(cmd_vel)
        scale = min(1.0, self.vel_max_mps / norm) if norm > 0 else 1.0
        self.control_input.linear.x, self.control_input.linear.y, self.control_input.linear.z = (np.array(cmd_vel) * scale).tolist()

        q = self.bebop_pose.pose.orientation
        rot_body_to_mocap = Rotation.from_quat(quat=[q.x, q.y, q.z, q.w], scalar_first=False)
        rot_mocap_to_z_up = Rotation.from_euler('x', 90, degrees=True) 
        rot_z_up_to_body = rot_body_to_mocap.inv() * rot_mocap_to_z_up
        _, _, current_yaw_cw = rot_z_up_to_body.as_euler('xyz', degrees=False)

        yaw_setpoint_cw = np.arctan2(traj_deriv[2], traj_deriv[0]) 
        err_yaw = (yaw_setpoint_cw - current_yaw_cw + np.pi) % (2 * np.pi) - np.pi 
        
        self.control_input.angular.y = float(-self.kp_yaw * err_yaw)
        self.control_pub_.publish(self.control_input)

        desired_pose_msg = PoseStamped()
        desired_pose_msg.header.stamp, desired_pose_msg.header.frame_id = now.to_msg(), 'world'
        desired_pose_msg.pose.position.x, desired_pose_msg.pose.position.y, desired_pose_msg.pose.position.z = pos_setpoint.tolist()

        r = Rotation.from_euler('y', -yaw_setpoint_cw, degrees=False) 
        qx, qy, qz, qw = r.as_quat()
        desired_pose_msg.pose.orientation.x, desired_pose_msg.pose.orientation.y, desired_pose_msg.pose.orientation.z, desired_pose_msg.pose.orientation.w = float(qx), float(qy), float(qz), float(qw)
        self.trajectory_pub_.publish(desired_pose_msg)

        tracking_err_msg = Twist()
        tracking_err_msg.linear.x, tracking_err_msg.linear.y, tracking_err_msg.linear.z = np.array(e_3d).tolist()
        tracking_err_msg.angular.y = float(err_yaw)
        self.tracking_error_pub_.publish(tracking_err_msg)

        self.trajectory_buffer.append(self._make_point(pos_setpoint))
        self.trajectory_buffer_pub_.publish(self._make_line_strip_marker(frame_id, list(self.trajectory_buffer), 1.0, 0.0, 0.0))

    @staticmethod
    def artificial_drift(drone_position: np.ndarray) -> np.ndarray:
        """
        Calculates a spatially varying 3D drift field bounded at 0.5 m/s.
        Lab Dimensions: x=13.5, y=2.75, z=5.5
        """
        # Lab dimensions (meters)
        L = np.array([13.5, 2.75, 5.5])
        x, y, z = drone_position

        # Generate a complex, spatially dependent raw vector
        # vx varies primarily with length (x)
        # vy creates a coupling between lateral position and longitudinal progress
        # vz creates a vertical gust depending on height
        vx = 0.4 * np.sin(2 * np.pi * x / L[0])
        vy = 0.4 * np.cos(np.pi * y / L[1] + np.pi * x / L[0])
        vz = 0.4 * np.sin(np.pi * z / L[2])

        drift_raw = np.array([vx, vy, vz])

        # 2-norm normalization to strictly enforce the 0.5 m/s limit
        magnitude = np.linalg.norm(drift_raw)
        max_magnitude = 0.5
        
        # If magnitude > 0.5, scale it down; otherwise, leave it alone.
        scale = max_magnitude / magnitude if magnitude > max_magnitude else 1.0
        
        return drift_raw * scale

def main(args=None):
    rclpy.init(args=args)
    node = BebopGuidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()