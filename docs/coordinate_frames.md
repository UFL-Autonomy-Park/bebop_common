# Coordinate frames and conventions

Trajectory-tracking code relies on explicit frame bookkeeping. This stack assumes the following conventions; the Bebop SDK is mapped to them by `ros2_bebop_driver`.

## Mocap native frame (M)
- `X` = West
- `Y` = Up
- `Z` = North

Looking down from above (Y up), rotating from West (X) to North (Z) is a **90° clockwise rotation**. Computing yaw as `arctan2(North, West)` therefore gives a **clockwise-positive** yaw angle.

## Drone rigid-body frame (B)
- `X` = Front
- `Y` = Left
- `Z` = Up

This is the standard ROS FLU (Front-Left-Up) frame.

## Bebop SDK command map
Controllers publish desired velocities on `cmd_vel`. The SDK maps them to the physical inputs:
- `linear.x`: pitch angle (positive = forward acceleration)
- `linear.y`: roll angle (positive = left acceleration)
- `linear.z`: vertical velocity (positive = up)
- `angular.y`: yaw setpoint (used historically; positive = counter-clockwise)

## 3D body-frame error projection
Velocity setpoints from the mocap frame are projected into the full 3D body frame, so errors are measured along the body X and Y axes where the drone actually applies pitch and roll. When the drone pitches aggressively, the `rot_body_to_mocap.inverse()` projection decomposes the "world up" requirement into body X and Z, which prevents altitude loss.
