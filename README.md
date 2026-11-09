<h1 align="center"> Bebop Common
</h1>
<div align="center">

![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)

A ROS 2 metapackage providing the common infrastructure needed to fly a Parrot Bebop 2 in the Indoor Lab with OptiTrack motion capture.
It resembles a flight stack, with a driver, mocap bridge, low-level velocity control, teleoperation, filtering and visualization. Off-board research code sits on top of `bebop_bringup`; `bebop_guidance` is a working example.

> [!IMPORTANT]
> This repo supports a **single agent only** (one Bebop at a time). Multi-agent flight is not supported.

</div>

**Requirements:** Ubuntu 22.04, ROS 2 Humble.

## Packages

| Name | Description |
|---|---|
| bebop_bringup | Launch file that starts the full single-agent stack. Requires a `params_file`. |
| bebop_control | Velocity-level PI controller; turns a desired velocity and odometry into Bebop `cmd_vel`. |
| bebop_teleop | Maps a console controller (Xbox / DualShock) to Bebop actions and mode switching. |
| indoor_lab_viz | Publishes the lab's flight-volume borders for RViz. |
| rho_filter | Exponentially convergent first-order differentiator (math by Cristian Nino). |
| bebop_guidance | **Example** off-board RISE trajectory-tracking controller. Copy it as a template for your own. |
| ros2_parrot_arsdk | colcon wrapper that builds the Parrot ARSDK. Submodule, fork of [jeremyfix/ros2_parrot_arsdk](https://github.com/jeremyfix/ros2_parrot_arsdk). |
| ros2_bebop_driver | ROS 2 Bebop driver. Submodule, fork of [jeremyfix/ros2_bebop_driver](https://github.com/jeremyfix/ros2_bebop_driver). |
| vrpn_mocap | VRPN client for OptiTrack Motive. Submodule, fork of [alvinsunyixiao/vrpn_mocap](https://github.com/alvinsunyixiao/vrpn_mocap). |

The forks keep their upstream licenses (BSD-3 / MIT) and are not edited here. Everything else is [MPL 2.0](LICENSE).

```
mocap ─► vrpn_mocap ─► rho_filter ─► your off-board node ─► bebop_control ─► ros2_bebop_driver ─► Bebop
                                      (e.g. bebop_guidance)       ▲
                                                          bebop_teleop (mode + manual)
```

## Installation

```bash
mkdir -p ~/<your_ws_name>/src
cd ~/<your_ws_name>/src
git clone --recurse-submodules https://github.com/UFL-Autonomy-Park/bebop_common.git
```

The three forks (`ros2_parrot_arsdk`, `ros2_bebop_driver`, `vrpn_mocap`) are git submodules and are empty after a plain clone. To pull them in:
```bash
git submodule update --init --recursive
```

### System dependencies

Run these before the first build:

```bash
# ros2_parrot_arsdk
sudo apt install libavahi-client-dev

# ros2_bebop_driver
sudo apt install ros-humble-camera-info-manager libavdevice-dev

# vrpn_mocap and any other missing ROS dependencies
cd ~/<your_ws_name>
rosdep install --from-paths src -y --ignore-src

# Required, or the ARSDK build hangs (see below)
git config --global color.ui auto
```

**Why `color.ui`?** `ros2_parrot_arsdk` downloads Google's `repo` tool, which asks an interactive question about colored output on first run. Under `colcon` that prompt is invisible and the build hangs. This setting answers it ahead of time. If the build still hangs, answer the prompt once by hand, then rebuild:

```bash
cd ~/<your_ws_name>/src/bebop_common/ros2_parrot_arsdk
mkdir build && cd build && cmake .. && make
```

### Build

```bash
cd ~/<your_ws_name>
colcon build --symlink-install
source install/setup.bash
```

The first build takes a few minutes and needs internet access (the ARSDK is downloaded and compiled). `ros2_bebop_driver` depends on `ros2_parrot_arsdk`; colcon orders them for you.

## Usage

Every node here declares its parameters without defaults, so you must supply a parameter file. `bebop_guidance/config/main.yaml` is a complete, tuned example.

**Run the example:**
```bash
ros2 launch bebop_guidance main.launch.py bebop_number:=106 joy_type:=xbox
```

**Run just the common stack** (for your own off-board node):
```bash
ros2 launch bebop_bringup bringup.launch.py params_file:=/abs/path/to/your.yaml bebop_number:=106
```

`bebop_number` sets the namespace (`bebop<N>`) and the drone's IP (`192.168.1.<N>`). Pass `rviz_config:=<file>` to open RViz, and `save_data:=false` to skip the rosbag.

**Writing your own off-board layer:** copy `bebop_guidance`, replace the node, and keep the interface. Subscribe to `pose` (remapped to `/vrpn_mocap/<ns>/pose`), `odom` and `mode`; publish the desired velocity on `cmd_vel_des`. The mode is `0` for teleop and `1` for off-board.

## Docs

- [Coordinate frames and conventions](docs/coordinate_frames.md): read this before writing any controller.

## Known issues

See the [issue tracker](https://github.com/UFL-Autonomy-Park/bebop_common/issues).
