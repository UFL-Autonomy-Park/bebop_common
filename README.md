# bebop_common

Shared ROS 2 workspace sources for flying Parrot Bebop 2 drones in the Indoor Lab: driver, motion-capture bridge, controller, teleop, filter and visualization.

**Requirements:** Ubuntu 22.04, ROS 2 Humble.

## Packages

| Package | Description | Origin | License |
| --- | --- | --- | --- |
| `bebop_control` | Velocity-level PID controller; turns desired velocity + odometry into Bebop `cmd_vel` | this repo | MPL-2.0 |
| `bebop_teleop` | Maps a console controller (Xbox / DualShock) to Bebop actions. May need launch-file updates | this repo | MPL-2.0 |
| `indoor_lab_viz` | Publishes the lab's flight-volume borders and an RViz config | this repo | MPL-2.0 |
| `rho_filter` | Exponentially convergent first-order differentiator for noisy signals (math by Cristian Nino) | submodule ([rho_filter](https://github.com/UFL-Autonomy-Park/rho_filter)) | MPL-2.0 |
| `ros2_parrot_arsdk` | colcon wrapper that downloads and builds the Parrot ARSDK | submodule, fork of [jeremyfix/ros2_parrot_arsdk](https://github.com/jeremyfix/ros2_parrot_arsdk) | BSD-3 |
| `ros2_bebop_driver` | ROS 2 rewrite of `bebop_autonomy`; publishers/subscribers for core Bebop functionality | submodule, fork of [jeremyfix/ros2_bebop_driver](https://github.com/jeremyfix/ros2_bebop_driver) | BSD-3 |
| `vrpn_mocap` | Turns VRPN messages from OptiTrack Motive into ROS 2 messages | submodule, fork of [alvinsunyixiao/vrpn_mocap](https://github.com/alvinsunyixiao/vrpn_mocap) | MIT |

The first three packages live directly in this repo. The other four are git submodules. Do not edit the three forks (`ros2_parrot_arsdk`, `ros2_bebop_driver`, `vrpn_mocap`) here; this repo only pins them.

## Installation

```bash
mkdir -p ~/<your_ws_name>/src
cd ~/<your_ws_name>/src
git clone --recurse-submodules https://github.com/UFL-Autonomy-Park/bebop_common.git
```

If you already cloned without `--recurse-submodules`:

```bash
git submodule update --init --recursive
```

### System dependencies

Run all of these before the first build:

```bash
# ros2_parrot_arsdk
sudo apt install libavahi-client-dev

# ros2_bebop_driver
sudo apt install ros-humble-camera-info-manager libavdevice-dev

# vrpn_mocap (and any other missing ROS dependencies, e.g. vrpn)
cd ~/<your_ws_name>
rosdep install --from-paths src -y --ignore-src

# Required, or the ARSDK build hangs (see below)
git config --global color.ui auto
```

**Why `color.ui`?** `ros2_parrot_arsdk` downloads Google's `repo` tool, which asks an interactive question about colored output the first time it runs. Under `colcon` that prompt is invisible and the build hangs. `git config --global color.ui auto` answers it ahead of time.

If the ARSDK build still hangs, answer the prompt manually once, then rebuild with colcon:

```bash
cd ~/<your_ws_name>/src/bebop_common/ros2_parrot_arsdk
mkdir build && cd build
cmake .. && make
```

### Build

```bash
cd ~/<your_ws_name>
colcon build --symlink-install
source install/setup.bash
```

The first build takes a few minutes, mostly `ros2_parrot_arsdk` (it downloads and compiles the Parrot SDK, so it needs internet access). It will not work offline. `ros2_bebop_driver` depends on `ros2_parrot_arsdk`, and colcon orders them for you. To rebuild just your own packages: `colcon build --symlink-install --packages-select bebop_control bebop_teleop indoor_lab_viz rho_filter`.

## Running the forked packages

**Bebop driver** (connect to the drone's Wi-Fi first; the default IP is `192.168.42.1`):

```bash
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=<BEBOP.IP.ADDRESS> namespace:=bebop
```

To debug under gdb:

```bash
ros2 run --prefix 'gdb -ex run --args' ros2_bebop_driver bebop_driver --ros-args -p bebop_ip:=<BEBOP.IP.ADDRESS>
```

**Motion capture** (VRPN server from OptiTrack Motive):

```bash
ros2 launch vrpn_mocap client.launch.yaml server:=<server ip> port:=3883
```

Poses appear on `/vrpn_mocap/<tracker_name>/pose` (and `twist` / `accel` if the mocap system reports them). See `vrpn_mocap/README.md` for all parameters.

## Orientation

Rough data flow: `vrpn_mocap` (pose) → `rho_filter` (filtered odometry on `filtered_odom`) → `bebop_control` (reads odometry, a desired-velocity `Twist` and a mode `Int32`; publishes `cmd_vel`) → `ros2_bebop_driver` → drone. `bebop_teleop` supplies the operator's commands and `indoor_lab_viz` draws the flight volume in RViz.

Parameters are declared without defaults in `bebop_control` and `rho_filter`, so each node needs a parameter file (see the `declare_parameter` calls in `src/`).

## Contributing and licensing

- Own packages (`bebop_control`, `bebop_teleop`, `indoor_lab_viz`, `rho_filter`) are licensed under [MPL 2.0](LICENSE); each source file carries the MPL header. The forks keep their upstream licenses.
- Each package's `package.xml` carries its description, maintainer and license.
- To update a submodule: `cd <submodule> && git pull origin main && cd .. && git add <submodule> && git commit`.
