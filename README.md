# Mobile Robotics Project 2
[![CI](https://github.com/rursprung/fhgr-mrproj2/actions/workflows/CI.yml/badge.svg)](https://github.com/rursprung/fhgr-mrproj2/actions/workflows/CI.yml)

An autonomous vehicle with Lidar will be developed as a group project by [Tim Barmettler](https://github.com/TimBarmettler4),
[Joel Flepp](https://github.com/joel5399), [Jan Gridling](https://github.com/Prince-Sigvald) and [Ralph Ursprung](https://github.com/rursprung)
as part of the [FHGR BSc Mobile Robotics](https://fhgr.ch/mr).

## Features

The robot can be operated using the keyboard and driven around. While driving, it creates a map of its environment.
In parallel, a camera is used to scan for a QR code and if this is found, the gun will shoot at the QR code.

### GUI

The GUI will likely be developed using rqt. It will include a manual control interface for steering the tank, as well as
several buttons to switch between different modes. To provide a visual representation of the scanned room, the current
live map will be displayed within the GUI. Additionally, the GUI will show the tank's current location, speed, and
direction of movement.

### Mapping

A 2D map is being created using [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) and can be
downloaded using [hector_geotiff](https://wiki.ros.org/hector_geotiff). If it is not downloaded, the data will be lost
once the software is stopped / the robot powered down.
Currently, the robot does not support loading a pre-made map and continue mapping in it.

## Architecture

### Structure

The software will be roughly structured along these boundaries:

```mermaid
graph TD
  Controller -->|"move command<br/>message: <code>geometry_msgs/Twist</code><br/>topic: <code>/cmd_vel</code>"| MC

  MC[ Movement Controller ]
  MC -->| PWM signal| LM[[ Left Motor ]]
  MC -->|PWM signal | RM[[ Right Motor ]]

  Controller -->|"fire command<br/>service: <code>FireAt</code><br/>request:<code>geometry_msgs/Point</code>"| GC
  GC[ Gun Controller ] -->| PWM signal| GS
  GC -->| Trigger signal | GT
  GS[["Gun Servo (height adjust)"]]
  GT[[ Gun Trigger ]]

  LIDAR[[LIDAR ]] -->|"point cloud<br/>message: <code>sensor_msgs/LaserScan</code><br/>topic: <code>/scan</code>"| Controller
  Camera[[ Camera ]] -->|"image<br/>message: <code>sensor_msgs/Image</code><br/>topic: <code>/image</code>"| IP
  IP[ Image Processor ] -->|"target information<br/>message: <code>geometry_msgs/PointStamped</code><br/>topic: <code>/target</code>"| Controller
```

Note that various helper ROS nodes are not (yet) shown here. Only the main components are shown.

When running in a simulator ([gazebo](https://gazebosim.org/)) the movement controller and gun controller will be
replaced by a simulator version thereof which will then interact with gazebo rather than the real hardware.

Convention used (& invented) for this diagram:

* Hardware is rendered like this:
  ```mermaid
  graph TD
      A[[Some Hardware]]
  ```
* Software (each box is a [ROS node](https://wiki.ros.org/Nodes)) is rendered like this:
  ```mermaid
  graph TD
      A[Some Software]
  ```

### State

```mermaid
stateDiagram-v2
    [*] --> D: turn on
    D --> G: target identified
    G --> D: shot fired

    D: Drive mode (Mapping, QR Code Search)
    G: Gun Mode (shoot at target)
```

## Working With This Repository

As this repository contains code for [ROS](https://ros.org/) (specifically, ROS noetic, the last 1.x release) and uses
the [catkin](https://catkin-tools.readthedocs.io/en/latest/) build tool, you'll need to work in a catkin workspace.

1. Source your ROS setup script (you should preferably add this to your `~/.profile`, esp. if you're working with an
   IDE!)
   ```bash
   source /opt/ros/noetic/setup.bash
   ```
2. Create a new folder for your new catkin workspace (note: you can have multiple catkin workspaces in parallel and also
   source all of their `setup.bash` files at the same time, that works fine):
   ```bash
   cd <wherever you want to have your workspace>
   ```
3. Clone this repository into the workspace and name the folder of the repository `src`:
   ```bash
   git clone --recurse-submodules git@github.com:rursprung/fhgr-mrproj2.git src
   ```
   Notes:
   * If you cloned it without specifying `src` at the end it'll create a folder named after the repository,
     i.e. `fhgr-mrproj2` - just use `mv fhgr-mrproj2 src` to fix this.
   * If you cloned it without specifying `--recurse-submodules` you can still manually init the submodules:
     ```bash
     git submodule init
     git submodule update
     ```
4. Install all required dependencies:
   ```bash
   rosdep install --from-paths . -i

   sudo pip install pynput # see https://github.com/ethz-asl/better_teleop for more details
   ```
5. Build the workspace & source its setup file:
   ```bash
   catkin build
   source devel/setup.bash
   ```
6. Optional: when working with an IDE you should add the sourcing of the `setup.bash` of your workspace also to
   your `~/.profile` (same as the one from ROS) so that it's set for all sessions and your IDE can access it!

### Contributing To The Repository
If you intend to contribute to the repository, please make sure that you're using a fork rather than working directly in the upstream repository!
Follow the guide above to set up your repository and then:
1. Create your fork on GitHub if you haven't done so already
2. Go to your local clone of the repository:
   ```bash
   cd <your workspace>/src
   ```
3. Rename the remote `origin` to `upstream`:
   ```bash
   git remote rename origin upstream
   ```
4. Add your own fork:
   ```bash
   git remote add origin <your fork URL>
   git fetch --all
   ```
5. Follow [the contribution guide](CONTRIBUTING.md) for further details.

### Running The Code
#### Simulation
After building the project you can run the simulated robot using:
```bash
roslaunch smt_launch_gazebo default.launch
```

You can use the arrow keys to navigate in the world.

#### On the Robot
To run the code on the robot you initially need to set up the udev rules for the LIDAR:
```bash
$(rospack find rplidar_ros)/scripts/create_udev_rules.sh
```
This only needs to be done once.

Afterward you can launch the on the robot:
```bash
roslaunch smt_launch_hardware default.launch
```
