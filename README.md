# Mobile Robotics Project 2
An autonomous vehicle with Lidar will be developed as a group project as part of the [FHGR BSc Mobile Robotics](https://fhgr.ch/mr).

## Goals of the Tank
### Modes of Operation
The tank will feature three different control modes:
* In manual mode, the user will utilize a laptop GUI to control the tank using arrow keys. While in this mode, the lidar will be active but won't map the room.
* The automatic exploration mode will scan the room and create a map of the surroundings. The live progress will be displayed in the laptop GUI.
* Once the mapping is complete, the third mode can be activated. In this mode, the camera will be enabled and will search for QR codes. Upon finding one, the gun will shoot at the QR code.

### GUI
The GUI will likely be developed using rqt. It will include a manual control interface for steering the tank, as well as several buttons to switch between different modes. To provide a visual representation of the scanned room, the current live map will be displayed within the GUI. Additionally, the GUI will show the tank's current location, speed, and direction of movement.

### Mapping
The mapping process will be conducted in 2D. However, it's important to note that the map will not be saved after the tank is powered off.

## Architecture
### Structure
The software will be roughly structured along these boundaries:
```mermaid
graph TD
    Controller -->|"move command<br/>message: <code>geometry_msgs/Twist</code><br/>topic: <code>/cmd_vel</code>"| MC

    MC[Movement Controller]
    MC -->|PWM signal| LM[[Left Motor]]
    MC -->|PWM signal| RM[[Right Motor]]

    Controller -->|"fire command<br/>service: <code>FireAt</code><br/>request:<code>geometry_msgs/Point</code>"| GC
    GC[Gun Controller] -->|PWM signal| GS
    GC -->|Trigger signal| GT
    GS[["Gun Servo (height adjust)"]]
    GT[[Gun Trigger]]

    LIDAR[[LIDAR]] -->|"point cloud<br/>message: <code>sensor_msgs/LaserScan</code><br/>topic: <code>/scan</code>"| Controller
    Camera[[Camera]] -->|"image<br/>message: <code>sensor_msgs/Image</code><br/>topic: <code>/image</code>"| IP
    IP[Image Processor] -->|"target information<br/>message: <code>geometry_msgs/PointStamped</code><br/>topic: <code>/target</code>"| Controller
```

Note that various helper ROS nodes are not (yet) shown here. Only the main components are shown.

When running in a simulator ([gazebo](https://gazebosim.org/)) the movement controller and gun controller will be replaced
by a simulator version thereof which will then interact with gazebo rather than the real hardware.

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
    [*] --> M: turn on
    M --> A: switch to automatic
    A --> M: switch to manual
    M --> S: switch to QR code search
    S --> M: switch to manual
    S --> G: target identified
    G --> M: shot fired

    M: Manual Control (remote)
    A: Automatic Exploration (Mapping)
    S: QR Code Search
    G: Gun Mode (shoot at target)
```
