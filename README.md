# gopher_ros_clearcore

This package provides control of a robot chest component using ROS. It initializes `chest_control`, `chest_serial`, and `chest_logger` ROS nodes and sets up various ROS subscribers and service providers for controlling the chest. It communicates with a Teknic ClearCore board over a serial interface to control the chest motion.

## Dependencies

- ROS 1 (tested on Noetic)
- `rospy`
- `geometry_msgs`

## Installation Steps

1. Create a ROS workspace:
    
    ```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    ```

2. Clone the package into the `src` directory:

    ```
    git clone https://github.com/<your-github-account>/gopher_ros_clearcore.git
    ```

3. Install dependencies:

    ```
    sudo apt-get update
    rosdep update
    rosdep install --from-paths . --ignore-src -y
    ```

4. Build the package:

    ```
    cd ~/catkin_ws/
    catkin_make
    ```

## How to launch the nodes

Use the following command to launch the three nodes, `chest_control`, `chest_serial`, and `chest_logger`:

```bash
roslaunch gopher_ros_clearcore chest.launch
```

## Examples of publishing to ROS topics

To send velocity commands to the chest component, you can publish a `geometry_msgs/Twist` message to the `/z_chest_vel` topic:

```bash
rostopic pub /z_chest_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.5
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10
```

To send position commands to the chest component, you can publish a `gopher_ros_clearcore/Position` message to the `/z_chest_pos` topic:

```bash
rostopic pub /z_chest_pos gopher_ros_clearcore/Position "position: 10.0
velocity: 1.0" -1
```

## Examples of calling ROS services

To call a ROS service, you can use the `rosservice` command. For example, to stop the chest:

```bash
rosservice call /z_chest_stop "command: true"
```

To enable the chest drive motor:

```bash
rosservice call /z_chest_drive "command: true"
```

To enable the chest brake:

```bash
rosservice call /z_chest_brake "command: true"
```

To enable the chest debug mode:

```bash
rosservice call /z_chest_debug "command: true"
```

To enable the chest logger:

```bash
rosservice call /z_chest_logger "command: true"
```

To start the chest homing process:

```bash
rosservice call /z_chest_home "command: true"
```

To move the chest to an absolute position with a specified speed fraction:

```bash
rosservice call /z_chest_abspos "position: 100.0
velocity: 0.75"
```

To move the chest to a relative position with a specified speed fraction:

```bash
rosservice call /z_chest_relpos "position: -10.0
velocity: 0.5"
```

## Examples of subscribing to the chest feedback

This part provides instructions on how to get feedback from the chest of a robot using the chest logger ROS node. The code subscribes to the `logged_info` ROS topic, which provides the chest component's status information. The information is deserialized and published to various ROS topics.

To subscribe and echo the `chest_position` topic:

```
rostopic echo /chest_position
```

To subscribe and echo the `chest_velocity` topic:

```
rostopic echo /chest_velocity
```

To subscribe and echo the `brake_status` topic:

```
rostopic echo /brake_status
```

To subscribe and echo the `motor_status` topic:

```
rostopic echo /motor_status
```

To subscribe and echo the `limits_status` topic:

```
rostopic echo /limits_status
```