# Object Follower Bot

This project implements a robot that follows objects using a camera. The robot is controlled using ROS (Robot Operating System) and Gazebo simulator.

## Youtube

[![Video](https://img.youtube.com/vi/GSFU_vwBcQU/maxresdefault.jpg)](https://www.youtube.com/watch?v=GSFU_vwBcQU)


## Installation

### Prerequisites

- ROS (Kinetic, Melodic, or Noetic)
- Gazebo
- OpenCV
- Python 3

### Setup

#### 1. Clone this repository to your catkin workspace:

```
git clone https://github.com/yourusername/object_follower_bot.git
```

#### 2. Build the package:
```
cd object_follower_bot
catkin_make
```
## Usage

#### 1. Launch the Gazebo simulator with the provided world and robot model:
```
roslaunch object_follower_bot gazebo.launch
```

#### 2. Run the object follower node:
```
rosrun object_follower_bot follow_bot.py
```

## Description

The `follow_bot.py` script subscribes to the RGB camera topic to detect objects. It calculates the error between the center of the detected object and the center of the camera frame. Based on this error, it controls the robot's movement to follow the object.

## ROS Nodes

### follow_bot_node

- Subscribes to: `/camera/rgb/image_raw`
- Publishes to: `/cmd_vel`

## Launch Files

### gazebo.launch

Launches Gazebo simulator with a world containing the robot model.


## Contributing

Contributions are welcome! Please fork the repository and create a pull request.




