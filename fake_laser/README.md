
fake_laser
=============

ROS node which publishes fake LaserScan messages:

<p align="center">
  <img src="https://lh3.googleusercontent.com/-fY1NKKUAy-w/Vukg1TsvjCI/AAAAAAAABv4/wAE9SbQMYXovx9ADvPuZh5sSo_anVuaLQCCo/s498-Ic42/fake_laser_diagram.png?raw=true" alt="Sublime's custom image"/>
</p>


<p align="center">
  <img src="https://lh3.googleusercontent.com/-OB69uFAsSxs/Vukg1WRYYjI/AAAAAAAABv0/7uFXT_5gfWIS7HVJYbDhekKE2-hXcCqHgCCo/s800-Ic42/fake_laser_rviz.png?raw=true" alt="Rviz"/>
</p>


## Installation

Clone the package into your catkin workspace:
```
git clone https://github.com/theja2289/fake_laser.git
```

## Usage
### fake_laser

For launching the fake_laser_publisher node, use:
```
roslaunch fake_laser fake_laser.launch
```

## ROS Features
### Publishers

#### scan
* sensor_msgs/LaserScan
* contains the sensor readings of the fake_laser
* update: 1s
* can be viewed in rviz

#### base_to_laser_broadcaster
* tf2_msgs/TFMessage
* update: 100 ms

## Notes
* Reference 1:(http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors)
* Reference 2:(http://answers.ros.org/question/198843/need-explanation-on-sensor_msgslaserscanmsg)
* assumptions based on Hokuyo UST-20LX  laser scanner
* base_link to base_laser transform was obtained from `Freight Robot` from `Fetch Robotics Inc.`
