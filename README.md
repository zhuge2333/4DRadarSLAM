# 4DRadarSLAM
## A 4D Imaging Radar SLAM System for Large-scale Environments based on Pose Graph Optimization

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04, 18.04 or 20.04.
ROS Kinetic, Melodic or Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation) and its additional ROS pacakge:

***4DRadarSLAM*** requires the following libraries:

- OpenMP
- PCL
- g2o
- suitesparse

The following ROS packages are required:
```
    sudo apt-get install ros-XXX-geodesy ros-XXX-pcl-ros ros-XXX-nmea-msgs ros-XXX-libg2o
```
**NOTICE:** remember to replace "XXX" on above command as your ROS distributions, for example, if your use ROS-noetic, the command should be:

```
    sudo apt-get install ros-noetic-geodesy ros-noetic-pcl-ros ros-noetic-nmea-msgs ros-noetic-libg2o
```
