# 4DRadarSLAM
## A 4D Imaging Radar SLAM System for Large-scale Environments based on Pose Graph Optimization

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04, 18.04 or 20.04.
ROS Kinetic, Melodic or Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation) and its additional ROS pacakge:

### 1.2 ***4DRadarSLAM*** requires the following libraries:

- OpenMP
- PCL
- g2o
- suitesparse

### 1.3 The following ROS packages are required:
```
    sudo apt-get install ros-XXX-geodesy ros-XXX-pcl-ros ros-XXX-nmea-msgs ros-XXX-libg2o
```
**NOTICE:** remember to replace "XXX" on above command as your ROS distributions, for example, if your use ROS-noetic, the command should be:

```
    sudo apt-get install ros-noetic-geodesy ros-noetic-pcl-ros ros-noetic-nmea-msgs ros-noetic-libg2o
```

## 4. Run our examples
Download [Our recorded rosbag](https://drive.google.com/drive/folders/1LpoX6_05Zic-mRLOD38EO0w2ABXI1rrW?usp=sharing) and then
```
roslaunch radar_graph_slam radar_graph_slam.launch
```
If everything is correct, you will get the result that matches our paper:)
The result of [indoor_aggressive.bag](https://drive.google.com/file/d/1UwEna7S6Unm0RuGcSZhUkEstNsbJaMjX/view?usp=sharing) (the Experiment-1 in our [paper](paper/r2live_ral_final.pdf)):
<div align="center">
    <img src="pics/indoor_aggressive_bag_res.png" width = 60% >
</div>

and [hku_main_building.bag](https://drive.google.com/file/d/1cwCuUYkUwL4ch_oloAoUMfL-G1MHTdMk/view?usp=sharing) (our Experiment-3).
<div align="center">
    <img src="pics/cover.png" width = 60% >
</div>

## 7. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the performance and reliability of our codes. For any technical issues, please contact me via email  <  >.

For commercial use, please contact Dr.  <  >
