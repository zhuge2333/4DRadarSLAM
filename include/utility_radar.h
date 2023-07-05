#pragma once
#ifndef _UTILITY_RADAR_ODOMETRY_H_
#define _UTILITY_RADAR_ODOMETRY_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include <opencv/cv.h>
// #include <opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

class ParamServer
{
public:

    ros::NodeHandle nh;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Sensor Configuration:
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer()
    {
        nh.param<std::string>("/robot_id", robot_id, "nuc_handcart");

        nh.param<std::string>("radar_slam/pointCloudTopic", pointCloudTopic, "/radar_enhanced_pcl");
        nh.param<std::string>("radar_slam/imuTopic", imuTopic, "/vectornav/imu");
        nh.param<std::string>("radar_slam/odomTopic", odomTopic, "/odom");
        nh.param<std::string>("radar_slam/gpsTopic", gpsTopic, "/ublox/fix");

        nh.param<std::string>("radar_slam/lidarFrame", lidarFrame, "livox");
        nh.param<std::string>("radar_slam/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("radar_slam/odometryFrame", odometryFrame, "odom_imu"); 
        nh.param<std::string>("radar_slam/mapFrame", mapFrame, "map");

        nh.param<bool>("radar_slam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("radar_slam/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("radar_slam/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<bool>("radar_slam/savePCD", savePCD, false);
        nh.param<std::string>("radar_slam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        nh.param<int>("radar_slam/downsampleRate", downsampleRate, 1);
        nh.param<float>("radar_slam/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("radar_slam/lidarMaxRange", lidarMaxRange, 1000.0);

        nh.param<float>("radar_slam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("radar_slam/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("radar_slam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("radar_slam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("radar_slam/imuGravity", imuGravity, 9.80511);
        nh.param<float>("radar_slam/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<vector<double>>("radar_slam/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("radar_slam/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("radar_slam/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        nh.param<float>("radar_slam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("radar_slam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("radar_slam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<int>("radar_slam/numberOfCores", numberOfCores, 2);
        nh.param<double>("radar_slam/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("radar_slam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("radar_slam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("radar_slam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("radar_slam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("radar_slam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<float>("radar_slam/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("radar_slam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("radar_slam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("radar_slam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("radar_slam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("radar_slam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("radar_slam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("radar_slam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("radar_slam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }

};





sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif