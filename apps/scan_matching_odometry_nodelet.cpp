// SPDX-License-Identifier: BSD-2-Clause

#include <ctime>
#include <chrono>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/octree/octree_search.h>

#include <Eigen/Dense>

#include <radar_graph_slam/ros_utils.hpp>
#include <radar_graph_slam/registrations.hpp>
#include <radar_graph_slam/ScanMatchingStatus.h>
#include <radar_graph_slam/keyframe.hpp>
#include <radar_graph_slam/keyframe_updater.hpp>
#include <radar_graph_slam/graph_slam.hpp>
#include <radar_graph_slam/information_matrix_calculator.hpp>

#include "utility_radar.h"

using namespace std;

namespace radar_graph_slam {

class ScanMatchingOdometryNodelet : public nodelet::Nodelet, public ParamServer {
public:
  typedef pcl::PointXYZI PointT;
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistWithCovarianceStamped, sensor_msgs::PointCloud2> ApproxSyncPolicy;
  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> ApproxSyncPolicy2;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ScanMatchingOdometryNodelet() {}
  virtual ~ScanMatchingOdometryNodelet() {}

  virtual void onInit() {
    NODELET_DEBUG("initializing scan_matching_odometry_nodelet...");
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params(); // this

    if(private_nh.param<bool>("enable_imu_frontend", false)) {
      msf_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/msf_core/pose", 1, boost::bind(&ScanMatchingOdometryNodelet::msf_pose_callback, this, _1, false));
      msf_pose_after_update_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/msf_core/pose_after_update", 1, boost::bind(&ScanMatchingOdometryNodelet::msf_pose_callback, this, _1, true));
    }
    //******** Subscribers **********
    ego_vel_sub.reset(new message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>(mt_nh, "/eagle_data/twist", 256));
    points_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(mt_nh, "/filtered_points", 32));
    sync.reset(new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(32), *ego_vel_sub, *points_sub));
    sync->registerCallback(boost::bind(&ScanMatchingOdometryNodelet::pointcloud_callback, this, _1, _2));
    imu_sub = nh.subscribe("/imu", 1024, &ScanMatchingOdometryNodelet::imu_callback, this);
    command_sub = nh.subscribe("/command", 10, &ScanMatchingOdometryNodelet::command_callback, this);

    //******** Publishers **********
    read_until_pub = nh.advertise<std_msgs::Header>("/scan_matching_odometry/read_until", 32);
    // Odometry of Radar scan-matching_
    odom_pub = nh.advertise<nav_msgs::Odometry>(odomTopic, 32);
    // Transformation of Radar scan-matching_
    trans_pub = nh.advertise<geometry_msgs::TransformStamped>("/scan_matching_odometry/transform", 32);
    status_pub = private_nh.advertise<ScanMatchingStatus>("/scan_matching_odometry/status", 8);
    aligned_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 32);
    submap_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_graph_slam/submap", 2);
  }

private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    auto& pnh = private_nh;
    points_topic = pnh.param<std::string>("points_topic", "/radar_enhanced_pcl");
    use_ego_vel = pnh.param<bool>("use_ego_vel", false);

    // The minimum tranlational distance and rotation angle between keyframes_.
    // If this value is zero, frames are always compared with the previous frame
    keyframe_delta_trans = pnh.param<double>("keyframe_delta_trans", 0.25);
    keyframe_delta_angle = pnh.param<double>("keyframe_delta_angle", 0.15);
    keyframe_delta_time = pnh.param<double>("keyframe_delta_time", 1.0);

    // Registration validation by thresholding
    enable_transform_thresholding = pnh.param<bool>("enable_transform_thresholding", false);
    enable_imu_thresholding = pnh.param<bool>("enable_imu_thresholding", false);
    max_acceptable_trans = pnh.param<double>("max_acceptable_trans", 1.0);
    max_acceptable_angle = pnh.param<double>("max_acceptable_angle", 1.0);
    max_diff_trans = pnh.param<double>("max_diff_trans", 1.0);
    max_diff_angle = pnh.param<double>("max_diff_angle", 1.0);
    max_egovel_cum = pnh.param<double>("max_egovel_cum", 1.0);

    map_cloud_resolution = pnh.param<double>("map_cloud_resolution", 0.05);
    keyframe_updater.reset(new KeyframeUpdater(pnh));

    enable_scan_to_map = pnh.param<bool>("enable_scan_to_map", false);
    max_submap_frames = pnh.param<int>("max_submap_frames", 5);

    enable_imu_fusion = private_nh.param<bool>("enable_imu_fusion", false);
    imu_debug_out = private_nh.param<bool>("imu_debug_out", false);
    cout << "enable_imu_fusion = " << enable_imu_fusion << endl;
    imu_fusion_ratio = private_nh.param<double>("imu_fusion_ratio", 0.1);

    // graph_slam.reset(new GraphSLAM(pnh.param<std::string>("g2o_solver_type", "lm_var")));

    // select a downsample method (VOXELGRID, APPROX_VOXELGRID, NONE)
    std::string downsample_method = pnh.param<std::string>("downsample_method", "VOXELGRID");
    double downsample_resolution = pnh.param<double>("downsample_resolution", 0.1);
    if(downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      auto voxelgrid = new pcl::VoxelGrid<PointT>();
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter.reset(voxelgrid);
    } else if(downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } else {
      if(downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" << std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
      pcl::PassThrough<PointT>::Ptr passthrough(new pcl::PassThrough<PointT>());
      downsample_filter = passthrough;
    }
    registration_s2s = select_registration_method(pnh);
    registration_s2m = select_registration_method(pnh);
  }

  void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
    
    Eigen::Quaterniond imu_quat_from(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
    Eigen::Quaterniond imu_quat_deskew = imu_quat_from * extQRPY;
    imu_quat_deskew.normalize();

    double roll, pitch, yaw;
    // tf::quaternionMsgToTF(imu_odom_msg->orientation, orientation);
    tf::Quaternion orientation = tf::Quaternion(imu_quat_deskew.x(),imu_quat_deskew.y(),imu_quat_deskew.z(),imu_quat_deskew.w());
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    imuPointerLast = (imuPointerLast + 1) % imuQueLength;
    imuTime[imuPointerLast] = imu_msg->header.stamp.toSec();
    imuRoll[imuPointerLast] = roll;
    imuPitch[imuPointerLast] = pitch;
    // cout << "get imu rp: " << roll << " " << pitch << endl;

    sensor_msgs::ImuPtr imu(new sensor_msgs::Imu);
    imu->header = imu_msg->header;
    imu->angular_velocity = imu_msg->angular_velocity; imu->linear_acceleration = imu_msg->linear_acceleration;
    imu->angular_velocity_covariance = imu_msg->angular_velocity_covariance;
    imu->linear_acceleration_covariance, imu_msg->linear_acceleration_covariance;
    imu->orientation_covariance = imu_msg->orientation_covariance;
    imu->orientation.w=imu_quat_deskew.w(); imu->orientation.x = imu_quat_deskew.x(); imu->orientation.y = imu_quat_deskew.y(); imu->orientation.z = imu_quat_deskew.z();
    {
      std::lock_guard<std::mutex> lock(imu_queue_mutex);
      imu_queue.push_back(imu);
    }

    static int cnt = 0;
    if(cnt == 0) {
      geometry_msgs::Quaternion imuQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, 0);
      global_orient_matrix = Eigen::Quaterniond(imuQuat.w, imuQuat.x, imuQuat.y, imuQuat.z).toRotationMatrix();
      ROS_INFO_STREAM("Initial IMU euler angles (RPY): "
            << RAD2DEG(roll) << ", " << RAD2DEG(pitch) << ", " << RAD2DEG(yaw));
      cnt = 1;
    }
    
  }

  bool flush_imu_queue() {
    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    if(keyframes.empty() || imu_queue.empty()) {
      return false;
    }
    bool updated = false;
    auto imu_cursor = imu_queue.begin();

    for(size_t i=0; i < keyframes.size(); i++) {
      auto keyframe = keyframes.at(i);
      if(keyframe->stamp < (*imu_cursor)->header.stamp) {
        continue;
      }
      if(keyframe->stamp > imu_queue.back()->header.stamp) {
        break;
      }
      // find the imu data which is closest to the keyframe_
      auto closest_imu = imu_cursor;
      for(auto imu = imu_cursor; imu != imu_queue.end(); imu++) {
        auto dt = ((*closest_imu)->header.stamp - keyframe->stamp).toSec();
        auto dt2 = ((*imu)->header.stamp - keyframe->stamp).toSec();
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }
        closest_imu = imu;
      }
      // if the time residual between the imu and keyframe_ is too large, skip it
      imu_cursor = closest_imu;
      if(0.2 < std::abs(((*closest_imu)->header.stamp - keyframe->stamp).toSec())) {
        continue;
      }
      sensor_msgs::Imu imu_;
      imu_.header = (*closest_imu)->header; imu_.orientation = (*closest_imu)->orientation;
      imu_.angular_velocity = (*closest_imu)->angular_velocity; imu_.linear_acceleration = (*closest_imu)->linear_acceleration;
      imu_.angular_velocity_covariance = (*closest_imu)->angular_velocity_covariance;
      imu_.linear_acceleration_covariance = (*closest_imu)->linear_acceleration_covariance;
      imu_.orientation_covariance = (*closest_imu)->orientation_covariance;
      keyframe->imu = imu_;
      updated = true;
    }
    auto remove_loc = std::upper_bound(imu_queue.begin(), imu_queue.end(), keyframes.back()->stamp, [=](const ros::Time& stamp, const sensor_msgs::ImuConstPtr& imupoint) { return stamp < imupoint->header.stamp; });
    imu_queue.erase(imu_queue.begin(), remove_loc);
    return updated;
  }

  std::pair<bool, sensor_msgs::Imu> get_closest_imu(ros::Time frame_stamp) {
    sensor_msgs::Imu imu_;
    std::pair<bool, sensor_msgs::Imu> false_result {false, imu_};
    if(keyframes.empty() || imu_queue.empty())
      return false_result;
    bool updated = false;
    auto imu_cursor = imu_queue.begin();
    
    // find the imu data which is closest to the keyframe_
    auto closest_imu = imu_cursor;
    for(auto imu = imu_cursor; imu != imu_queue.end(); imu++) {
      auto dt = ((*closest_imu)->header.stamp - frame_stamp).toSec();
      auto dt2 = ((*imu)->header.stamp - frame_stamp).toSec();
      if(std::abs(dt) < std::abs(dt2)) {
        break;
      }
      closest_imu = imu;
    }
    // if the time residual between the imu and keyframe_ is too large, skip it
    imu_cursor = closest_imu;
    if(0.2 < std::abs(((*closest_imu)->header.stamp - frame_stamp).toSec()))
      return false_result;

    imu_.header = (*closest_imu)->header; imu_.orientation = (*closest_imu)->orientation;
    imu_.angular_velocity = (*closest_imu)->angular_velocity; imu_.linear_acceleration = (*closest_imu)->linear_acceleration;
    imu_.angular_velocity_covariance = (*closest_imu)->angular_velocity_covariance; 
    imu_.linear_acceleration_covariance = (*closest_imu)->linear_acceleration_covariance;
    imu_.orientation_covariance = (*closest_imu)->orientation_covariance;

    updated = true;
    // cout << (*closest_imu)->orientation <<endl;
    std::pair<bool, sensor_msgs::Imu> result {updated, imu_};
    return result;
  }


  void transformUpdate(Eigen::Matrix4d& odom_to_update) // IMU
  {
		if (imuPointerLast >= 0) 
    {
      // cout << "    ";
      float imuRollLast = 0, imuPitchLast = 0;
      while (imuPointerFront != imuPointerLast) {
        if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
          break;
        }
        imuPointerFront = (imuPointerFront + 1) % imuQueLength;
      }
      cout << "    ";
      if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
        imuRollLast = imuRoll[imuPointerFront];
        imuPitchLast = imuPitch[imuPointerFront];
        cout << "    ";
      }
      else {
        cout << "    ";
        int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
        float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack])
                          / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod) 
                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

        imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
        imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
      }
      
      Eigen::Matrix3d matr = odom_to_update.block<3, 3>(0, 0);
      // Eigen::Vector3d xyz = odom_to_update.block<3, 1>(0, 3);
      Eigen::Vector3d ypr_odom = R2ypr(matr.block<3,3>(0,0));
      geometry_msgs::Quaternion imuQuat = tf::createQuaternionMsgFromRollPitchYaw(imuRollLast, imuPitchLast, ypr_odom(0));
      Eigen::Matrix3d imu_rot = Eigen::Matrix3d(Eigen::Quaterniond(imuQuat.w, imuQuat.x, imuQuat.y, imuQuat.z));
      Eigen::Vector3d ypr_imu = R2ypr(imu_rot);
      // IMU orientation transformed from world coordinate to map coordinate
      Eigen::Matrix3d imu_rot_transed = global_orient_matrix.inverse() * imu_rot;
      Eigen::Vector3d ypr_imu_trans = R2ypr(imu_rot_transed);
      double& yaw_ = ypr_odom(0);
      double pitch_fused = (1 - imu_fusion_ratio) * ypr_odom(1) + imu_fusion_ratio * ypr_imu_trans(1);
      double roll_fused = (1 - imu_fusion_ratio) * ypr_odom(2) + imu_fusion_ratio * ypr_imu_trans(2);
      geometry_msgs::Quaternion rosQuat = tf::createQuaternionMsgFromRollPitchYaw(roll_fused, pitch_fused, yaw_);
      Eigen::Quaterniond quat_updated = Eigen::Quaterniond(rosQuat.w, rosQuat.x, rosQuat.y, rosQuat.z);
      odom_to_update.block<3, 3>(0, 0) = quat_updated.toRotationMatrix();

      if (imu_debug_out)
        cout << "IMU rp: " << RAD2DEG(ypr_imu(2)) << " " << RAD2DEG(ypr_imu(1))
            << ". IMU transed rp: " << RAD2DEG(ypr_imu_trans(2)) << " " << RAD2DEG(ypr_imu_trans(1))
            // << ". Odom rp: " << RAD2DEG(ypr_odom(2)) << " " << RAD2DEG(ypr_odom(1))
            // << ". Updated rp: " << RAD2DEG(roll_fused) << " " << RAD2DEG(pitch_fused)
            // << ". Roll Pitch increment: " << RAD2DEG(roll_fused - ypr_odom(2)) << " " << RAD2DEG(pitch_fused - ypr_odom(1)) 
            << endl;
		}
  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  void pointcloud_callback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& twistMsg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if(!ros::ok()) {
      return;
    }
    timeLaserOdometry = cloud_msg->header.stamp.toSec();
    double this_cloud_time = cloud_msg->header.stamp.toSec();
    static double last_cloud_time = this_cloud_time;

    double dt = this_cloud_time - last_cloud_time;
    double egovel_cum_x = twistMsg->twist.twist.linear.x * dt;
    double egovel_cum_y = twistMsg->twist.twist.linear.y * dt;
    double egovel_cum_z = twistMsg->twist.twist.linear.z * dt;
    // If too large, set 0
    if (pow(egovel_cum_x,2)+pow(egovel_cum_y,2)+pow(egovel_cum_z,2) > pow(max_egovel_cum, 2));
    else egovel_cum.block<3, 1>(0, 3) = Eigen::Vector3d(egovel_cum_x, egovel_cum_y, egovel_cum_z);
    
    last_cloud_time = this_cloud_time;

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Matching
    Eigen::Matrix4d pose = matching(cloud_msg->header.stamp, cloud);
    geometry_msgs::TwistWithCovariance twist = twistMsg->twist;
    // publish map to odom frame
    publish_odometry(cloud_msg->header.stamp, mapFrame, odometryFrame, pose, twist);

    // In offline estimation, point clouds will be supplied until the published time
    std_msgs::HeaderPtr read_until(new std_msgs::Header());
    read_until->frame_id = points_topic;
    read_until->stamp = cloud_msg->header.stamp + ros::Duration(1, 0);
    read_until_pub.publish(read_until);

    read_until->frame_id = "/filtered_points";
    read_until_pub.publish(read_until);
  }


  void msf_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg, bool after_update) {
    if(after_update) {
      msf_pose_after_update = pose_msg;
    } else {
      msf_pose = pose_msg;
    }
  }

  /**
   * @brief downsample a point cloud
   * @param cloud  input cloud
   * @return downsampled point cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);

    return filtered;
  }

  /**
   * @brief estimate the relative pose between an input cloud and a keyframe_ cloud
   * @param stamp  the timestamp of the input cloud
   * @param cloud  the input cloud
   * @return the relative pose between the input cloud and the keyframe_ cloud
   */
  Eigen::Matrix4d matching(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    if(!keyframe_cloud_s2s) {
      prev_time = ros::Time();
      prev_trans_s2s.setIdentity();
      keyframe_pose_s2s.setIdentity();
      keyframe_stamp = stamp;
      keyframe_cloud_s2s = cloud;//downsample(cloud);
      registration_s2s->setInputTarget(keyframe_cloud_s2s); // Scan-to-scan
      if (enable_scan_to_map){
        prev_trans_s2m.setIdentity();
        keyframe_pose_s2m.setIdentity();
        keyframe_cloud_s2m = cloud;
        registration_s2m->setInputTarget(keyframe_cloud_s2m);
      }
      return Eigen::Matrix4d::Identity();
    }
    // auto filtered = downsample(cloud);
    auto filtered = cloud;
    // Set Source Cloud
    registration_s2s->setInputSource(filtered);
    if (enable_scan_to_map)
      registration_s2m->setInputSource(filtered);

    std::string msf_source;
    Eigen::Isometry3d msf_delta = Eigen::Isometry3d::Identity();
    
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    Eigen::Matrix4d odom_s2s_now;
    Eigen::Matrix4d odom_s2m_now;

    // **********  Matching  **********
    Eigen::Matrix4d guess;
    if (use_ego_vel)
      guess = prev_trans_s2s * egovel_cum * msf_delta.matrix();
    else
      guess = prev_trans_s2s * msf_delta.matrix();

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    registration_s2s->align(*aligned, guess.cast<float>());
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
    s2s_matching_time.push_back(time_used);

    publish_scan_matching_status(stamp, cloud->header.frame_id, aligned, msf_source, msf_delta);

    // If not converged, use last transformation
    if(!registration_s2s->hasConverged()) {
      NODELET_INFO_STREAM("scan matching_ has not converged!!");
      NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
      if (enable_scan_to_map) return keyframe_pose_s2m * prev_trans_s2m;
      else return keyframe_pose_s2s * prev_trans_s2s;
    }
    Eigen::Matrix4d trans_s2s = registration_s2s->getFinalTransformation().cast<double>();
    odom_s2s_now = keyframe_pose_s2s * trans_s2s;

    Eigen::Matrix4d trans_s2m;
    if (enable_scan_to_map){
      registration_s2m->align(*aligned, guess.cast<float>());
      if(!registration_s2m->hasConverged()) {
        NODELET_INFO_STREAM("scan matching_ has not converged!!");
        NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
        return keyframe_pose_s2m * prev_trans_s2m;
      }
      trans_s2m = registration_s2m->getFinalTransformation().cast<double>();
      odom_s2m_now = keyframe_pose_s2m * trans_s2m;
    }

    // Add abnormal judgment, that is, if the difference between the two frames matching point cloud 
    // transition matrix is too large, it will be discarded
    bool thresholded = false;
    if(enable_transform_thresholding) {
      Eigen::Matrix4d radar_delta;
      if(enable_scan_to_map) radar_delta = prev_trans_s2m.inverse() * trans_s2m;
      else radar_delta = prev_trans_s2s.inverse() * trans_s2s;
      double dx_rd = radar_delta.block<3, 1>(0, 3).norm();
      // double da_rd = std::acos(Eigen::Quaterniond(radar_delta.block<3, 3>(0, 0)).w())*180/M_PI;
      Eigen::AngleAxisd rotation_vector;
      rotation_vector.fromRotationMatrix(radar_delta.block<3, 3>(0, 0));
      double da_rd = rotation_vector.angle();
      Eigen::Matrix3d rot_rd = radar_delta.block<3, 3>(0, 0).cast<double>();
      bool too_large_trans = dx_rd > max_acceptable_trans || da_rd > max_acceptable_angle;
      double da, dx, delta_rot_imu = 0;
      Eigen::Matrix3d matrix_rot; Eigen::Vector3d delta_trans_egovel;

      if (enable_imu_thresholding) {
        // Use IMU orientation to determine whether the matching result is good or not
        sensor_msgs::Imu frame_imu;
        Eigen::Matrix3d rot_imu = Eigen::Matrix3d::Identity();
        auto result = get_closest_imu(stamp);
        if (result.first) {
          frame_imu = result.second;
          Eigen::Quaterniond imu_quat(frame_imu.orientation.w, frame_imu.orientation.x, frame_imu.orientation.y, frame_imu.orientation.z);
          Eigen::Quaterniond prev_imu_quat(last_frame_imu.orientation.w, last_frame_imu.orientation.x, last_frame_imu.orientation.y, last_frame_imu.orientation.z);
          rot_imu = (prev_imu_quat.inverse() * imu_quat).toRotationMatrix();
          Eigen::Vector3d eulerAngle_imu = rot_imu.eulerAngles(0,1,2); // roll pitch yaw
          Eigen::Vector3d eulerAngle_rd = last_radar_delta.block<3,3>(0,0).eulerAngles(0,1,2);
          Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(restrict_rad(eulerAngle_imu(0)),Eigen::Vector3d::UnitX()));
          Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(restrict_rad(eulerAngle_imu(1)),Eigen::Vector3d::UnitY()));
          Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(restrict_rad(eulerAngle_rd(2)),Eigen::Vector3d::UnitZ()));
          matrix_rot = yawAngle * pitchAngle * rollAngle;
          da = fabs(std::acos(Eigen::Quaterniond(rot_rd.inverse() * rot_imu).w()))*180/M_PI;
          delta_rot_imu = fabs(std::acos(Eigen::Quaterniond(rot_imu).w()))*180/M_PI;
          last_frame_imu = frame_imu;
        }
        delta_trans_egovel = egovel_cum.block<3,1>(0,3).cast<double>();
        Eigen::Vector3d delta_trans_radar = radar_delta.block<3,1>(0,3).cast<double>();
        dx = (delta_trans_egovel - delta_trans_radar).norm();

        if (dx > max_diff_trans || da > max_diff_angle || too_large_trans) {
          Eigen::Matrix4d mat_est(Eigen::Matrix4d::Identity());
          mat_est.block<3, 3>(0, 0) = matrix_rot;
          mat_est.block<3, 1>(0, 3) = delta_trans_egovel;
          if (too_large_trans) cout << "Too large transform! " << dx_rd << "[m] " << da_rd << "[deg] ";
          cout << "Difference of Odom and IMU/EgoVel too large " << dx << "[m] " << da << "[deg] (" << stamp << ")" << endl;
          prev_trans_s2s = prev_trans_s2s * mat_est;
          thresholded = true;
          if (enable_scan_to_map){
            prev_trans_s2m = prev_trans_s2m * mat_est;
            odom_s2m_now = keyframe_pose_s2m * prev_trans_s2m;
          }
          else odom_s2s_now = keyframe_pose_s2s * prev_trans_s2s;
        }
      }
      else {
        if (too_large_trans) {
          cout << "Too large transform!!  " << dx_rd << "[m] " << da_rd << "[degree]"<<
            " Ignore this frame (" << stamp << ")" << endl;
          prev_trans_s2s = trans_s2s;
          thresholded = true;
          if (enable_scan_to_map){
            prev_trans_s2m = trans_s2m;
            odom_s2m_now = keyframe_pose_s2m * prev_trans_s2m * radar_delta;
          }
          else odom_s2s_now = keyframe_pose_s2s * prev_trans_s2s * radar_delta;
        }
      }
      last_radar_delta = radar_delta;
      
      if(0){
        cout << "radar trans:" << dx_rd << " m rot:" << da_rd << " degree. EgoVel " << delta_trans_egovel.norm() << " m. "
        << "IMU rot " << delta_rot_imu << " degree." << endl;
        cout << "dx " << dx << " m. da " << da << " degree." << endl;
      }
    }
    prev_time = stamp;
    if (!thresholded) {
      prev_trans_s2s = trans_s2s;
      prev_trans_s2m = trans_s2m;
    }
    
    //********** Decided whether to accept the frame as a key frame or not **********
    if(keyframe_updater->decide(Eigen::Isometry3d(odom_s2s_now), stamp)) {
      // Loose Coupling the IMU roll & pitch
      if (enable_imu_fusion){
        if(enable_scan_to_map) transformUpdate(odom_s2m_now);
        else transformUpdate(odom_s2s_now);
      }

      keyframe_cloud_s2s = filtered;
      registration_s2s->setInputTarget(keyframe_cloud_s2s);
      keyframe_pose_s2s = odom_s2s_now;
      keyframe_stamp = stamp;
      prev_time = stamp;
      prev_trans_s2s.setIdentity();

      double accum_d = keyframe_updater->get_accum_distance();
      KeyFrame::Ptr keyframe(new KeyFrame(keyframe_index, stamp, Eigen::Isometry3d(odom_s2s_now.cast<double>()), accum_d, cloud));
      keyframe_index ++;
      keyframes.push_back(keyframe);

      // record keyframe's imu
      flush_imu_queue();

      if (enable_scan_to_map){
        pcl::PointCloud<PointT>::Ptr submap_cloud(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::ConstPtr submap_cloud_downsampled;
        for(size_t i=std::max(0, (int)keyframes.size()-max_submap_frames); i < keyframes.size()-1; i++){
          Eigen::Matrix4d rel_pose = keyframes.at(i)->odom_scan2scan.matrix().inverse() * keyframes.back()->odom_scan2scan.matrix();
          pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>());
          pcl::transformPointCloud(*keyframes.at(i)->cloud, *cloud_transformed, rel_pose);
          *submap_cloud += *cloud_transformed;
        }
        submap_cloud_downsampled = downsample(submap_cloud);
        keyframe_cloud_s2m = submap_cloud_downsampled;
        registration_s2m->setInputTarget(keyframe_cloud_s2m);
        
        keyframes.back()->odom_scan2map = Eigen::Isometry3d(odom_s2m_now);
        keyframe_pose_s2m = odom_s2m_now;
        prev_trans_s2m.setIdentity();
      }
    }
    
    if (aligned_points_pub.getNumSubscribers() > 0)
    {
      pcl::transformPointCloud (*cloud, *aligned, odom_s2s_now);
      aligned->header.frame_id = odometryFrame;
      aligned_points_pub.publish(*aligned);
    }

    if (enable_scan_to_map)
      return odom_s2m_now;
    else
      return odom_s2s_now;
  }


  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time& stamp, const std::string& father_frame_id, const std::string& child_frame_id, const Eigen::Matrix4d& pose_in, const geometry_msgs::TwistWithCovariance twist_in) {
    // publish transform stamped for IMU integration
    geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose_in, father_frame_id, child_frame_id); //"map" 
    trans_pub.publish(odom_trans);

    // broadcast the transform over TF
    map2odom_broadcaster.sendTransform(odom_trans);

    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = father_frame_id;   // frame: /odom
    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = pose_in(0, 3);
    odom.pose.pose.position.y = pose_in(1, 3);
    odom.pose.pose.position.z = pose_in(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;
    odom.twist = twist_in;

    odom_pub.publish(odom);
  }

  /**
   * @brief publish scan matching_ status
   */
  void publish_scan_matching_status(const ros::Time& stamp, const std::string& frame_id, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned, const std::string& msf_source, const Eigen::Isometry3d& msf_delta) {
    if(!status_pub.getNumSubscribers()) {
      return;
    }

    ScanMatchingStatus status;
    status.header.frame_id = frame_id;
    status.header.stamp = stamp;
    status.has_converged = registration_s2s->hasConverged();
    status.matching_error = registration_s2s->getFitnessScore();

    const double max_correspondence_dist = 0.5;

    int num_inliers = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for(int i=0; i<aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      registration_s2s->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      if(k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        num_inliers++;
      }
    }
    status.inlier_fraction = static_cast<float>(num_inliers) / aligned->size();

    status.relative_pose = isometry2pose(Eigen::Isometry3d(registration_s2s->getFinalTransformation().cast<double>()));

    if(!msf_source.empty()) {
      status.prediction_labels.resize(1);
      status.prediction_labels[0].data = msf_source;

      status.prediction_errors.resize(1);
      Eigen::Isometry3d error = Eigen::Isometry3d(registration_s2s->getFinalTransformation().cast<double>()).inverse() * msf_delta;
      status.prediction_errors[0] = isometry2pose(error.cast<double>());
    }

    status_pub.publish(status);
  }

  void command_callback(const std_msgs::String& str_msg) {
    if (str_msg.data == "time") {
      std::sort(s2s_matching_time.begin(), s2s_matching_time.end());
      double median = s2s_matching_time.at(size_t(s2s_matching_time.size() / 2));
      cout << "Scan Matching time cost (median): " << median << endl;
    }
  }

private:
  // ROS topics
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  // ros::Subscriber points_sub;
  ros::Subscriber msf_pose_sub;
  ros::Subscriber msf_pose_after_update_sub;
  ros::Subscriber imu_sub;

  std::mutex imu_queue_mutex;
  std::deque<sensor_msgs::ImuConstPtr> imu_queue;
  sensor_msgs::Imu last_frame_imu;

  bool enable_imu_fusion;
  bool imu_debug_out;
  Eigen::Matrix3d global_orient_matrix;  // The rotation matrix with initial IMU roll & pitch measurement (yaw = 0)
    double timeLaserOdometry = 0;
    int imuPointerFront;
    int imuPointerLast;
    double imuTime[imuQueLength];
    float imuRoll[imuQueLength];
    float imuPitch[imuQueLength];
    double imu_fusion_ratio;

  std::unique_ptr<message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>> ego_vel_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> points_sub;
  std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

  // Submap
  ros::Publisher submap_pub;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;
  std::vector<KeyFrame::Ptr> keyframes;
  size_t keyframe_index = 0;
  double map_cloud_resolution;
  int  max_submap_frames;
  bool enable_scan_to_map;

  // std::unique_ptr<GraphSLAM> graph_slam;
  // std::unique_ptr<InformationMatrixCalculator> inf_calclator;
  
  ros::Publisher odom_pub;
  ros::Publisher trans_pub;
  // ros::Publisher keyframe_trans_pub;
  ros::Publisher aligned_points_pub;
  ros::Publisher status_pub;
  ros::Publisher read_until_pub;
  tf::TransformListener tf_listener;
  tf::TransformBroadcaster map2odom_broadcaster; // map => odom_frame

  std::string points_topic;


  // keyframe_ parameters
  double keyframe_delta_trans;  // minimum distance between keyframes_
  double keyframe_delta_angle;  //
  double keyframe_delta_time;   //

  // registration validation by thresholding
  bool enable_transform_thresholding;  //
  bool enable_imu_thresholding;
  double max_acceptable_trans;  //
  double max_acceptable_angle;
  double max_diff_trans;
  double max_diff_angle;
  double max_egovel_cum;
  Eigen::Matrix4d last_radar_delta = Eigen::Matrix4d::Identity();

  // odometry calculation
  geometry_msgs::PoseWithCovarianceStampedConstPtr msf_pose;
  geometry_msgs::PoseWithCovarianceStampedConstPtr msf_pose_after_update;

  Eigen::Matrix4d egovel_cum = Eigen::Matrix4d::Identity();
  bool use_ego_vel;

  ros::Time prev_time;
  Eigen::Matrix4d prev_trans_s2s;                  // previous relative transform from keyframe_
  Eigen::Matrix4d keyframe_pose_s2s;               // keyframe_ pose
  Eigen::Matrix4d prev_trans_s2m;
  Eigen::Matrix4d keyframe_pose_s2m;               // keyframe_ pose
  ros::Time keyframe_stamp;                    // keyframe_ time
  pcl::PointCloud<PointT>::ConstPtr keyframe_cloud_s2s;  // keyframe_ point cloud
  pcl::PointCloud<PointT>::ConstPtr keyframe_cloud_s2m;  // keyframe_ point cloud

  // Registration
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration_s2s;    // Scan-to-Scan Registration
  pcl::Registration<PointT, PointT>::Ptr registration_s2m;    // Scan-to-Submap Registration

  // Time evaluation
  std::vector<double> s2s_matching_time;
  ros::Subscriber command_sub;
};

}  // namespace radar_graph_slam

PLUGINLIB_EXPORT_CLASS(radar_graph_slam::ScanMatchingOdometryNodelet, nodelet::Nodelet)
