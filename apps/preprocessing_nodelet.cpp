// SPDX-License-Identifier: BSD-2-Clause

#include <string>
#include <fstream>
#include <functional>

#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/filter.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>

#include "radar_ego_velocity_estimator.h"
#include "rio_utils/radar_point_cloud.h"
#include "utility_radar.h"

using namespace std;

namespace radar_graph_slam {

class PreprocessingNodelet : public nodelet::Nodelet, public ParamServer {
public: 
  // typedef pcl::PointXYZI PointT;
  typedef pcl::PointXYZI PointT;

  PreprocessingNodelet() {}
  virtual ~PreprocessingNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    initializeTransformation();
    initializeParams();

    points_sub = nh.subscribe(pointCloudTopic, 64, &PreprocessingNodelet::cloud_callback, this);
    imu_sub = nh.subscribe(imuTopic, 1, &PreprocessingNodelet::imu_callback, this);
    command_sub = nh.subscribe("/command", 10, &PreprocessingNodelet::command_callback, this);

    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 32);
    colored_pub = nh.advertise<sensor_msgs::PointCloud2>("/colored_points", 32);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 32);
    gt_pub = nh.advertise<nav_msgs::Odometry>("/aftmapped_to_init", 16);
  
    std::string topic_twist = private_nh.param<std::string>("topic_twist", "/eagle_data/twist");
    std::string topic_inlier_pc2 = private_nh.param<std::string>("topic_inlier_pc2", "/eagle_data/inlier_pc2");
    std::string topic_outlier_pc2 = private_nh.param<std::string>("topic_outlier_pc2", "/eagle_data/outlier_pc2");
    pub_twist = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(topic_twist, 5);
    pub_inlier_pc2 = nh.advertise<sensor_msgs::PointCloud2>(topic_inlier_pc2, 5);
    pub_outlier_pc2 = nh.advertise<sensor_msgs::PointCloud2>(topic_outlier_pc2, 5);
    pc2_raw_pub = nh.advertise<sensor_msgs::PointCloud2>("/eagle_data/pc2_raw",1);
    enable_dynamic_object_removal = private_nh.param<bool>("enable_dynamic_object_removal", false);
    power_threshold = private_nh.param<float>("power_threshold", 0);
  }

private:
  void initializeTransformation(){
    livox_to_RGB = (cv::Mat_<double>(4,4) << 
    -0.006878330000, -0.999969000000, 0.003857230000, 0.029164500000,  
    -7.737180000000E-05, -0.003856790000, -0.999993000000, 0.045695200000,
     0.999976000000, -0.006878580000, -5.084110000000E-05, -0.19018000000,
    0,  0,  0,  1);
    RGB_to_livox =livox_to_RGB.inv();
    Thermal_to_RGB = (cv::Mat_<double>(4,4) <<
    0.9999526089706319, 0.008963747151337641, -0.003798822163962599, 0.18106962419014,  
    -0.008945181135788245, 0.9999481006917174, 0.004876439015823288, -0.04546324090016857,
    0.00384233617405678, -0.004842226763999368, 0.999980894463835, 0.08046453079998771,
    0,0,0,1);
    Radar_to_Thermal = (cv::Mat_<double>(4,4) <<
    0.999665,    0.00925436,  -0.0241851,  -0.0248342,
    -0.00826999, 0.999146,    0.0404891,   0.0958317,
    0.0245392,   -0.0402755,  0.998887,    0.0268037,
    0,  0,  0,  1);
    Change_Radarframe=(cv::Mat_<double>(4,4) <<
    0,-1,0,0,
    0,0,-1,0,
    1,0,0,0,
    0,0,0,1);
    Radar_to_livox=RGB_to_livox*Thermal_to_RGB*Radar_to_Thermal*Change_Radarframe;
    std::cout << "Radar_to_livox = "<< std::endl << " "  << Radar_to_livox << std::endl << std::endl;
  }
  void initializeParams() {
    std::string downsample_method = private_nh.param<std::string>("downsample_method", "VOXELGRID");
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);

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
    }

    std::string outlier_removal_method = private_nh.param<std::string>("outlier_removal_method", "STATISTICAL");
    if(outlier_removal_method == "STATISTICAL") {
      int mean_k = private_nh.param<int>("statistical_mean_k", 20);
      double stddev_mul_thresh = private_nh.param<double>("statistical_stddev", 1.0);
      std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;

      pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
      sor->setMeanK(mean_k);
      sor->setStddevMulThresh(stddev_mul_thresh);
      outlier_removal_filter = sor;
    } else if(outlier_removal_method == "RADIUS") {
      double radius = private_nh.param<double>("radius_radius", 0.8);
      int min_neighbors = private_nh.param<int>("radius_min_neighbors", 2);
      std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;

      pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
      rad->setRadiusSearch(radius);
      rad->setMinNeighborsInRadius(min_neighbors);
      outlier_removal_filter = rad;
    } 
    // else if (outlier_removal_method == "BILATERAL")
    // {
    //   double sigma_s = private_nh.param<double>("bilateral_sigma_s", 5.0);
    //   double sigma_r = private_nh.param<double>("bilateral_sigma_r", 0.03);
    //   std::cout << "outlier_removal: BILATERAL " << sigma_s << " - " << sigma_r << std::endl;

    //   pcl::FastBilateralFilter<PointT>::Ptr fbf(new pcl::FastBilateralFilter<PointT>());
    //   fbf->setSigmaS (sigma_s);
    //   fbf->setSigmaR (sigma_r);
    //   outlier_removal_filter = fbf;
    // }
     else {
      std::cout << "outlier_removal: NONE" << std::endl;
    }

    use_distance_filter = private_nh.param<bool>("use_distance_filter", true);
    distance_near_thresh = private_nh.param<double>("distance_near_thresh", 1.0);
    distance_far_thresh = private_nh.param<double>("distance_far_thresh", 100.0);
    z_low_thresh = private_nh.param<double>("z_low_thresh", -5.0);
    z_high_thresh = private_nh.param<double>("z_high_thresh", 20.0);


    std::string file_name = private_nh.param<std::string>("gt_file_location", "");
    publish_tf = private_nh.param<bool>("publish_tf", false);

    ifstream file_in(file_name);
    if (!file_in.is_open()) {
        cout << "Can not open this gt file" << endl;
    }
    else{
      std::vector<std::string> vectorLines;
      std::string line;
      while (getline(file_in, line)) {
          vectorLines.push_back(line);
      }
      
      for (size_t i = 1; i < vectorLines.size(); i++) {
          std::string line_ = vectorLines.at(i);
          double stamp,tx,ty,tz,qx,qy,qz,qw;
          stringstream data(line_);
          data >> stamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
          nav_msgs::Odometry odom_msg;
          odom_msg.header.frame_id = mapFrame;
          odom_msg.child_frame_id = baselinkFrame;
          odom_msg.header.stamp = ros::Time().fromSec(stamp);
          odom_msg.pose.pose.orientation.w = qw;
          odom_msg.pose.pose.orientation.x = qx;
          odom_msg.pose.pose.orientation.y = qy;
          odom_msg.pose.pose.orientation.z = qz;
          odom_msg.pose.pose.position.x = tx;
          odom_msg.pose.pose.position.y = ty;
          odom_msg.pose.pose.position.z = tz;
          std::lock_guard<std::mutex> lock(odom_queue_mutex);
          odom_msgs.push_back(odom_msg);
      }
    }
    file_in.close();
  }

  void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
    sensor_msgs::Imu imu_data;
    imu_data.header.stamp = imu_msg->header.stamp;
    imu_data.header.seq = imu_msg->header.seq;
    imu_data.header.frame_id = "imu_frame";
    Eigen::Quaterniond q_ahrs(imu_msg->orientation.w,
                              imu_msg->orientation.x,
                              imu_msg->orientation.y,
                              imu_msg->orientation.z);
    Eigen::Quaterniond q_r = 
        Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_rr = 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_out =  q_r * q_ahrs * q_rr;
    imu_data.orientation.w = q_out.w();
    imu_data.orientation.x = q_out.x();
    imu_data.orientation.y = q_out.y();
    imu_data.orientation.z = q_out.z();
    imu_data.angular_velocity.x = imu_msg->angular_velocity.x;
    imu_data.angular_velocity.y = -imu_msg->angular_velocity.y;
    imu_data.angular_velocity.z = -imu_msg->angular_velocity.z;
    imu_data.linear_acceleration.x = imu_msg->linear_acceleration.x;
    imu_data.linear_acceleration.y = -imu_msg->linear_acceleration.y;
    imu_data.linear_acceleration.z = -imu_msg->linear_acceleration.z;
    imu_pub.publish(imu_data);
    // imu_queue.push_back(imu_msg);
    double time_now = imu_msg->header.stamp.toSec();
    bool updated = false;
    if (odom_msgs.size() != 0) {
      while (odom_msgs.front().header.stamp.toSec() + 0.001 < time_now) {
        std::lock_guard<std::mutex> lock(odom_queue_mutex);
        odom_msgs.pop_front();
        updated = true;
        if (odom_msgs.size() == 0)
          break;
      }
    }
    if (updated == true && odom_msgs.size() > 0){
      if (publish_tf) {
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.child_frame_id = baselinkFrame;
        tf_msg.header.frame_id = mapFrame;
        tf_msg.header.stamp = odom_msgs.front().header.stamp;
        // tf_msg.header.stamp = ros::Time().now();
        tf_msg.transform.rotation = odom_msgs.front().pose.pose.orientation;
        tf_msg.transform.translation.x = odom_msgs.front().pose.pose.position.x;
        tf_msg.transform.translation.y = odom_msgs.front().pose.pose.position.y;
        tf_msg.transform.translation.z = odom_msgs.front().pose.pose.position.z;
        tf_broadcaster.sendTransform(tf_msg);
      }
      
      gt_pub.publish(odom_msgs.front());
    }
  }

  void cloud_callback(const sensor_msgs::PointCloud::ConstPtr&  eagle_msg) { // const pcl::PointCloud<PointT>& src_cloud_r
    
    RadarPointCloudType radarpoint_raw;
    PointT radarpoint_xyzi;
    pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw( new pcl::PointCloud<RadarPointCloudType> );
    pcl::PointCloud<PointT>::Ptr radarcloud_xyzi( new pcl::PointCloud<PointT> );

    radarcloud_xyzi->header.frame_id = baselinkFrame;
    radarcloud_xyzi->header.seq = eagle_msg->header.seq;
    radarcloud_xyzi->header.stamp = eagle_msg->header.stamp.toSec() * 1e6;
    for(int i = 0; i < eagle_msg->points.size(); i++)
    {
        // cout << i << ":    " <<eagle_msg->points[i].x<<endl;
        if(eagle_msg->channels[2].values[i] > power_threshold) //"Power"
        {
            if (eagle_msg->points[i].x == NAN || eagle_msg->points[i].y == NAN || eagle_msg->points[i].z == NAN) continue;
            if (eagle_msg->points[i].x == INFINITY || eagle_msg->points[i].y == INFINITY || eagle_msg->points[i].z == INFINITY) continue;
            cv::Mat ptMat, dstMat;
            ptMat = (cv::Mat_<double>(4, 1) << eagle_msg->points[i].x, eagle_msg->points[i].y, eagle_msg->points[i].z, 1);    
            // Perform matrix multiplication and save as Mat_ for easy element access
            dstMat= Radar_to_livox * ptMat;
            radarpoint_raw.x = dstMat.at<double>(0,0);
            radarpoint_raw.y = dstMat.at<double>(1,0);
            radarpoint_raw.z = dstMat.at<double>(2,0);
            radarpoint_raw.intensity = eagle_msg->channels[2].values[i];
            radarpoint_raw.doppler = eagle_msg->channels[0].values[i];
            radarpoint_xyzi.x = dstMat.at<double>(0,0);
            radarpoint_xyzi.y = dstMat.at<double>(1,0);
            radarpoint_xyzi.z = dstMat.at<double>(2,0);
            radarpoint_xyzi.intensity = eagle_msg->channels[2].values[i];

            radarcloud_raw->points.push_back(radarpoint_raw);
            radarcloud_xyzi->points.push_back(radarpoint_xyzi);
        }
    }

    //********** Publish PointCloud2 Format Raw Cloud **********
    sensor_msgs::PointCloud2 pc2_raw_msg;
    pcl::toROSMsg(*radarcloud_raw, pc2_raw_msg);
    pc2_raw_msg.header.stamp = eagle_msg->header.stamp;
    pc2_raw_msg.header.frame_id = baselinkFrame;
    pc2_raw_pub.publish(pc2_raw_msg);

    //********** Ego Velocity Estimation **********
    Eigen::Vector3d v_r, sigma_v_r;
    sensor_msgs::PointCloud2 inlier_radar_msg, outlier_radar_msg;
    clock_t start_ms = clock();
    if (estimator.estimate(pc2_raw_msg, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg))
    {
        clock_t end_ms = clock();
        double time_used = double(end_ms - start_ms) / CLOCKS_PER_SEC;
        egovel_time.push_back(time_used);
        
        geometry_msgs::TwistWithCovarianceStamped twist;
        twist.header.stamp         = pc2_raw_msg.header.stamp;
        twist.twist.twist.linear.x = v_r.x();
        twist.twist.twist.linear.y = v_r.y();
        twist.twist.twist.linear.z = v_r.z();

        twist.twist.covariance.at(0)  = std::pow(sigma_v_r.x(), 2);
        twist.twist.covariance.at(7)  = std::pow(sigma_v_r.y(), 2);
        twist.twist.covariance.at(14) = std::pow(sigma_v_r.z(), 2);

        pub_twist.publish(twist);
        pub_inlier_pc2.publish(inlier_radar_msg);
        pub_outlier_pc2.publish(outlier_radar_msg);

    }
    else{;}

    pcl::PointCloud<PointT>::Ptr radarcloud_inlier( new pcl::PointCloud<PointT> );
    pcl::fromROSMsg (inlier_radar_msg, *radarcloud_inlier);

    pcl::PointCloud<PointT>::ConstPtr src_cloud;
    if (enable_dynamic_object_removal)
      src_cloud = radarcloud_inlier;
    else
      src_cloud = radarcloud_xyzi;

    
    if(src_cloud->empty()) {
      return;
    }

    src_cloud = deskewing(src_cloud);

    // if baselinkFrame is defined, transform the input cloud to the frame
    if(!baselinkFrame.empty()) {
      if(!tf_listener.canTransform(baselinkFrame, src_cloud->header.frame_id, ros::Time(0))) {
        std::cerr << "failed to find transform between " << baselinkFrame << " and " << src_cloud->header.frame_id << std::endl;
      }

      tf::StampedTransform transform;
      tf_listener.waitForTransform(baselinkFrame, src_cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
      tf_listener.lookupTransform(baselinkFrame, src_cloud->header.frame_id, ros::Time(0), transform);

      pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
      pcl_ros::transformPointCloud(*src_cloud, *transformed, transform);
      transformed->header.frame_id = baselinkFrame;
      transformed->header.stamp = src_cloud->header.stamp;
      src_cloud = transformed;
    }

    pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter(src_cloud);
    // filtered = passthrough(filtered);
    filtered = downsample(filtered);
    filtered = outlier_removal(filtered);

    // Distance Histogram
    static size_t num_frame = 0;
    if (num_frame % 10 == 0) {
      Eigen::VectorXi num_at_dist = Eigen::VectorXi::Zero(100);
      for (int i = 0; i < filtered->size(); i++){
        int dist = floor(filtered->at(i).getVector3fMap().norm());
        if (dist < 100)
          num_at_dist(dist) += 1;
      }
      num_at_dist_vec.push_back(num_at_dist);
    }

    points_pub.publish(*filtered);
    
  }


  pcl::PointCloud<PointT>::ConstPtr passthrough(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    PointT pt;
    for(int i = 0; i < cloud->size(); i++){
      if (cloud->at(i).z < 10 && cloud->at(i).z > -2){
        pt.x = (*cloud)[i].x;
        pt.y = (*cloud)[i].y;
        pt.z = (*cloud)[i].z;
        pt.intensity = (*cloud)[i].intensity;
        filtered->points.push_back(pt);
      }
    }
    filtered->header = cloud->header;
    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      // Remove NaN/Inf points
      pcl::PointCloud<PointT>::Ptr cloudout(new pcl::PointCloud<PointT>());
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud, *cloudout, indices);
      
      return cloudout;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!outlier_removal_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr distance_filter(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());

    filtered->reserve(cloud->size());
    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const PointT& p) {
      double d = p.getVector3fMap().norm();
      double z = p.z;
      return d > distance_near_thresh && d < distance_far_thresh && z < z_high_thresh && z > z_low_thresh;
    });
    // for (size_t i=0; i<cloud->size(); i++){
    //   const PointT p = cloud->points.at(i);
    //   double d = p.getVector3fMap().norm();
    //   double z = p.z;
    //   if (d > distance_near_thresh && d < distance_far_thresh && z < z_high_thresh && z > z_low_thresh)
    //     filtered->points.push_back(p);
    // }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr deskewing(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    ros::Time stamp = pcl_conversions::fromPCL(cloud->header.stamp);
    if(imu_queue.empty()) {
      return cloud;
    }

    // the color encodes the point number in the point sequence
    if(colored_pub.getNumSubscribers()) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>());
      colored->header = cloud->header;
      colored->is_dense = cloud->is_dense;
      colored->width = cloud->width;
      colored->height = cloud->height;
      colored->resize(cloud->size());

      for(int i = 0; i < cloud->size(); i++) {
        double t = static_cast<double>(i) / cloud->size();
        colored->at(i).getVector4fMap() = cloud->at(i).getVector4fMap();
        colored->at(i).r = 255 * t;
        colored->at(i).g = 128;
        colored->at(i).b = 255 * (1 - t);
      }
      colored_pub.publish(*colored);
    }

    sensor_msgs::ImuConstPtr imu_msg = imu_queue.front();

    auto loc = imu_queue.begin();
    for(; loc != imu_queue.end(); loc++) {
      imu_msg = (*loc);
      if((*loc)->header.stamp > stamp) {
        break;
      }
    }

    imu_queue.erase(imu_queue.begin(), loc);

    Eigen::Vector3f ang_v(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    ang_v *= -1;

    pcl::PointCloud<PointT>::Ptr deskewed(new pcl::PointCloud<PointT>());
    deskewed->header = cloud->header;
    deskewed->is_dense = cloud->is_dense;
    deskewed->width = cloud->width;
    deskewed->height = cloud->height;
    deskewed->resize(cloud->size());

    double scan_period = private_nh.param<double>("scan_period", 0.1);
    for(int i = 0; i < cloud->size(); i++) {
      const auto& pt = cloud->at(i);

      // TODO: transform IMU data into the LIDAR frame
      double delta_t = scan_period * static_cast<double>(i) / cloud->size();
      Eigen::Quaternionf delta_q(1, delta_t / 2.0 * ang_v[0], delta_t / 2.0 * ang_v[1], delta_t / 2.0 * ang_v[2]);
      Eigen::Vector3f pt_ = delta_q.inverse() * pt.getVector3fMap();

      deskewed->at(i) = cloud->at(i);
      deskewed->at(i).getVector3fMap() = pt_;
    }

    return deskewed;
  }

  bool RadarRaw2PointCloudXYZ(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudxyz)
  {
      pcl::PointXYZ point_xyz;
      for(int i = 0; i < raw->size(); i++)
      {
          point_xyz.x = (*raw)[i].x;
          point_xyz.y = (*raw)[i].y;
          point_xyz.z = (*raw)[i].z;
          cloudxyz->points.push_back(point_xyz);
      }
      return true;
  }
  bool RadarRaw2PointCloudXYZI(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudxyzi)
  {
      pcl::PointXYZI radarpoint_xyzi;
      for(int i = 0; i < raw->size(); i++)
      {
          radarpoint_xyzi.x = (*raw)[i].x;
          radarpoint_xyzi.y = (*raw)[i].y;
          radarpoint_xyzi.z = (*raw)[i].z;
          radarpoint_xyzi.intensity = (*raw)[i].intensity;
          cloudxyzi->points.push_back(radarpoint_xyzi);
      }
      return true;
  }

  void command_callback(const std_msgs::String& str_msg) {
    if (str_msg.data == "time") {
      std::sort(egovel_time.begin(), egovel_time.end());
      double median = egovel_time.at(size_t(egovel_time.size() / 2));
      cout << "Ego velocity time cost (median): " << median << endl;
    }
    else if (str_msg.data == "point_distribution") {
      Eigen::VectorXi data(100);
      for (size_t i = 0; i < num_at_dist_vec.size(); i++){ // N
        Eigen::VectorXi& nad = num_at_dist_vec.at(i);
        for (int j = 0; j< 100; j++){
          data(j) += nad(j);
        }
      }
      data /= num_at_dist_vec.size();
      for (int i=0; i<data.size(); i++){
        cout << data(i) << ", ";
      }
      cout << endl;
    }
  }

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Subscriber imu_sub;
  std::vector<sensor_msgs::ImuConstPtr> imu_queue;
  ros::Subscriber points_sub;
  
  ros::Publisher points_pub;
  ros::Publisher colored_pub;
  ros::Publisher imu_pub;
  ros::Publisher gt_pub;

  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;


  bool use_distance_filter;
  double distance_near_thresh;
  double distance_far_thresh;
  double z_low_thresh;
  double z_high_thresh;

  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Filter<PointT>::Ptr outlier_removal_filter;

  cv::Mat Radar_to_livox; // Transform Radar point cloud to LiDAR Frame
  cv::Mat Thermal_to_RGB,Radar_to_Thermal,RGB_to_livox,livox_to_RGB,Change_Radarframe;
  rio::RadarEgoVelocityEstimator estimator;
  ros::Publisher pub_twist, pub_inlier_pc2, pub_outlier_pc2, pc2_raw_pub;

  float power_threshold;
  bool enable_dynamic_object_removal = false;

  std::mutex odom_queue_mutex;
  std::deque<nav_msgs::Odometry> odom_msgs;
  bool publish_tf;

  ros::Subscriber command_sub;
  std::vector<double> egovel_time;

  std::vector<Eigen::VectorXi> num_at_dist_vec;
};

}  // namespace radar_graph_slam

PLUGINLIB_EXPORT_CLASS(radar_graph_slam::PreprocessingNodelet, nodelet::Nodelet)
