// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>

#include <geometry_msgs/Transform.h>
#include <sensor_msgs/Imu.h>

#include <g2o/edge_se3_priorz.hpp>

namespace g2o {
class VertexSE3;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace radar_graph_slam {

/**
 * @brief KeyFrame (pose node)
 */
struct KeyFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PointT = pcl::PointXYZI;
  using Ptr = std::shared_ptr<KeyFrame>;

  KeyFrame(const size_t index, const ros::Time& stamp, const Eigen::Isometry3d& odom_scan2scan, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr& cloud);
  KeyFrame(const std::string& directory, g2o::HyperGraph* graph);
  virtual ~KeyFrame();

  void save(const std::string& directory);
  bool load(const std::string& directory, g2o::HyperGraph* graph);

  long id() const;
  Eigen::Isometry3d estimate() const;

public:
  size_t index;
  ros::Time stamp;                                // timestamp
  Eigen::Isometry3d odom_scan2scan;               // odometry (estimated by scan_matching_odometry)
  Eigen::Isometry3d odom_scan2map;
  double accum_distance;                          // accumulated distance from the first node (by scan_matching_odometry)
  pcl::PointCloud<PointT>::ConstPtr cloud;        // point cloud
  boost::optional<Eigen::Vector4d> floor_coeffs;  // detected floor's coefficients
  boost::optional<Eigen::Vector3d> utm_coord;     // UTM coord obtained by GPS
  boost::optional<Eigen::Vector1d> altitude;      // Altitude (Filtered) obtained by Barometer

  boost::optional<Eigen::Vector3d> acceleration;    //
  boost::optional<Eigen::Quaterniond> orientation;  //

  geometry_msgs::Transform trans_integrated; // relative transform obtained by imu preintegration

  boost::optional<sensor_msgs::Imu> imu; // the IMU message close to keyframe_

  g2o::VertexSE3* node;  // node instance
};

/**
 * @brief KeyFramesnapshot for map cloud generation
 */
struct KeyFrameSnapshot {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PointT = KeyFrame::PointT;
  using Ptr = std::shared_ptr<KeyFrameSnapshot>;

  KeyFrameSnapshot(const KeyFrame::Ptr& key);
  KeyFrameSnapshot(const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud);

  ~KeyFrameSnapshot();

public:
  Eigen::Isometry3d pose;                   // pose estimated by graph optimization
  pcl::PointCloud<PointT>::ConstPtr cloud;  // point cloud
};

}  // namespace radar_graph_slam

#endif  // KEYFRAME_HPP
