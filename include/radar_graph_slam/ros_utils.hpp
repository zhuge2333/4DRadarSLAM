// SPDX-License-Identifier: BSD-2-Clause

#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf_conversions/tf_eigen.h>

namespace radar_graph_slam {

extern const int imuQueLength = 200;
extern const float scanPeriod = 0.083333;

double restrict_rad(double rad){
  double out;
  if (rad < -M_PI/2)
    out = rad + M_PI;
  else if (rad > M_PI/2)
    out = rad - M_PI;
  else out = rad;
  return out;
}

Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R) {
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr;
}


/**
 * @brief convert Eigen::Matrix to geometry_msgs::TransformStamped
 * @param stamp            timestamp
 * @param pose             Eigen::Matrix to be converted
 * @param frame_id         tf frame_id
 * @param child_frame_id   tf child frame_id
 * @return converted TransformStamped
 */
static geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4d& pose, const std::string& frame_id, const std::string& child_frame_id) {
  Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));
  quat.normalize();
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = stamp;
  odom_trans.header.frame_id = frame_id;
  odom_trans.child_frame_id = child_frame_id;

  odom_trans.transform.translation.x = pose(0, 3);
  odom_trans.transform.translation.y = pose(1, 3);
  odom_trans.transform.translation.z = pose(2, 3);
  odom_trans.transform.rotation = odom_quat;

  return odom_trans;
}

static nav_msgs::Odometry matrix2odom(const ros::Time& stamp, const Eigen::Matrix4d& pose, const std::string& frame_id, const std::string& child_frame_id){
  Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));
  quat.normalize();
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();
  
  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id;
  odom.child_frame_id = child_frame_id;

  odom.pose.pose.position.x = pose(0, 3);
  odom.pose.pose.position.y = pose(1, 3);
  odom.pose.pose.position.z = pose(2, 3);
  odom.pose.pose.orientation = odom_quat;
  
  return odom;
}

static Eigen::Isometry3d pose2isometry(const geometry_msgs::Pose& pose) {
  Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
  mat.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  mat.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix();
  return mat;
}

static Eigen::Isometry3d transform2isometry(const geometry_msgs::Transform& transf) {
  Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
  mat.translation() = Eigen::Vector3d(transf.translation.x, transf.translation.y, transf.translation.z);
  mat.linear() = Eigen::Quaterniond(transf.rotation.w, transf.rotation.x, transf.rotation.y, transf.rotation.z).toRotationMatrix();
  return mat;
}

static Eigen::Isometry3d tf2isometry(const tf::StampedTransform& trans) {
  Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
  mat.translation() = Eigen::Vector3d(trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z());
  mat.linear() = Eigen::Quaterniond(trans.getRotation().w(), trans.getRotation().x(), trans.getRotation().y(), trans.getRotation().z()).toRotationMatrix();
  return mat;
}

static Eigen::Isometry3d quaternion2isometry(const Eigen::Quaterniond& orient) {
  Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
  mat.linear() = Eigen::Quaterniond(orient.w(), orient.x(), orient.y(), orient.z()).toRotationMatrix();
  return mat;
}

static geometry_msgs::Pose isometry2pose(const Eigen::Isometry3d& mat) {
  Eigen::Quaterniond quat(mat.linear());
  Eigen::Vector3d trans = mat.translation();

  geometry_msgs::Pose pose;
  pose.position.x = trans.x();
  pose.position.y = trans.y();
  pose.position.z = trans.z();
  pose.orientation.w = quat.w();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();

  return pose;
}

static nav_msgs::Odometry isometry2odom(const ros::Time& stamp, const Eigen::Isometry3d& mat, const std::string& frame_id, const std::string& child_frame_id) {
  Eigen::Quaterniond quat(mat.linear());
  Eigen::Vector3d trans = mat.translation();
  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id;
  odom.child_frame_id = child_frame_id;
  odom.pose.pose.position.x = trans.x();
  odom.pose.pose.position.y = trans.y();
  odom.pose.pose.position.z = trans.z();
  odom.pose.pose.orientation.w = quat.w();
  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();

  return odom;
}

static Eigen::Isometry3d odom2isometry(const nav_msgs::OdometryConstPtr& odom_msg) {
  const auto& orientation = odom_msg->pose.pose.orientation;
  const auto& position = odom_msg->pose.pose.position;

  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = quat.toRotationMatrix();
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return isometry;
}

}  // namespace radar_graph_slam

#endif  // ROS_UTILS_HPP
