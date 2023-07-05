// This file is part of RIO - Radar Inertial Odometry and Radar ego velocity estimation.
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <angles/angles.h>
#include <eigen3/Eigen/Dense>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>

namespace rio
{
typedef double Real;

typedef Eigen::Vector2d Vector2;
typedef Eigen::Vector3d Vector3;
typedef Eigen::Vector4d Vector4;
typedef Eigen::Matrix<double, 7, 1> Vector7;
typedef Eigen::Matrix<double, 11, 1> Vector11;
typedef Eigen::VectorXd Vector;
typedef Eigen::Matrix2d Matrix2;
typedef Eigen::Matrix3d Matrix3;
typedef Eigen::Matrix4d Matrix4;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::Quaterniond Quaternion;
typedef Eigen::Isometry3d Isometry;
typedef Eigen::AngleAxisd AngleAxis;

struct EulerAngles : public Vector3
{
  EulerAngles() : Vector3(0, 0, 0) {}

  EulerAngles(const Real roll, const Real pitch, const Real yaw) : Vector3(roll, pitch, yaw) {}

  EulerAngles(const Vector3& eul_n_b) : Vector3(eul_n_b) {}

  EulerAngles from_degrees(const Vector3& eul_rad)
  {
    x() = angles::from_degrees(eul_rad.x());
    y() = angles::from_degrees(eul_rad.y());
    z() = angles::from_degrees(eul_rad.z());
    return EulerAngles(x(), y(), z());
  }

  Vector3 to_degrees() { return Vector3(angles::to_degrees(x()), angles::to_degrees(y()), angles::to_degrees(z())); }

  Real& roll() { return Vector3::x(); }
  Real roll() const { return Vector3::x(); }

  Real& pitch() { return Vector3::y(); }
  Real pitch() const { return Vector3::y(); }

  Real& yaw() { return Vector3::z(); }
  Real yaw() const { return Vector3::z(); }
};

struct NavigationSolution
{
  NavigationSolution()
  {
    pose_n_b.linear().setIdentity();
    pose_n_b.translation().setZero();
    v_n_b.setZero();
  }
  NavigationSolution(const Isometry& pose_n_b, const Vector3& v_n_b) : pose_n_b{pose_n_b}, v_n_b{v_n_b} {}
  NavigationSolution(const Vector3& p_n_b, const Quaternion& q_n_b, const Vector3 v_n_b) : v_n_b{v_n_b}
  {
    setQuaternion(q_n_b);
    setPosition_n_b(p_n_b);
  }

  Vector3 getPosition_n_b() const { return pose_n_b.translation(); }

  void setPosition_n_b(const Vector3& p_n_b) { pose_n_b.translation() = p_n_b; }

  Quaternion getQuaternion_n_b() const { return Quaternion(pose_n_b.linear()); }

  void setQuaternion(const Quaternion& q_n_b) { pose_n_b.linear() = q_n_b.normalized().toRotationMatrix(); }

  EulerAngles getEuler_n_b() const
  {
    const Quaternion q = getQuaternion_n_b();
    tf2::Quaternion q_msg(q.x(), q.y(), q.z(), q.w());
    tf2::Matrix3x3 R(q_msg);
    EulerAngles eul;
    R.getEulerYPR(eul.yaw(), eul.pitch(), eul.roll());

    return eul;
  }

  void setEuler_n_b(const EulerAngles& eul_n_b)
  {
    tf2::Quaternion q_n_b;
    q_n_b.setRPY(eul_n_b.roll(), eul_n_b.pitch(), eul_n_b.yaw());
    pose_n_b.linear() = Matrix3(Quaternion(q_n_b.w(), q_n_b.x(), q_n_b.y(), q_n_b.z()));
  }

  Matrix3 getC_n_b() const { return pose_n_b.linear(); }

  Isometry getPoseRos() const
  {
    tf2::Quaternion q_n_b;
    q_n_b.setRPY(M_PI, 0, 0);
    return Matrix3(Quaternion(q_n_b.w(), q_n_b.x(), q_n_b.y(), q_n_b.z())) * pose_n_b;
  }

  Vector3 getVelocityRos() const { return Vector3(v_n_b.x(), -v_n_b.y(), -v_n_b.z()); }

  Isometry pose_n_b;  // position and attitude in navigation frame
  Vector3 v_n_b;      // [m/s]

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ImuData
{
  ImuData() : dt{0} {}
  ImuData(const double dt, const Vector3& a_b_ib, const Vector3& w_b_ib) : dt{dt}, a_b_ib{a_b_ib}, w_b_ib{w_b_ib} {}

  Real dt;         // [s]
  Vector3 a_b_ib;  // [m/s^2]
  Vector3 w_b_ib;  // [rad/s]

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ImuDataStamped
{
  ImuDataStamped() : dt{0} {}
  ImuDataStamped(const ros::Time& time_stamp,
                 const std::string frame_id,
                 const double dt,
                 const Vector3& a_b_ib,
                 const Vector3& w_b_ib) :
    time_stamp{time_stamp},
    frame_id{frame_id},
    dt{dt},
    a_b_ib{a_b_ib},
    w_b_ib{w_b_ib}
  {
  }

  ImuDataStamped(const sensor_msgs::ImuConstPtr& imu_msg, const Real dt) :
    time_stamp{imu_msg->header.stamp},
    frame_id{imu_msg->header.frame_id},
    dt{dt},
    a_b_ib{Vector3(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z)},
    w_b_ib{Vector3(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z)}
  {
  }

  sensor_msgs::Imu toImuMsg()
  {
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp          = time_stamp;
    imu_msg.angular_velocity.x    = w_b_ib.x();
    imu_msg.angular_velocity.y    = w_b_ib.y();
    imu_msg.angular_velocity.z    = w_b_ib.z();
    imu_msg.linear_acceleration.x = a_b_ib.x();
    imu_msg.linear_acceleration.y = a_b_ib.y();
    imu_msg.linear_acceleration.z = a_b_ib.z();
    return imu_msg;
  }

  ros::Time time_stamp;  // ros::Time
  std::string frame_id;  // frame id
  Real dt;               // [s]
  Vector3 a_b_ib;        // [m/s^2]
  Vector3 w_b_ib;        // [rad/s]

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace rio
