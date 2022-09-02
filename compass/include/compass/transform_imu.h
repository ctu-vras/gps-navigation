#pragma once

/**
 * \file
 * \brief A corrected version of IMU transformer library which changes orientation correctly.
 * \author Paul Bovbel (+ changes from Martin Pecka)
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 * 
 * This function in this file file has been copied from
 * https://github.com/ros-perception/imu_pipeline/blob/noetic-devel/imu_transformer/include/imu_transformer/tf2_sensor_msgs.h
 * and a change was made to resolve https://github.com/ros-perception/imu_pipeline/issues/8 .
 */

#include <Eigen/Core>

#include <geometry_msgs/TransformStamped.h>
#include <imu_transformer/tf2_sensor_msgs.h>
#include <sensor_msgs/Imu.h>

namespace compass
{

/**
* Transforms sensor_msgs::Imu data from one frame to another
*/
inline
void doTransform(const sensor_msgs::Imu &imu_in, sensor_msgs::Imu &imu_out, const geometry_msgs::TransformStamped& t_in)
{

  imu_out.header = t_in.header;

  // Discard translation, only use orientation for IMU transform
  Eigen::Quaternion<double> r(
    t_in.transform.rotation.w, t_in.transform.rotation.x, t_in.transform.rotation.y, t_in.transform.rotation.z);
  Eigen::Transform<double,3,Eigen::Affine> t(r);

  Eigen::Vector3d vel = t * Eigen::Vector3d(
    imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);

  imu_out.angular_velocity.x = vel.x();
  imu_out.angular_velocity.y = vel.y();
  imu_out.angular_velocity.z = vel.z();

  tf2::transformCovariance(imu_in.angular_velocity_covariance, imu_out.angular_velocity_covariance, r);

  Eigen::Vector3d accel = t * Eigen::Vector3d(
    imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);


  imu_out.linear_acceleration.x = accel.x();
  imu_out.linear_acceleration.y = accel.y();
  imu_out.linear_acceleration.z = accel.z();

  tf2::transformCovariance(imu_in.linear_acceleration_covariance, imu_out.linear_acceleration_covariance, r);

  // Orientation expresses attitude of the new frame_id in a fixed world frame. This is why the transform here applies
  // in the opposite direction.
  Eigen::Quaternion<double> orientation = Eigen::Quaternion<double>(
    imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z) * r.inverse();

  imu_out.orientation.w = orientation.w();
  imu_out.orientation.x = orientation.x();
  imu_out.orientation.y = orientation.y();
  imu_out.orientation.z = orientation.z();

  // Orientation is measured relative to the fixed world frame, so it doesn't change when applying a static
  // transform to the sensor frame.
  imu_out.orientation_covariance = imu_in.orientation_covariance;

}

}