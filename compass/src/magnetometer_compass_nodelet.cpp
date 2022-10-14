/**
 * \file
 * \brief Compute various azimuth values based on a magnetometer, IMU orientation and possibly also GPS coordinates.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <ctime>
#include <map>
#include <memory>
#include <string>

#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <angles/angles.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <imu_transformer/tf2_sensor_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <compass_msgs/Azimuth.h>

#include <compass/magnetometer_compass_nodelet.h>
#include <compass/transform_imu.h>

#define WMM2010 "wmm2010"
#define WMM2015 "wmm2015v2"
#define WMM2020 "wmm2020"

namespace compass
{

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::MagneticField> SyncPolicy;

struct AzimuthPublishersConfigForOrientation
{
  uint8_t orientation {};
  uint8_t reference {};

  ros::Publisher quatPub;
  ros::Publisher imuPub;
  ros::Publisher posePub;
  ros::Publisher radPub;
  ros::Publisher degPub;

  bool publishQuat {false};
  bool publishImu {false};
  bool publishPose {false};
  bool publishRad {false};
  bool publishDeg {false};
  
  bool publish {false};

  geometry_msgs::QuaternionStamped quatMsg;
  compass_msgs::Azimuth azimuthMsg;
  sensor_msgs::Imu imuMsg;
  geometry_msgs::PoseWithCovarianceStamped poseMsg;

  tf2::Matrix3x3 rotMatrix;
  double roll {}, pitch {}, yaw {};

  void init(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& frameId, const std::string& paramPrefix, 
    const std::string& topicPrefix, uint8_t reference, uint8_t orientation,
    const std::string& referenceStr, const std::string& orientationStr);

  void publishAzimuths(const tf2::Quaternion& azimuth, double variance, const ros::Time& stamp,
    const sensor_msgs::Imu& imuInBody);
};

struct AzimuthPublishersConfig
{
  uint8_t reference {};

  AzimuthPublishersConfigForOrientation ned;
  AzimuthPublishersConfigForOrientation enu;

  bool publish {false};

  const tf2::Quaternion nedToEnu {-M_SQRT2/2, -M_SQRT2/2, 0, 0};
  const tf2::Quaternion enuToNed {this->nedToEnu.inverse()};
  
  void init(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& frameId,
    const std::string& paramPrefix, const std::string& topicPrefix, uint8_t reference, const std::string& referenceStr);

  void publishAzimuths(const tf2::Quaternion& nedAzimuth, double variance, const ros::Time& stamp,
    const sensor_msgs::Imu& imuInBody);
};

struct MagnetometerCompassNodeletPrivate
{
  std::map<uint32_t, std::unique_ptr<GeographicLib::MagneticModel>> magneticModels;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Imu>> imuSub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::MagneticField>> magSub;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> syncSub;
  ros::Subscriber fixSub;
  ros::Subscriber magBiasSub;
  std::unique_ptr<tf2_ros::TransformListener> tfListener;
  
  ros::Publisher magUnbiasedPub;
  bool publishMagUnbiased {false};

  AzimuthPublishersConfig magPublishers;
  AzimuthPublishersConfig truePublishers;
  AzimuthPublishersConfig utmPublishers;
};

MagnetometerCompassNodelet::MagnetometerCompassNodelet() : data(new MagnetometerCompassNodeletPrivate)
{
}

MagnetometerCompassNodelet::~MagnetometerCompassNodelet()
{
}

void MagnetometerCompassNodelet::onInit()
{
  auto nh = this->getNodeHandle();
  auto pnh = this->getPrivateNodeHandle();

  this->frame = pnh.param("frame", std::string("base_link"));
  this->magAzimuthLowPass = pnh.param("low_pass_ratio", this->magAzimuthLowPass);
  if (pnh.hasParam("magnetic_declination"))
  {
    const auto declination = pnh.param("magnetic_declination", 0.0);
    this->lastMagneticDeclination.setRPY(0, 0, declination);
    this->magneticDeclinationForced = true;
  }
  else
  {
    this->magneticModelsPath = pnh.param(
      "magnetic_models_path", ros::package::getPath("compass") + "/data/magnetic");
    this->forcedMagneticModelName = pnh.param("magnetic_model", std::string());
  }
  
  this->variance = pnh.param("initial_variance", this->variance);

  if (pnh.hasParam("initial_mag_bias_x") || pnh.hasParam("initial_mag_bias_y") || pnh.hasParam("initial_mag_bias_z"))
  {
    sensor_msgs::MagneticField msg;
    msg.magnetic_field.x = pnh.param("initial_mag_bias_x", this->lastMagBias.x());
    msg.magnetic_field.y = pnh.param("initial_mag_bias_y", this->lastMagBias.y());
    msg.magnetic_field.z = pnh.param("initial_mag_bias_z", this->lastMagBias.z());

    const auto scalingMatrix = pnh.param("initial_mag_scaling_matrix", std::vector<double>({}));
    if (scalingMatrix.size() == 9)
      std::copy(scalingMatrix.begin(), scalingMatrix.end(), msg.magnetic_field_covariance.begin());
    else if (!scalingMatrix.empty())
      ROS_ERROR("Parameter initial_mag_scaling_matrix has to have either 0 or 9 values.");

    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > covMatrix(msg.magnetic_field_covariance.data());
    const auto hasScale = covMatrix.cwiseAbs().sum() > 1e-10;
    
    ROS_INFO("Initial magnetometer bias is %0.3f %0.3f %0.3f %s scaling factor",
      msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z, hasScale ? "with" : "without");
    
    this->magBiasCb(msg);
  }
  
  if (pnh.hasParam("initial_lat") && pnh.hasParam("initial_lon"))
  {
    sensor_msgs::NavSatFix msg;
    if (pnh.hasParam("initial_year") && pnh.param("initial_year", 0) > 1970)
    {
       struct tm time{};
       time.tm_year = pnh.param("initial_year", 0);
       msg.header.stamp.sec = static_cast<uint32_t>(mktime(&time));
    }
    else
    {
      // We can encounter invalid time when running a bag file and starting this node before the playback node
      if (ros::Time::isValid())
        msg.header.stamp = ros::Time::now();
      else
        msg.header.stamp.sec = ros::WallTime::now().sec;
    }
    
    msg.latitude = pnh.param("initial_lat", this->lastFix.latitude);
    msg.longitude = pnh.param("initial_lon", this->lastFix.longitude);
    msg.altitude = pnh.param("initial_alt", this->lastFix.altitude);

    auto computedValues = "declination and grid convergence";
    if (this->magneticDeclinationForced)
      computedValues = "grid convergence";
    ROS_INFO(
      "Initial GPS coords for computation of %s are %.6f, %.6f, altitude %.0f m, year %u",
      computedValues, msg.latitude, msg.longitude, msg.altitude, getYear(msg.header.stamp));
    
    this->fixCb(msg);
  }

  this->data->publishMagUnbiased = pnh.param("publish_mag_unbiased", this->data->publishMagUnbiased);
  
  // Set default publishers
  this->data->magPublishers.ned.publishDeg = true;
  
  bool publish = this->data->publishMagUnbiased;

  // Read the publish_* parameters and set up the selected publishers
  this->data->magPublishers.init(nh, pnh, this->frame, "publish", "compass",
    compass_msgs::Azimuth::REFERENCE_MAGNETIC, "mag");
  publish |= this->data->magPublishers.publish;
  this->data->truePublishers.init(nh, pnh, this->frame, "publish", "compass",
    compass_msgs::Azimuth::REFERENCE_GEOGRAPHIC, "true");
  publish |= this->data->truePublishers.publish;
  this->data->utmPublishers.init(nh, pnh, this->frame, "publish", "compass",
    compass_msgs::Azimuth::REFERENCE_UTM, "utm");
  publish |= this->data->utmPublishers.publish;

  if (!publish)
    ROS_WARN("No publishers have been requested. Please, set one of the publish_* parameters to true.");
  
  if (this->data->publishMagUnbiased)
    this->data->magUnbiasedPub = nh.advertise<sensor_msgs::MagneticField>("imu/mag_unbiased", 10);
  
  this->data->tfListener = std::make_unique<tf2_ros::TransformListener>(this->tf, nh);
  
  this->data->imuSub = std::make_unique<message_filters::Subscriber<sensor_msgs::Imu>>(nh, "imu/data", 100);
  this->data->magSub = std::make_unique<message_filters::Subscriber<sensor_msgs::MagneticField>>(nh, "imu/mag", 100);
  this->data->syncSub = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(200),
    *this->data->imuSub, *this->data->magSub);
  this->data->syncSub->registerCallback(&MagnetometerCompassNodelet::imuMagCb, this);
  
  this->data->fixSub = nh.subscribe("gps/fix", 10, &MagnetometerCompassNodelet::fixCb, this);
  this->data->magBiasSub = nh.subscribe("imu/mag_bias", 10, &MagnetometerCompassNodelet::magBiasCb, this);
}

void AzimuthPublishersConfigForOrientation::init(ros::NodeHandle& nh, ros::NodeHandle& pnh,
  const std::string& frameId, const std::string& paramPrefix, const std::string& topicPrefix,
  const uint8_t reference, const uint8_t orientation,
  const std::string& referenceStr, const std::string& orientationStr)
{
  this->reference = reference;
  this->orientation = orientation;

  this->quatMsg.header.frame_id = frameId;
  this->azimuthMsg.header.frame_id = frameId;
  this->imuMsg.header.frame_id = frameId;
  this->poseMsg.header.frame_id = frameId;

  this->azimuthMsg.orientation = this->orientation;
  this->azimuthMsg.reference = this->reference;
  
  auto prefix = paramPrefix + "_" + referenceStr + "_azimuth_" + orientationStr + "_";
  this->publishQuat = pnh.param(prefix + "quat", this->publishQuat);
  this->publishImu = pnh.param(prefix + "imu", this->publishImu);
  this->publishPose = pnh.param(prefix + "pose", this->publishPose);
  this->publishRad = pnh.param(prefix + "rad", this->publishRad);
  this->publishDeg = pnh.param(prefix + "deg", this->publishDeg);
  
  this->publish = this->publishQuat || this->publishImu || this->publishPose || this->publishDeg || this->publishRad;

  prefix = topicPrefix + "/" + referenceStr + "/" + orientationStr + "/";
  if (this->publishQuat) this->quatPub = nh.advertise<geometry_msgs::QuaternionStamped>(prefix + "quat", 10);
  if (this->publishImu) this->imuPub = nh.advertise<sensor_msgs::Imu>(prefix + "imu", 10);
  if (this->publishPose) this->posePub = nh.advertise<geometry_msgs::PoseStamped>(prefix + "pose", 10);
  if (this->publishRad) this->radPub = nh.advertise<compass_msgs::Azimuth>(prefix + "rad", 10);
  if (this->publishDeg) this->degPub = nh.advertise<compass_msgs::Azimuth>(prefix + "deg", 10);
}

void AzimuthPublishersConfig::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& frameId,
  const std::string& paramPrefix, const std::string& topicPrefix,
  const uint8_t reference, const std::string& referenceStr)
{
  this->reference = reference;
  this->ned.init(nh, pnh, frameId, paramPrefix, topicPrefix, reference,
    compass_msgs::Azimuth::ORIENTATION_NED, referenceStr, "ned");
  this->enu.init(nh, pnh, frameId, paramPrefix, topicPrefix, reference,
    compass_msgs::Azimuth::ORIENTATION_ENU, referenceStr, "enu");
  this->publish = this->ned.publish || this->enu.publish;
}

void MagnetometerCompassNodelet::imuMagCb(const sensor_msgs::Imu& imu, const sensor_msgs::MagneticField& mag)
{
  this->lastImu = imu;
  this->lastMag = mag;

  if (!this->hasMagBias)
  {
    ROS_ERROR_DELAYED_THROTTLE(10.0, "Magnetometer bias not available, compass cannot work. Waiting...");
    return;
  }

  try
  {
    // do not use tf2::doTransform(Imu), it's buggy!
    auto transform = this->tf.lookupTransform(this->frame, imu.header.frame_id, imu.header.stamp, ros::Duration(0.1));
    compass::doTransform(imu, this->lastImuInBody, transform);
  }
  catch (const tf2::TransformException& e)
  {
    NODELET_ERROR_THROTTLE(1.0, "Could not transform IMU data to frame %s because: %s", this->frame.c_str(), e.what());
    return;
  }

  tf2::Vector3 field;
  tf2::convert(mag.magnetic_field, field);
  field = this->lastMagScale * (field - this->lastMagBias);
  
  sensor_msgs::MagneticField magUnbiased = mag;
  tf2::convert(field, magUnbiased.magnetic_field);
  
  if (this->data->publishMagUnbiased)
    this->data->magUnbiasedPub.publish(magUnbiased);
  
  try
  {
    this->lastMagUnbiasedInBody = this->tf.transform(magUnbiased, this->frame, ros::Duration(0.1));
  }
  catch (const tf2::TransformException& e)
  {
    NODELET_ERROR_THROTTLE(1.0, "Could not transform magnetometer to frame %s because: %s",
      this->frame.c_str(), e.what());
    return;
  }

  // Compensate attitude in the magnetometer measurements

  tf2::Quaternion rot;
  tf2::convert(this->lastImuInBody.orientation, rot);
  tf2::Matrix3x3 rotMatrix(rot);
  double roll, pitch, yaw;
  rotMatrix.getRPY(roll, pitch, yaw);

#if 0
  rot.setRPY(roll, pitch, 0);
  tf2::Vector3 magNoAttitude;
  tf2::convert(this->lastMagUnbiasedInBody.magnetic_field, magNoAttitude);
  magNoAttitude = tf2::quatRotate(rot, magNoAttitude);
  
  const auto magNorth = magNoAttitude.x();
  const auto magEast = magNoAttitude.y();
#else
  // Copied from INSO, not sure where do the numbers come from
  const auto magNorth =
    this->lastMagUnbiasedInBody.magnetic_field.x * cos(pitch) +
    this->lastMagUnbiasedInBody.magnetic_field.y * sin(pitch) * sin(roll) +
    this->lastMagUnbiasedInBody.magnetic_field.z * sin(pitch) * cos(roll);

  const auto magEast =
    this->lastMagUnbiasedInBody.magnetic_field.y * cos(roll) -
    this->lastMagUnbiasedInBody.magnetic_field.z * sin(roll);
#endif

  // This formula gives north-referenced clockwise-increasing azimuth
  const auto magAzimuthNow = atan2(magEast, magNorth);
  tf2::Quaternion magAzimuthNowQuat;
  magAzimuthNowQuat.setRPY(0, 0, magAzimuthNow);
  
  if (this->magAzimuth.w() == 0.0)
    this->magAzimuth = magAzimuthNowQuat;
  else  // low-pass filter
    this->magAzimuth = this->magAzimuth.slerp(magAzimuthNowQuat, 1 - this->magAzimuthLowPass);
  this->updateVariance();
  
  this->data->magPublishers.publishAzimuths(this->magAzimuth, this->variance, mag.header.stamp, this->lastImuInBody);
  
  if ((this->data->truePublishers.publish || this->data->utmPublishers.publish) &&
    this->lastMagneticDeclination.w() != 0.0)
  {
    const auto trueAzimuth = this->lastMagneticDeclination * this->magAzimuth;
    this->data->truePublishers.publishAzimuths(trueAzimuth, this->variance, mag.header.stamp, this->lastImuInBody);
    
    if (this->data->utmPublishers.publish && this->lastFix.header.stamp != ros::Time(0))
    {
      int zone;
      bool isNorthHemisphere;
      double northing, easting, utmGridConvergence, projectionScale;
      GeographicLib::UTMUPS::Forward(this->lastFix.latitude, this->lastFix.longitude,
        zone, isNorthHemisphere, easting, northing, utmGridConvergence, projectionScale);
      utmGridConvergence = angles::from_degrees(utmGridConvergence);

      tf2::Quaternion utmGridConvergenceQuat;
      utmGridConvergenceQuat.setRPY(0, 0, utmGridConvergence);
      
      const auto utmHeading = utmGridConvergenceQuat * trueAzimuth;
      this->data->utmPublishers.publishAzimuths(utmHeading, this->variance, mag.header.stamp, this->lastImuInBody);
    }
  }
}

void AzimuthPublishersConfig::publishAzimuths(
  const tf2::Quaternion& nedAzimuth, const double variance, const ros::Time& stamp, const sensor_msgs::Imu& imuInBody)
{
  if (!this->publish)
    return;

  if (this->ned.publish)
  {
    auto imuNed = imuInBody;  // If IMU message should not be published, we fake it here with the ENU-referenced one
    if (this->ned.publishImu)
    {
      geometry_msgs::TransformStamped tf;
      tf.header.stamp = imuInBody.header.stamp;
      tf.header.frame_id = imuInBody.header.frame_id + "_ned";
      // do not use tf2::doTransform(Imu), it's buggy!
      tf2::convert(this->enuToNed, tf.transform.rotation);
      compass::doTransform(imuInBody, imuNed, tf);
    }
    this->ned.publishAzimuths(nedAzimuth, variance, stamp, imuNed);
  }

  if (this->enu.publish)
  {
    // Rotate to ENU
    auto enuAzimuth = this->nedToEnu * nedAzimuth;
    
    // Cancel out all non-yaw parts of the quaterion
    this->enu.rotMatrix = static_cast<tf2::Matrix3x3>(enuAzimuth);
    this->enu.rotMatrix.getRPY(this->enu.roll, this->enu.pitch, this->enu.yaw);
    enuAzimuth.setRPY(0, 0, this->enu.yaw);

    this->enu.publishAzimuths(enuAzimuth, variance, stamp, imuInBody);
  }
}

void AzimuthPublishersConfigForOrientation::publishAzimuths(
  const tf2::Quaternion& azimuth, const double variance, const ros::Time& stamp, const sensor_msgs::Imu& imuInBody)
{
  if (this->publishQuat || this->publishImu || this->publishPose)
  {
    tf2::convert(azimuth, this->quatMsg.quaternion);
    
    if (this->publishQuat)
    {
      this->quatMsg.header.stamp = stamp;
      this->quatPub.publish(this->quatMsg);
    }
    
    if (this->publishImu)
    {
      // The IMU message comes in an arbitrarily-referenced frame, and we adjust its yaw to become georeferenced.
      double azimuthYaw;
      this->rotMatrix = static_cast<tf2::Matrix3x3>(azimuth);
      this->rotMatrix.getRPY(this->roll, this->pitch, azimuthYaw);
      
      tf2::Quaternion imuRot;
      tf2::convert(imuInBody.orientation, imuRot);
      this->rotMatrix = static_cast<tf2::Matrix3x3>(imuRot);
      this->rotMatrix.getRPY(this->roll, this->pitch, this->yaw);

      tf2::Quaternion desiredRot;
      desiredRot.setRPY(this->roll, this->pitch, azimuthYaw);
      
      const auto diffRot = desiredRot * imuRot.inverse();

      geometry_msgs::TransformStamped tf;
      tf.header = imuInBody.header;
      // do not use tf2::doTransform(Imu), it's buggy!
      tf2::convert(diffRot, tf.transform.rotation);
      compass::doTransform(imuInBody, this->imuMsg, tf);
      
      this->imuMsg.orientation_covariance[8] = variance;
      
      this->imuPub.publish(this->imuMsg);
    }
    
    if (this->publishPose)
    {
      this->poseMsg.header.stamp = stamp;
      this->poseMsg.pose.pose.orientation = this->quatMsg.quaternion;
      this->poseMsg.pose.covariance[35] = variance;
      this->posePub.publish(this->poseMsg);
    }
  }

  if (this->publishDeg || this->publishRad)
  {
    this->azimuthMsg.header.stamp = stamp;
    this->azimuthMsg.variance = variance;

    this->rotMatrix = static_cast<tf2::Matrix3x3>(azimuth);
    this->rotMatrix.getRPY(this->roll, this->pitch, this->yaw);

    const auto azimuthRad = angles::normalize_angle_positive(this->yaw);
    if (this->publishRad)
    {
      this->azimuthMsg.azimuth = azimuthRad;
      this->azimuthMsg.unit = compass_msgs::Azimuth::UNIT_RAD;
      this->radPub.publish(this->azimuthMsg);
    }

    if (this->publishDeg)
    {
      const auto azimuthDeg = angles::to_degrees(azimuthRad);
      this->azimuthMsg.azimuth = azimuthDeg;
      this->azimuthMsg.unit = compass_msgs::Azimuth::UNIT_DEG;
      this->degPub.publish(this->azimuthMsg);
    }
  }
}

std::string MagnetometerCompassNodelet::getBestMagneticModel(const ros::Time& date) const
{
  if (!this->forcedMagneticModelName.empty())
    return this->forcedMagneticModelName;

  const auto year = getYear(date);  // If the conversion failed, year would be 0, thus triggering the last branch.
  if (year >= 2020)
    return WMM2020;
  else if (year >= 2015)
    return WMM2015;
  else
    return WMM2010;
}

uint32_t MagnetometerCompassNodelet::getYear(const ros::Time& date) 
{
  // Convert the given date to struct tm
  const auto dateAsTimeT = static_cast<time_t>(date.sec);
  struct tm dateAsStructTm{};
  const auto result = gmtime_r(&dateAsTimeT, &dateAsStructTm);

  // This would be true if the time conversion failed (it should never happen, but...)
  if (result == nullptr)
    return 0;

  return dateAsStructTm.tm_year + 1900;
}

double MagnetometerCompassNodelet::getMagneticDeclination(const sensor_msgs::NavSatFix& fix) const
{
  const auto year = getYear(fix.header.stamp);
  if (this->data->magneticModels[year] == nullptr)
  {
    const auto model = this->getBestMagneticModel(fix.header.stamp);
    
    try
    {
      this->data->magneticModels[year] =
        std::make_unique<GeographicLib::MagneticModel>(model, this->magneticModelsPath);
    }
    catch (const GeographicLib::GeographicErr& e)
    {
      ROS_ERROR_THROTTLE(1.0, "Could not create magnetic field model %s for year %u because of the following error: %s",
        model.c_str(), year, e.what());
      return 0.0;
    }
    
    NODELET_INFO("Initialized magnetic model %s for year %u.", model.c_str(), year);

    const auto& magModel = *this->data->magneticModels[year];
    if (year < magModel.MinTime() || year > magModel.MaxTime())
      NODELET_ERROR("Using magnetic field model %s for an invalid year %u!", model.c_str(), year);
  }

  const auto& magModel = *this->data->magneticModels[year];
  
  if (fix.altitude < magModel.MinHeight() || fix.altitude > magModel.MaxHeight())
  {
    NODELET_ERROR_THROTTLE(10.0, "Using magnetic field model %s in altitude %.0f m which is outside the model range.",
      magModel.MagneticModelName().c_str(), fix.altitude);
  }
  
  double Bx, By, Bz;
  magModel(year, fix.latitude, fix.longitude, fix.altitude, Bx, By, Bz);

  double H, F, D, I;
  GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);
  
  return angles::from_degrees(D);
}

void MagnetometerCompassNodelet::fixCb(const sensor_msgs::NavSatFix& fix)
{
  this->lastFix = fix;
  if (!this->magneticDeclinationForced)
    this->lastMagneticDeclination.setRPY(0, 0, this->getMagneticDeclination(fix));
}

void MagnetometerCompassNodelet::magBiasCb(const sensor_msgs::MagneticField& bias)
{
  tf2::convert(bias.magnetic_field, this->lastMagBias);
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > covMatrix(bias.magnetic_field_covariance.data());
  if (covMatrix.cwiseAbs().sum() > 1e-10)
    this->lastMagScale.setFromOpenGLSubMatrix(covMatrix.transpose().data());
  this->hasMagBias = true;
}

void MagnetometerCompassNodelet::updateVariance()
{
  // TODO: measure consistency of IMU rotation and azimuth increase similar to
  //  https://www.sciencedirect.com/science/article/pii/S2405959519302929
}

}

PLUGINLIB_EXPORT_CLASS(compass::MagnetometerCompassNodelet, nodelet::Nodelet)