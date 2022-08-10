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
#include <geometry_msgs/PoseStamped.h>
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

#include <magnetometer_compass/compass_nodelet.h>

#define WMM2010 "wmm2010"
#define WMM2015 "wmm2015v2"
#define WMM2020 "wmm2020"

namespace cras
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
  geometry_msgs::PoseStamped poseMsg;

  tf2::Matrix3x3 rotMatrix;
  double roll {}, pitch {}, yaw {};

  void init(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& frameId, const std::string& paramPrefix, 
    const std::string& topicPrefix, uint8_t reference, uint8_t orientation,
    const std::string& referenceStr, const std::string& orientationStr);

  void publishAzimuths(const tf2::Quaternion& azimuth, const ros::Time& stamp);
};

struct AzimuthPublishersConfig
{
  uint8_t reference {};

  AzimuthPublishersConfigForOrientation ned;
  AzimuthPublishersConfigForOrientation enu;

  bool publish {false};

  tf2::Quaternion nedToEnu {-M_SQRT2/2, -M_SQRT2/2, 0, 0};
  
  void init(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& frameId,
    const std::string& paramPrefix, const std::string& topicPrefix, uint8_t reference, const std::string& referenceStr);

  void publishAzimuths(const tf2::Quaternion& nedAzimuth, const ros::Time& stamp);
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
  this->magneticModelsPath = pnh.param(
    "magnetic_models_path", ros::package::getPath("magnetometer_compass") + "/data/magnetic");
  this->forcedMagneticModelName = pnh.param("magnetic_model", std::string());

  if (pnh.hasParam("initial_mag_bias_x") || pnh.hasParam("initial_mag_bias_y") || pnh.hasParam("initial_mag_bias_z"))
  {
    sensor_msgs::MagneticField msg;
    msg.magnetic_field.x = pnh.param("initial_mag_bias_x", this->lastMagBias.x);
    msg.magnetic_field.y = pnh.param("initial_mag_bias_y", this->lastMagBias.y);
    msg.magnetic_field.z = pnh.param("initial_mag_bias_z", this->lastMagBias.z);

    ROS_INFO("Initial magnetometer bias is %0.3f %0.3f %0.3f",
      msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);
    
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

    ROS_INFO(
      "Initial GPS coords for computation of declination and grid convergence are %.6f, %.6f, altitude %.0f m, year %u",
      msg.latitude, msg.longitude, msg.altitude, getYear(msg.header.stamp));
    
    this->fixCb(msg);
  }

  this->data->publishMagUnbiased = pnh.param("publish_mag_unbiased", this->data->publishMagUnbiased);
  
  // Set default publishers
  this->data->magPublishers.ned.publishDeg = true;
  
  bool publish = this->data->publishMagUnbiased;

  // Read the publish_* paramters and set up the selected publishers
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
  
//  geometry_msgs::TransformStamped t;
//  t.header.stamp.sec = ros::WallTime::now().sec;
//  t.header.frame_id = "base_link";
//  t.child_frame_id = "imu";
//  tf2::Quaternion q;
//  q.setRPY(M_PI, 0, M_PI_2);
//  tf2::convert(q, t.transform.rotation);
//  this->tf.setTransform(t, "", true);

//  sensor_msgs::MagneticField bias;
//  bias.magnetic_field.x = 10;
//  bias.magnetic_field.y = 1;
//  bias.magnetic_field.z = -2;
//  this->magBiasCb(bias);
//  
//  sensor_msgs::NavSatFix msg;
//  msg.header.stamp.sec = ros::WallTime::now().sec;
//  msg.latitude = 52;
//  msg.longitude = 15.5;
//  msg.altitude = 400;
//  this->fixCb(msg);
//  
//  sensor_msgs::Imu imuMsg;
//  imuMsg.header.stamp.sec = ros::WallTime::now().sec;
//  imuMsg.header.frame_id = "imu";
//  imuMsg.orientation.w = 1;
//  
//  sensor_msgs::MagneticField mag;
//  mag.header.stamp.sec = ros::WallTime::now().sec;
//  mag.header.frame_id = "imuMsg";
//  mag.magnetic_field.x = 100;
//  mag.magnetic_field.y = 10;
//  mag.magnetic_field.z = -20;
//  
//  this->imuMagCb(imuMsg, mag); 
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
  if (!this->hasMagBias)
  {
    ROS_ERROR_DELAYED_THROTTLE(10.0, "Magnetometer bias not available, compass cannot work. Waiting...");
    return;
  }

  sensor_msgs::Imu imuInBody;
  try
  {
    imuInBody = this->tf.transform(imu, this->frame, ros::Duration(0.1));
  }
  catch (const tf2::TransformException& e)
  {
    NODELET_ERROR_THROTTLE(1.0, "Could not transform IMU data to frame %s because: %s", this->frame.c_str(), e.what());
    return;
  }

  sensor_msgs::MagneticField magUnbiased = mag;
  magUnbiased.magnetic_field.x -= this->lastMagBias.x;
  magUnbiased.magnetic_field.y -= this->lastMagBias.y;
  magUnbiased.magnetic_field.z -= this->lastMagBias.z;
  
  if (this->data->publishMagUnbiased)
    this->data->magUnbiasedPub.publish(magUnbiased);
  
  sensor_msgs::MagneticField magInBody;
  try
  {
    magInBody = this->tf.transform(magUnbiased, this->frame, ros::Duration(0.1));
  }
  catch (const tf2::TransformException& e)
  {
    NODELET_ERROR_THROTTLE(1.0, "Could not transform magnetometer to frame %s because: %s",
      this->frame.c_str(), e.what());
    return;
  }
  
  tf2::Quaternion rot;
  tf2::convert(imuInBody.orientation, rot);
  tf2::Matrix3x3 rotMatrix(rot);
  double roll, pitch, yaw;
  rotMatrix.getRPY(roll, pitch, yaw);

  // compensate tilt, no need to evaluate z component
  const auto magNorth =
    magInBody.magnetic_field.x * cos(pitch) +
    magInBody.magnetic_field.y * sin(pitch) * sin(roll) +
    magInBody.magnetic_field.z * sin(pitch) * cos(roll);

  const auto magEast =
    magInBody.magnetic_field.y * cos(roll) -
    magInBody.magnetic_field.z * sin(roll);

  // This formula gives north-referenced clockwise-increasing azimuthMsg
  const auto magAzimuthNow = -atan2(magEast, magNorth);
  tf2::Quaternion magAzimuthNowQuat;
  magAzimuthNowQuat.setRPY(0, 0, magAzimuthNow);
  
  if (this->magAzimuth.w() == 0.0)
    this->magAzimuth = magAzimuthNowQuat;
  else  // low-pass filter
    this->magAzimuth = this->magAzimuth.slerp(magAzimuthNowQuat, 1 - this->magAzimuthLowPass);
  this->data->magPublishers.publishAzimuths(this->magAzimuth, mag.header.stamp);
  
  if ((this->data->truePublishers.publish || this->data->utmPublishers.publish) &&
    this->lastMagneticDeclination.w() != 0.0)
  {
    const auto trueAzimuth = this->lastMagneticDeclination * this->magAzimuth;
    this->data->truePublishers.publishAzimuths(trueAzimuth, mag.header.stamp);
    
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
      this->data->utmPublishers.publishAzimuths(utmHeading, mag.header.stamp);
    }
  }
}

void AzimuthPublishersConfig::publishAzimuths(const tf2::Quaternion& nedAzimuth, const ros::Time& stamp)
{
  if (!this->publish)
    return;

  if (this->ned.publish)
    this->ned.publishAzimuths(nedAzimuth, stamp);

  if (this->enu.publish)
  {
    // Rotate to ENU
    auto enuAzimuth = this->nedToEnu * nedAzimuth;
    
    // Cancel out all non-yaw parts of the rotation
    this->enu.rotMatrix = static_cast<tf2::Matrix3x3>(enuAzimuth);
    this->enu.rotMatrix.getRPY(this->enu.roll, this->enu.pitch, this->enu.yaw);
    enuAzimuth.setRPY(0, 0, this->enu.yaw);

    this->enu.publishAzimuths(enuAzimuth, stamp);
  }
}

void AzimuthPublishersConfigForOrientation::publishAzimuths(const tf2::Quaternion& azimuth, const ros::Time& stamp)
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
      this->imuMsg.header.stamp = stamp;
      this->imuMsg.orientation = this->quatMsg.quaternion;
      this->imuPub.publish(this->imuMsg);
    }
    
    if (this->publishPose)
    {
      this->poseMsg.header.stamp = stamp;
      this->poseMsg.pose.orientation = this->quatMsg.quaternion;
      this->posePub.publish(this->poseMsg);
    }
  }

  if (this->publishDeg || this->publishRad)
  {
    this->azimuthMsg.header.stamp = stamp;
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
  this->lastMagneticDeclination.setRPY(0, 0, this->getMagneticDeclination(fix));
}

void MagnetometerCompassNodelet::magBiasCb(const sensor_msgs::MagneticField& bias)
{
  this->lastMagBias = bias.magnetic_field;
  this->hasMagBias = true;
}

}

PLUGINLIB_EXPORT_CLASS(cras::MagnetometerCompassNodelet, nodelet::Nodelet)