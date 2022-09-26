# magnetometer\_compass

Compass based on a 3-axis magnetometer, attitude readings and possibly also GPS.

Because there is no well established Azimuth message in ROS, this node publishes our custom `compass_msgs/Azimuth`
as well as a few other formats that are capable of carrying orientation information. It also offers the azimuth
values in both radians and degrees, because radians are the ROS standard, while degrees are more used in the
geographic area. There are tens of possible combinations of the output data formats, so each of the published
topics has a boolean parameter that enables it. By default, there is only one enabled output topic.

## Explaining some basic terms

So that you know what output is the right for you.

### Orientation
- NED (North-East-Down): Azimuth will be 0 pointing to north, and increases clockwise. This is consistent with the
                         azimuth used in cartography and by tourists.
- ENU (East-North-Up): Azimuth will be 0 pointing to east, and increases counter-clockwise. This is consistent with
                       REP-103 and robot_localization package.
### References for north
- Magnetic: points towards the magnetic north of Earth (travels in time).
- Geographic ("true"): points towards the geographic Earth (i.e. the WGS84 North Pole). It is static in time.
- UTM: points in the north direction on the cartesian UTM grid (similar to Geographic, but it can slightly diverge
       at the edges of UTM maps). You probably want this azimuth reference for navigation tasks in UTM coordinates.

Magnetic azimuth can be computed directly from the magnetometer and IMU orientation. To compute the other two
references, you need to provide the latitude, longitude, altitude and time in addition to the magnetometer and
IMU orientation. These are the required inputs to compute magnetic declination and UTM grid convergence, which are
the offsets by which geographic and UTM references differ from the magnetic. This is why this compass node subscribes
to the GPS fix messages. Until at least a single GPS fix message is received, neither geographic- nor UTM-referenced
data are published. If you do not have a GPS receiver, you can alternatively provide these values in parameters.

For the magnetometer to work correctly, it is required to measure its bias. This node listens on the `imu/mag_bias`
topic for this measurement, and until at least one message arrives, the node will not publish anything. If you do not
have a node publishing the bias, you can alternatively provide it via parameters. Depending on the application, it
may be required to re-estimate the bias from time to time even during runtime.

## Node compass\_node and nodelet magnetometer\_compass/Compass

### Subscribed topics
- `imu/data` (`sensor_msgs/Imu`): Output from an IMU or an orientation filtering algorithm. It should have valid
                                  contents of `orientation` and at least roll and pitch should be estimated as
                                  well as possible (relative to the gravity vector). These messages should come at
                                  the same rate as the magnetometer data (or faster).
- `imu/mag` (`sensor_msgs/MagneticField`): 3-axis magnetometer measurements (bias not removed).
- `imu/mag_bias` (`sensor_msgs/MagneticField`): Bias of the magnetometer. This value will be subtracted from the
                                                incoming magnetometer measurements (only the `magnetic_field` field
                                                is relevant). Messages on this topic do not need to come repeatedly
                                                if the bias does not change.
- `gps/fix` (`sensor_msgs/NavSatFix`, optional): GPS fix messages from which the latitude, longitude, altitude and
                                                 current year can be read. These are further used to compute
                                                 magnetic declination and UTM grid convergence factor if requested.
- TF: This node requires a (usually static) transform between `~frame` and the frame ID of the IMU and magnetometer
      messages.

### Published topics (see above for explanation)
- `imu/mag_unbiased` (`sensor_msgs/MagneticField`, enabled by param `~publish_mag_unbiased`, off by default):
    The magnetic field measurement with bias removed.

- `compass/mag/ned/deg` (`compass_msgs/Azimuth`, enabled by param `~publish_mag_azimuth_ned_deg`, on by default):
    Magnetic azimuth in NED in degrees (the same values you can see on touristic magnetic compasses).
- `compass/mag/ned/rad` (`compass_msgs/Azimuth`, enabled by param `~publish_mag_azimuth_ned_rad`, off by default):
    Magnetic azimuth in NED in radians.
- `compass/mag/ned/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_mag_azimuth_ned_quat`, off by default):
    Magnetic azimuth in NED as a quaternion.
- `compass/mag/ned/imu` (`sensor_msgs/Imu`, enabled by param `~publish_mag_azimuth_ned_imu`, off by default):
    Magnetic azimuth in NED inside an IMU message (only `orientation` and `header` fields are valid).
- `compass/mag/ned/pose` (`geometry_msgs/PoseStamped`, enabled by param `~publish_mag_azimuth_ned_pose`, off by default):
    Magnetic azimuth in NED as a pose (translation will always be zero).

- `compass/mag/enu/deg` (`compass_msgs/Azimuth`, enabled by param `~publish_mag_azimuth_enu_deg`, off by default):
    Magnetic azimuth in ENU in degrees.
- `compass/mag/enu/rad` (`compass_msgs/Azimuth`, enabled by param `~publish_mag_azimuth_enu_rad`, off by default):
    Magnetic azimuth in ENU in radians.
- `compass/mag/enu/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_mag_azimuth_enu_quat`, off by default):
    Magnetic azimuth in ENU as a quaternion.
- `compass/mag/enu/imu` (`sensor_msgs/Imu`, enabled by param `~publish_mag_azimuth_enu_imu`, off by default):
    Magnetic azimuth in ENU inside an IMU message (only `orientation` and `header` fields are valid).
- `compass/mag/enu/pose` (`geometry_msgs/PoseStamped`, enabled by param `~publish_mag_azimuth_enu_pose`, off by default):
    Magnetic azimuth in ENU as a pose (translation will always be zero).

- `compass/true/ned/deg` (`compass_msgs/Azimuth`, enabled by param `~publish_true_azimuth_ned_deg`, off by default):
    Geographic ("true") azimuth in NED in degrees.
- `compass/true/ned/rad` (`compass_msgs/Azimuth`, enabled by param `~publish_true_azimuth_ned_rad`, off by default):
    Geographic ("true") azimuth in NED in radians.
- `compass/true/ned/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_true_azimuth_ned_quat`, off by default):
    Geographic ("true") azimuth in NED as a quaternion.
- `compass/true/ned/imu` (`sensor_msgs/Imu`, enabled by param `~publish_true_azimuth_ned_imu`, off by default):
    Geographic ("true") azimuth in NED inside an IMU message (only `orientation` and `header` fields are valid).
- `compass/true/ned/pose` (`geometry_msgs/PoseStamped`, enabled by param `~publish_true_azimuth_ned_pose`, off by default):
    Geographic ("true") azimuth in NED as a pose (translation will always be zero).

- `compass/true/enu/deg` (`compass_msgs/Azimuth`, enabled by param `~publish_true_azimuth_enu_deg`, off by default):
    Geographic ("true") azimuth in ENU in degrees.
- `compass/true/enu/rad` (`compass_msgs/Azimuth`, enabled by param `~publish_true_azimuth_enu_rad`, off by default):
    Geographic ("true") azimuth in ENU in radians.
- `compass/true/enu/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_true_azimuth_enu_quat`, off by default):
    Geographic ("true") azimuth in ENU as a quaternion.
- `compass/true/enu/imu` (`sensor_msgs/Imu`, enabled by param `~publish_true_azimuth_enu_imu`, off by default):
    Geographic ("true") azimuth in ENU inside an IMU message (only `orientation` and `header` fields are valid).
- `compass/true/enu/pose` (`geometry_msgs/PoseStamped`, enabled by param `~publish_true_azimuth_enu_pose`, off by default):
    Geographic ("true") azimuth in ENU as a pose (translation will always be zero).

- `compass/utm/ned/deg` (`compass_msgs/Azimuth`, enabled by param `~publish_utm_azimuth_ned_deg`, off by default):
    UTM heading in NED in degrees.
- `compass/utm/ned/rad` (`compass_msgs/Azimuth`, enabled by param `~publish_utm_azimuth_ned_rad`, off by default):
    UTM heading in NED in radians.
- `compass/utm/ned/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_utm_azimuth_ned_quat`, off by default):
    UTM heading in NED as a quaternion.
- `compass/utm/ned/imu` (`sensor_msgs/Imu`, enabled by param `~publish_utm_azimuth_ned_imu`, off by default):
    UTM heading in NED inside an IMU message (only `orientation` and `header` fields are valid).
- `compass/utm/ned/pose` (`geometry_msgs/PoseStamped`, enabled by param `~publish_utm_azimuth_ned_pose`, off by default):
    UTM heading in NED as a pose (translation will always be zero).

- `compass/utm/enu/deg` (`compass_msgs/Azimuth`, enabled by param `~publish_utm_azimuth_enu_deg`, off by default):
    UTM heading in ENU in degrees.
- `compass/utm/enu/rad` (`compass_msgs/Azimuth`, enabled by param `~publish_utm_azimuth_enu_rad`, off by default):
    UTM heading in ENU in radians.
- `compass/utm/enu/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_utm_azimuth_enu_quat`, off by default):
    UTM heading in ENU as a quaternion.
- `compass/utm/enu/imu` (`sensor_msgs/Imu`, enabled by param `~publish_utm_azimuth_enu_imu`, off by default):
    UTM heading in ENU inside an IMU message (only `orientation` and `header` fields are valid).
- `compass/utm/enu/pose` (`geometry_msgs/PoseStamped`, enabled by param `~publish_utm_azimuth_enu_pose`, off by default):
    UTM heading in ENU as a pose (translation will always be zero).

### Parameters
- all the `publish_*` parameters mentioned above.
- `~frame` (string, default `base_link`): Frame into which the IMU and magnetometer data should be transformed.
- `~low_pass_ratio` (double, default 0.95): The azimuth is filtered with a low-pass filter. This sets its
                                            aggressivity (0 means raw measurements, 1 means no updates).
- `~initial_mag_bias_x` (double, no default, optional): Magnetometer bias in the X axis.
- `~initial_mag_bias_y` (double, no default, optional): Magnetometer bias in the Y axis.
- `~initial_mag_bias_z` (double, no default, optional): Magnetometer bias in the Z axis.
  - If you specify any of the `~initial_mag_bias_*` params, the node does not need to receive the bias messages.
- `~initial_lat` (double, no default, optional): Latitude in degrees.
- `~initial_lon` (double, no default, optional): Longitude in degrees.
  - If you specify both `~initial_lat` and `~initial_lon`, the node does not need to receive the GPS fix messages.
- `~initial_alt` (double, default 0): Altitude in meters (it is usually okay to omit it and use the default).
- `~initial_year` (int, no default, optional): If set, overrides the current time for declination computation.
- `~magnetic_models_path` (string, defaults to the pre-installed directory): Directory with WMM magnetic field
     models. You usually do not need to use other than the preinstalled models. But if you do, specify the path to
     the custom models directory here.
- `~magnetic_model` (string, defaults to autodetection by year): Name of the magnetic field model to use. If omitted,
     an automated decision is made based on the current year (or `~initial_year`, if set). This model is used for
     computing magnetic declination.

## Node visualize\_azimuth

This node visualizes an `Azimuth` messages in RViz converting them to a pose.

### Subscribed topics

- `~azimuth` (`compass_msgs/Azimuth`): The azimuth to visualize.

### Published topics

- `~azimuth_vis` (`geometry_msgs/PoseWithCovarianceStamped`): The pose which visualizes the azimuth.