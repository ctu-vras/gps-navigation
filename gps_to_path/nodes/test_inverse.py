import rospy
from stamped_msgs.msg import Float64
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
from gps_to_path.msg import MagShift
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_from_euler
import tf2_ros
import math

import json
from numpy import deg2rad, rad2deg, NAN
from math import sin, cos, atan2, floor, pi

mag_azimuths = [290,280,275,272,271,270] 
# inverse atan2

for azimuth in mag_azimuths:
    x_NED = sin(azimuth)