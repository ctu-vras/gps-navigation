from odom_fix_matching import OdomMatcher
from copy import copy
import numpy as np

# Into this dictionary put all the bag file names you want processed with the key
# being the name of the robot.
bags = {
'tradr':[],

'spot': ['/home/robot/0908_bags/spot_2022-09-08-13-54-22.bag',],

'husky': []
}

# For each key in 'bags' dictionary create an array of odometry topics to be matched
# and visualised. If a topic does not occur in a particular file that is fine.
odom_topics = {
    'tradr':["/icp_odom","gps_odom","odometry/filtered"],
    'spot': ["/icp_odom","/spot/odometry"],
    'husky': ["/icp_odom","/imu_and_wheel_odom"],
    'marv': ["/imu_odom","/odom"]
}

joy_topics = {
    'tradr': "/local_joy/cmd_vel",
    'spot':  "/joy_local/cmd_vel",
    'husky': None,
    'marv':  None
}


for robot in bags.keys():
    for i,bag in enumerate(bags[robot]):

        bag = OdomMatcher(
            bag,                        # Bag file name.
            bag[:-4],                  # File name used for saving figures.
            copy(odom_topics[robot]),         # Odometry topics.
            use_weights     = True,     # Weight matching based on covariance.
            fix_topic       = '/fix',   # Fix topic name.
            joy_topic       = None, #joy_topics[robot], # Joystick topic name.
            switch_w_h      = False,    # Switch covariance of lat and lon.
            use_odom_covs   = False,    # Use the covariance of the odometries.
            produce_animation = True,   # Produce an animation of the movement of the robot.
            match_z         = False,     # Match z of odom with fix.
            forced_background_coords = np.array([[445000,5542500],[445500,5543300]]),
        )

        bag.run()

