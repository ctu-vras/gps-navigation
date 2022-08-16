from odom_fix_matching import OdomMatcher
from copy import copy

# Into this dictionary put all the bag file names you want processed with the key
# being the name of the robot.
bags = {
'tradr':['fn1','fn2','...'],

'spot': [],

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


for robot in bags.keys():
    for i,bag in enumerate(bags[robot]):

        bag = OdomMatcher(
            bag,                        # Bag file name.
            bag[:-4],                  # File name used for saving figures.
            copy(odom_topics[robot]),         # Odometry topics.
            use_weights     = True,     # Weight matching based on covariance.
            fix_topic       = '/fix',   # Fix topic name.
            switch_w_h      = False,    # Switch covariance of lat and lon.
            use_odom_covs   = False,    # Use the covariance of the odometries.
            produce_animation = True,   # Produce an animation of the movement of the robot.
            match_z         = False     # Match z of odom with fix.
        )

        bag.run()

