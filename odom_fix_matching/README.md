Still work in progress. Using covariance as weights might not be working exactly right.

RUN:

python3 odom_lsq_matching.py **path_to_bag** **odom_names_as_comma_separated_str** **use_cov_as_weight_bool**


EXAMPLE USAGE:

python3 odom_lsq_matching.py "/home/robot/Downloads/spot_2022-06-16-14-12-32.no_sensors.bag" "/icp_odom,/spot/odometry" False
