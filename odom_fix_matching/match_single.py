from odom_fix_matching import OdomMatcher
import sys

help_string =   "---------------\n\
Run as follows:\n\
python3 ./match_single.py bag_path odom1,odom2,... use_weights(bool) produce_animation(bool)\n\
e.g. python3 ./match_single.py /home/data/spot.bag /icp_odom,/spot/odometry true true\n\
---------------\n"

if len(sys.argv) == 5:

    bag_file = sys.argv[1]

    odometry_topic_names = sys.argv[2].split(',')

    weights = sys.argv[3]
    if weights.lower() == "false":
        weights = False
    else:
        weights = True

    ani = sys.argv[4]
    if ani.lower() == "false":
        ani = False
    else:
        ani = True
else:
    print("Not enough or too many arguments. Expected 4 got {}.".format(len(sys.argv) - 1))
    print(help_string)
    quit()


bag = OdomMatcher(
    bag_file,                   # Bag file name.
    bag_file[:-4],              # File name used for saving figures.
    odometry_topic_names,       # Odometry topics.
    use_weights     = weights,  # Weight matching based on covariance.
    fix_topic       = '/fix',   # Fix topic name.
    joy_topic       = '/local_joy/cmd_vel', # Joystick topic name.
    switch_w_h      = False,    # Switch covariance of lat and lon.
    use_odom_covs   = False,    # Use the covariance of the odometries.
    produce_animation = ani,    # Produce an animation of the movement of the robot.
    match_z         = False     # Match z of odom with fix.
)

bag.run()

