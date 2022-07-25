# What is this

This is a python package used to match and visualise odometry and fix data obtained from topics of a .bag file.

The matching is done by finding time-wise corresponding pairs of points of two topics (either odometry and fix or two odometries). Then a transformation matrix is found, which after applying on one of the two topics minimises the sum of squared distances between the pairs of points.

Each two topics are matched and visualised in a figure. The mean distance between the pairs of points is calculated and is part of the plots.

If fix is available, an animation of the movement of the robot is created from the matched arrays.

# How to run

-   Install requirements:

    **pip install -r ./requirements.txt**

-   Either run matching on a single bag file...:

    **python3 ./match_single.py bag_path odom1,odom2,... use_weights(bool) produce_animation(bool)**

-   For example:

    **python3 ./match_single.py /home/data/spot.bag /icp_odom,/spot/odometry true true**

-   ...or process multiple bag files:
    1)  Open ./match_multiple.py
    2)  Edit the *bags* dictionary variable.
    3)  Edit the *odom_topics* dictionary variable.
    4)  (optional) Change parameters of *bag=OdomMatcher(...)*.
    5)  Run command: **python3 ./match_multiple.py**
