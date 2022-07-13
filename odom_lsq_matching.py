# %%
import rosbag
import rospy
import tf2_ros
import matplotlib.pyplot as plt
import numpy as np
import math
import sys
from scipy.spatial import KDTree
import utm
from PIL import Image

# %%
class Point():
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

class Position():
    def __init__(self,x,y,z,cov_x,cov_y,cov_z,t):
        self.position = Point(x,y,z)
        self.cov = Point(cov_x,cov_y,cov_z)
        self.time = t

class OdomMatcher():
    """
        Class for matching odometry to fix.

        Input data in the form of a bag file. The bag file is parsed for the odometries and the fix.
        Corresponding points between an odom and fix are then found. Match is obtained by least squares
        matching of the corresponding points.
        
        Output is mean distance between corresponding points and a picture of the matched points for each odometry.
    """
    def __init__(self,bag_path,fix_topic,odom_topics, use_weights):
        self.bag_path = bag_path
        self.bag = rosbag.Bag(self.bag_path)
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(1000000))
        self.fix_topic = fix_topic
        self.odom_topics = odom_topics
        self.use_weights = use_weights

    def fill_tf_buffer(self):
        """ Currently not used. """
        for topic, msg, t in self.bag.read_messages(topics=['/tf', '/tf_static']):
            for msg_tf in msg.transforms:
                if topic == '/tf_static':
                    self.tfBuffer.set_transform_static(msg_tf, "default_authority")
                else:
                    self.tfBuffer.set_transform(msg_tf, "default_authority")

    def get_odom_frame(self,odom):
        """ Currently not used. """
        for topic, msg, t in self.bag.read_messages(topics=[odom]):
            return msg.header.frame_id

    def parse_topics(self):
        """ Save messages from desired topics (fix,odoms) as lists of points with time."""
        self.odometries = {}
        self.odometries_t = {}
        self.fixes = []
        self.fixes_t = []

        for topic in self.odom_topics:
            self.odometries[topic] = []
            self.odometries_t[topic] = []

        for topic, msg, t in self.bag.read_messages(topics=self.odom_topics + [self.fix_topic]):
            if topic in self.odom_topics:
                position = Position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,0,0,0,(msg.header.stamp).to_nsec())
                self.odometries[topic].append(position)
                self.odometries_t[topic].append((msg.header.stamp).to_nsec())
            
            else:
                easting,northing,_,_ = utm.from_latlon(msg.latitude,msg.longitude)
                position = Position(easting,northing,0,msg.position_covariance[0],msg.position_covariance[4],0,(msg.header.stamp).to_nsec())
                self.fixes.append(position)
                self.fixes_t.append((msg.header.stamp).to_nsec())

    def get_odom_correspondences(self):
        """ 
            Finds a corresponding odom point for each fix point or vice versa (if fix has more points than odom).
            Matching is done via a KDTree.
        """
        self.odom_correspondences = {}

        for odom in self.odometries:
            self.odom_correspondences[odom] = [[],[]]

            shorter_list = []
            longer_list = []

            if len(self.odometries[odom]) >= len(self.fixes):
                shorter_list = np.array(self.fixes)
                longer_list = np.array(self.odometries[odom])
                longer_t = np.array(self.odometries_t[odom])
            else:
                shorter_list = np.array(self.odometries[odom])
                longer_list = np.array(self.fixes)
                longer_t = np.array(self.fixes_t)

            tree = KDTree(longer_t.reshape(-1,1))

            shorter_times = np.array([p.time for p in shorter_list]).reshape(-1,1)
            _,indices = tree.query(shorter_times, k=1)
            correspondence = longer_list[indices]

            self.odom_correspondences[odom][0] = shorter_list
            self.odom_correspondences[odom][1] = correspondence
        return

    def nearest_orthonormal(self, M):
        assert M.ndim == 2
        assert M.shape[0] == M.shape[1]
        U, s, V = np.linalg.svd(M, full_matrices=False)
        R = np.matmul(U, V)
        return R

    def get_transformation_matrix(self, x, y, w = None):
        """Find transform R, t between x and y, such that the sum of squared
        distances ||R * x[:, i] + t - y[:, i]|| is minimum.

        :param x: Points to align, D-by-M array.
        :param y: Reference points to align to, D-by-M array.

        :return: Optimized transform from SE(D) as (D+1)-by-(D+1) array,
            T = [R t; 0... 1].
        """
        assert x.shape == y.shape, 'Inputs must be same size.'
        assert x.shape[1] > 0
        assert y.shape[1] > 0
        d = x.shape[0]
        T = np.eye(d + 1)

        # Center points.
        x_mean = x.mean(axis=1, keepdims=True)
        y_mean = y.mean(axis=1, keepdims=True)
        x_centered = x - x_mean
        y_centered = y - y_mean

        # Avoid loop through individual vectors.
        # M = x_centered @ y_centered.T
        if w is not None and self.use_weights:
            W = np.diag(w)
            M = np.matmul(x_centered, W)
            M = np.matmul(M, y_centered.T)
        else:
            M = np.matmul(x_centered, y_centered.T)
        R = self.nearest_orthonormal(M).T

        if d == 3 and not np.isclose(np.linalg.det(R), 1.0):
            raise ValueError("Rotation R, R'*R = I, det(R) = 1, could not be found.")

        t = y_mean - np.matmul(R, x_mean)
        T[:-1, :-1] = R
        T[:-1, -1:] = t

        return T

    def match_odom_and_fix(self):
        self.odom_T_matrices = {}
        self.matched_points = {}
        for odom in self.odom_correspondences:
            x = np.array([[pos.position.x, pos.position.y, pos.position.z] for pos in self.odom_correspondences[odom][0]]).T
            y = np.array([[pos.position.x, pos.position.y, pos.position.z] for pos in self.odom_correspondences[odom][1]]).T
            w = np.array([pos[0].cov.x + pos[0].cov.y + pos[0].cov.z + pos[1].cov.x + pos[1].cov.y + pos[1].cov.z for pos in zip(self.odom_correspondences[odom][0],self.odom_correspondences[odom][1])])
            self.w = w
            T = self.get_transformation_matrix(x,y,w)
            self.odom_T_matrices[odom] = T
            
            x = (T[:-1, :-1] @ x + T[:-1, -1:])     # Apply the transformation.
            self.matched_points[odom] = [x,y]   
    
    def run(self):
        self.parse_topics()
        self.get_odom_correspondences()
        self.match_odom_and_fix()
        self.visualise_matched()
        self.get_mde()

    def visualise_centered_original(self):
        """ Visualise data before rotation matching. """
        fix_positions = np.array([[pos.position.x,pos.position.y] for pos in self.fixes])
        fix_center = np.mean(fix_positions,axis=0)
        plt.scatter(fix_positions[:,0],fix_positions[:,1])

        for odom in self.odometries:
            odom_positions = np.array([[pos.position.x,pos.position.y] for pos in self.odom_correspondences[odom][1]])
            odom_positions += fix_center
            plt.scatter(odom_positions[:,0],odom_positions[:,1])

        plt.show()
    
    def get_mde(self):
        """ MDE is the mean distance between a pair of corresponding points. """
        for odom in self.odom_topics:
            diff = self.matched_points[odom][0] - self.matched_points[odom][1]
            squared = np.square(diff)
            summed = np.sum(squared,axis=0)
            sqrt = np.sqrt(summed)
            if self.use_weights:
                sqrt = sqrt * self.w/np.sum(self.w) * len(sqrt)
            mde = np.mean(sqrt)

            if self.use_weights:
                print("Weighted MDE of odometry {} is {}. (work in progress)".format(odom,mde))
            else:
                print("MDE of odometry {} is {}.".format(odom,mde))

    def visualise_matched(self):
        """ Visualise points of fix and odoms after their matching. """
        for odom in self.odom_topics:
            points1 = self.matched_points[odom][0]
            points2 = self.matched_points[odom][1]

            fig, ax = plt.subplots(figsize=(8,8), dpi=400)
            
            ax.scatter(points1[0,:],points1[1,:],s=1)
            ax.scatter(points2[0,:],points2[1,:],s=1)
            ax.set_title(odom)
            plt.show()


    def __del__(self):
        self.bag.close()

# %%
if len(sys.argv) >= 4:
    USE_WEIGHTS = sys.argv[3]
    if USE_WEIGHTS.lower() == "false":
        USE_WEIGHTS = False
    else:
        USE_WEIGHTS = True
else:
    USE_WEIGHTS = False

if len(sys.argv) >= 2:
    bag_file = sys.argv[1]
else:
    bag_file = "/home/robot/Downloads/spot_2022-06-16-14-12-32.no_sensors.bag"

if len(sys.argv) >= 3:
    odometry_topic_names = sys.argv[2].split(',')
else:
    odometry_topic_names = ["/icp_odom","/spot/odometry"]

bag = OdomMatcher(bag_file, "/fix", odometry_topic_names, use_weights=USE_WEIGHTS)

bag.run()

 


