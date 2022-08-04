import rosbag
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.axes_grid1 import make_axes_locatable
import matplotlib.animation as animation
from matplotlib.patches import Ellipse
import numpy as np
import math
from scipy.spatial import KDTree
from copy import copy
import utm
import warnings
from PIL import Image
from background_map import get_background_image

MIN_COV = 1e-02         # [m]
MAX_COV = 1e+03         # [m]
MAX_CORRESP_DELAY = 1   # [s]
NUM_TRIMMED_POINTS = 300
ELIPSE_SIGMA = 1

class Point():
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

class Position():
    def __init__(self,x,y,z,t,cov_x=None,cov_y=None,cov_z=None):
        self.position = Point(x,y,z)
        self.cov = Point(cov_x,cov_y,cov_z)
        self.time = t
    
    def position_arr(self):
        return [self.position.x, self.position.y, self.position.z]
    
    def cov_arr(self):
        return [self.cov.x, self.cov.y, self.cov.z]

class PositionsContainer():
    """ Used to store arrays of positions, which is how odometries and fix are represented. """
    def __init__(self):
        self.positions = []
    
    def __len__(self):
        return len(self.positions)

    def positions_arr(self):
        return np.array([p.position_arr() for p in self.positions])
    
    def covs_arr(self,none_col_sub=False):
        covs = np.array([p.cov_arr() for p in self.positions])
        not_none_cols = np.where(covs[0] != None)[0]    # Assume that a None col has None in each row.
        if none_col_sub is False:
            covs = covs[:,not_none_cols]        # Omit cols with None.
        elif none_col_sub is None:
            pass
        else:
            covs[covs==None] = none_col_sub     # Substitute None.
        
        return covs
    
    def time_arr(self):
        return np.array([p.time for p in self.positions])

class OdomMatcher():
    """
        Class for matching points between odometries and fix.

        Input data in the form of a bag file. The bag file is parsed for the odometries and the fix.
        Points from the different topics are then matches. Match is obtained by least squares matching
        of the corresponding points.
        
        Output is the mean distance between corresponding points and a figure of the matched points
        for each odometry. Also produces a gif of the robot's traversing of the recorder paths.
    """
    def __init__(self,bag_path,fn,odom_topics, use_weights, fix_topic=None, switch_w_h=False, use_odom_covs = False, produce_animation=True, match_z = False):
        self.bag_path = bag_path
        self.fn = fn
        print("Opening file {}.".format(self.bag_path))
        try:
            self.bag = rosbag.Bag(self.bag_path)
        except:
            print("\nCould not open bag file {}\n".format(self.bag_path))
            quit()
        self.fix_topic = fix_topic
        self.odom_topics = odom_topics
        self.use_weights = use_weights
        self.switch_w_h = switch_w_h
        self.use_odom_covs = use_odom_covs
        self.produce_animation = produce_animation
        self.match_z = match_z

        self.bg_image = None
        self.bg_image_animation = None

    def parse_topics(self):
        """ Save messages from desired topics (fix,odoms) as lists of points with time."""
        self.odometries = {}
        self.fixes = PositionsContainer()

        self.min_lat = 10000
        self.max_lat = -1
        self.min_long = 10000
        self.max_long = -1

        self.first_odom_values = {}

        for topic in copy(self.odom_topics):
            if topic[0] == '/':
                self.odom_topics.append(topic[1:])
                self.odometries[topic[1:]] = PositionsContainer()
                print("Adding topic {}.".format(topic[1:]))
            self.odometries[topic] = PositionsContainer()

        # The fix topic sometimes shows itself as 'fix' instead of '/fix'.
        # Apparently this also happens to odom topics...
        if self.fix_topic is not None and self.fix_topic[0] == '/':
            topics = self.odom_topics + [self.fix_topic, self.fix_topic[1:]]
        else:
            topics = self.odom_topics + [self.fix_topic]
            
        for topic, msg, t in self.bag.read_messages(topics=topics):
        
            # Save odom msg as Position object.
            if topic in self.odom_topics:
                if self.use_odom_covs:
                    position = Position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                                        (msg.header.stamp).to_nsec(),msg.pose.covariance[0],
                                        msg.pose.covariance[7], msg.pose.covariance[14])
                else:
                    position = Position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,(msg.header.stamp).to_nsec())
                self.odometries[topic].positions.append(position)

                # First odom values are used to translate two odometries to the same (0,0) beginning.
                if not topic in self.first_odom_values:
                    self.first_odom_values[topic] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]).reshape(-1,1)
            
            # Save fix msg as Position object.
            else:
                # Save lat/long extremes.
                if msg.latitude < self.min_lat:
                    self.min_lat = msg.latitude
                elif msg.latitude > self.max_lat:
                    self.max_lat = msg.latitude
                if msg.longitude < self.min_long:
                    self.min_long = msg.longitude
                elif msg.longitude > self.max_long:
                    self.max_long = msg.longitude

                # Transform to UTM.
                easting,northing,_,_ = utm.from_latlon(msg.latitude,msg.longitude)
                
                position = Position(easting,northing,0,(msg.header.stamp).to_nsec(),msg.position_covariance[4],msg.position_covariance[0],None)
                self.fixes.positions.append(position)

        topics_to_remove = []
        for topic in self.odom_topics:
            if self.odometries[topic].positions == []: 
                topics_to_remove.append(topic)
                
        for topic in topics_to_remove:
            self.odom_topics.remove(topic)
            self.odometries.pop(topic)
            print("Odom topic {} has not occured in bag file {} and won't be used.".format(topic,self.bag_path))

        if self.fix_topic and self.fixes.positions == []:
            print("Fix topic {} has not occured in bag file {} and won't be used.".format(self.fix_topic,self.bag_path))
            self.fix_topic = None
    
    def get_subsequent_distances(self,positions_container):
        """ For each point calculate the distances between two subsequent points. """

        p1 = positions_container.positions_arr()
        p2 = np.roll(p1,shift=-1,axis=0)

        p1 = p1[:-1,:]
        p2 = p2[:-1,:]
        
        dist = self.dist(p1,p2)
        dist = np.insert(dist,0,0)

        return dist

    def find_correspondences(self):
        """ 
            Finds a corresponding (i.e. nearest in time) odom point for each fix point
            or vice versa (if fix has more points than odom). Matching is done via a KDTree.
        """
        self.correspondences = {}
        self.distances = {}

        for i in range(len(self.odometries)):
            odom_key = self.odom_topics[i]             
            odom = self.odometries[odom_key]

            self.correspondences[odom_key] = {}
            self.distances[odom_key] = {}

            for j in range(len(self.odometries) - i):
                # Matching with fix
                if j == 0:
                    if self.fix_topic is not None:
                        odom_or_fix_key = self.fix_topic
                        odom_or_fix = self.fixes
                    # No fix - no matching
                    else:continue
                # Matching with odom
                else:
                    odom_or_fix_key = self.odom_topics[i + j] 
                    odom_or_fix = self.odometries[odom_or_fix_key]
                
                self.correspondences[odom_key][odom_or_fix_key] = [[],[]]

                # Out of the two arrays of points, the shorter one serves
                # as a reference and the longer one is matched to it.
                if len(odom) >= len(odom_or_fix):
                    shorter = odom_or_fix
                    longer = odom

                else:
                    shorter = odom
                    longer = odom_or_fix
                
                shorter_t = shorter.time_arr().reshape(-1,1)
                longer_t = longer.time_arr().reshape(-1,1)

                tree = KDTree(longer_t)

                _,indices = tree.query(shorter_t, k=2)

                # Filter out matches, where there is a larger time interval between
                # the two points than MAX_CORRESP_DELAY.
                nearest_dt = np.fabs(longer_t[indices[:,0]] - shorter_t)
                near_corresp_indices = np.where(nearest_dt < MAX_CORRESP_DELAY * 1e+09)[0]
                
                shorter.positions = (np.array(shorter.positions)[near_corresp_indices]).tolist()
                shorter_t = shorter_t[near_corresp_indices]
                indices = indices[near_corresp_indices]

                # Interpolate in time from the two (time-wise) nearest points.
                correspondence = self.interpolate(longer,longer_t,shorter_t,indices)
                #correspondence = longer_list[indices[:,0]]

                # Always save odom to index 0 and odom_or_fix to index 1.
                if len(odom) >= len(odom_or_fix):
                    self.correspondences[odom_key][odom_or_fix_key][0] = correspondence
                    self.correspondences[odom_key][odom_or_fix_key][1] = shorter
                else:
                    self.correspondences[odom_key][odom_or_fix_key][0] = shorter
                    self.correspondences[odom_key][odom_or_fix_key][1] = correspondence

                # Also get subsequent distances, which are used as weights when matching
                # rotation between two odoms.
                dists_1 = self.get_subsequent_distances(correspondence)
                dists_2 = self.get_subsequent_distances(shorter)
                self.distances[odom_key][odom_or_fix_key] = (dists_1 + dists_2)/2

        return

    def interpolate(self,longer,longer_t,shorter_t,indices):
        """
            For a given point and another pair of points find an interpolation in time
            from the pair of points so that the point obtained by the interpolation
            matches time-wise with the given point.
        """
        p_1 = longer.positions_arr()[indices[:,0]]
        p_2 = longer.positions_arr()[indices[:,1]]

        cov_1 = longer.covs_arr(none_col_sub=0)[indices[:,0]]
        cov_2 = longer.covs_arr(none_col_sub=0)[indices[:,1]]

        t_1 = longer_t[indices[:,0]]
        t_2 = longer_t[indices[:,1]]

        t_a = t_2 - t_1
        t_b = shorter_t - t_1

        dt = t_b/t_a

        interp_p = p_1 + (p_2-p_1) * dt

        cov = cov_1 * (1-dt) + cov_2 * dt
        cov[cov == 0] = None

        interp_p_container = PositionsContainer()
        interp_p_container.positions = [Position(interp_p[i,0],interp_p[i,1],interp_p[i,2],shorter_t[i],cov[i,0],cov[i,1],cov[i,2]) for i in range(len(interp_p))]
        return interp_p_container

    def nearest_orthonormal(self, M):
        assert M.ndim == 2
        assert M.shape[0] == M.shape[1]
        U, s, V = np.linalg.svd(M, full_matrices=False)
        R = np.matmul(U, V)
        return R
        
    def get_transformation_matrix(self, x, y, omit_z = True, covs = None, rot_only = False, distances = None):
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

        # If we don't want to match in z-coordinate.
        if omit_z:
            x[2,:] = 0
            y[2,:] = 0
            
        x_centered = x - x_mean
        y_centered = y - y_mean

        # Avoid loop through individual vectors.
        # M = x_centered @ y_centered.T

        # Used if no translation is wanted (usually with two odoms - we put both
        # beginnings to zero).
        if rot_only:            
            w = np.cumsum(distances)
            zero_indices = np.where(w == 0)[0]
            w[zero_indices] = 0.000001
            w = np.reciprocal(w)
            w = np.minimum(w,np.ones(w.shape))
            W = np.diag(w)
            M = np.matmul(x, W)
            M = np.matmul(M, y.T)

        else:
            if not covs is None and covs.size == 0 and self.use_weights:
                w = self.cov_to_weights(covs)
                W = np.diag(w)
                M = np.matmul(x_centered, W)
                M = np.matmul(M, y_centered.T)
            else:
                M = np.matmul(x_centered, y_centered.T)

        R = self.nearest_orthonormal(M).T

        if d == 3 and not np.isclose(np.fabs(np.linalg.det(R)), 1.0):
            print(R)
            print(np.linalg.det(R))
            raise ValueError("Rotation R, R'*R = I, det(R) = 1, could not be found.")

        if rot_only:
            t = 0 * y_mean
        else:
            t = y_mean - np.matmul(R, x_mean)

        T[:-1, :-1] = R
        T[:-1, -1:] = t

        return T
    
    def cov_to_weights(self,covs):
        """ 
            Transforms a 1D or 2D array of covariances of points
            to a 1D array of weights. Weights signify how reliable
            is the measurement at the given point.

            Each weight is calculated as 1/(mean of covs).
        """
        # Ensure no 0 covs.
        covs[covs == None] = 1
        covs[covs == 0] = 1

        # 1D case.
        if covs.ndim == 1 or (covs.ndim == 2 and covs.shape[1] == 1):
            return np.reciprocal(covs)
        
        # 2D case.
        w = np.mean(covs,axis=1)
        w[w == 0] = 1   # Ensure no 0s before reciprocal.
        w = np.reciprocal(w)
        
        return w

    def match_points(self):
        """ For pairs of arrays of corresponding points find transformation minimising lsq error
        and transform the arrays. """
        self.matched_points = {}
        for odom_topic in self.correspondences.keys():
            self.matched_points[odom_topic] = {}
            for correspondence_topic in self.correspondences[odom_topic].keys():
                both_are_odoms = odom_topic in self.odom_topics and correspondence_topic in self.odom_topics
                
                matched = self.correspondences[odom_topic][correspondence_topic]
                a = matched[0].positions_arr().T
                b = matched[1].positions_arr().T
                
                covs_combined = np.mean((matched[0].covs_arr(none_col_sub=0),matched[1].covs_arr(none_col_sub=0)),axis=0)
                covs_combined[covs_combined==0] = None
                covs = np.concatenate((matched[0].covs_arr(none_col_sub=False),matched[1].covs_arr(none_col_sub=False)),axis=1)
                covs = covs.astype('float')
                
                
                if not both_are_odoms:
                    if self.match_z:
                        T = self.get_transformation_matrix(np.copy(a),np.copy(b),False,covs,False,None)
                    else:
                        T = self.get_transformation_matrix(np.copy(a),np.copy(b),True,covs,False,None)

                    a_transformed = (T[:-1, :-1] @ a + T[:-1, -1:])     # Apply the transformation.
                    self.matched_points[odom_topic][correspondence_topic] = [a_transformed.T,b.T,covs_combined]   

                else:
                    a_translated = a - a[:,0].reshape(-1,1)
                    b_translated = b - b[:,0].reshape(-1,1)
                    distances = self.distances[odom_topic][correspondence_topic]
                    T = self.get_transformation_matrix(np.copy(a_translated),np.copy(b_translated),False,None,True,distances)

                    a_transformed = (T[:-1, :-1] @ a_translated + T[:-1, -1:])     # Apply the transformation.
                    self.matched_points[odom_topic][correspondence_topic] = [a_transformed.T,b_translated.T,covs_combined]

        return

    def visualise_centered_original(self):
        """ Visualise data before rotation matching. """
        fix_positions = np.array([[pos.position.x,pos.position.y] for pos in self.fixes])
        fix_center = np.mean(fix_positions,axis=0)
        plt.scatter(fix_positions[:,0],fix_positions[:,1])

        for odom in self.odometries:
            odom_positions = np.array([[pos.position.x,pos.position.y] for pos in self.correspondences[odom][1]])
            odom_positions += fix_center
            plt.scatter(odom_positions[:,0],odom_positions[:,1])

        plt.show()
    
    def visualise_3d(self):
        def set_axes_equal(ax):
            '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
            cubes as cubes, etc..  This is one possible solution to Matplotlib's
            ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

            Input
            ax: a matplotlib axis, e.g., as output from plt.gca().
            '''

            x_limits = ax.get_xlim3d()
            y_limits = ax.get_ylim3d()
            z_limits = ax.get_zlim3d()

            x_range = abs(x_limits[1] - x_limits[0])
            x_middle = np.mean(x_limits)
            y_range = abs(y_limits[1] - y_limits[0])
            y_middle = np.mean(y_limits)
            z_range = abs(z_limits[1] - z_limits[0])
            z_middle = np.mean(z_limits)

            # The plot bounding box is a sphere in the sense of the infinity
            # norm, hence I call half the max range the plot radius.
            plot_radius = 0.5*max([x_range, y_range, z_range])

            ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
            ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
            ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        fix_positions = np.array([[pos.position.x,pos.position.y,pos.position.z] for pos in self.fixes])
        fix_center = np.mean(fix_positions,axis=0)
        ax.scatter(fix_positions[:,0],fix_positions[:,1],fix_positions[:,2])

        for odom in self.odometries:
            odom_positions = np.array([[pos.position.x,pos.position.y,pos.position.z] for pos in self.correspondences[odom][1]])
            odom_positions += fix_center
            ax.scatter(odom_positions[:,0],odom_positions[:,1],odom_positions[:,2])

        set_axes_equal(ax)
        plt.show()

    def dist(self,a,b):
        """ Returns distances between pairs of points. """
        diff = a - b
        squared = np.square(diff)
        summed = np.sum(squared,axis=1)
        sqrt = np.sqrt(summed)
        return sqrt
    
    def md(self,a,b,w):
        """ MD is the mean distance between two arrays of pairs of corresponding points. """
        if a.ndim > 1:
            dist = self.dist(a,b)
        else:
            dist = np.fabs(a-b)
        if self.use_weights:
            dist = dist * w/np.sum(w) * len(dist)
        md = np.mean(dist)
        return md

    def get_coord_mds(self,a,b,w):
        """ Obtain mean distance in each single dimension. """
        x_md = self.md(a[:,0],b[:,0],w)
        y_md = self.md(a[:,1],b[:,1],w)
        z_md = self.md(a[:,2],b[:,2],w)
        return [x_md,y_md,z_md]

    def get_margin(self,min_long,max_long,min_lat,max_lat):
        """ Margin of background map picture. """
        y_margin = (max_lat-min_lat) * 0.1
        x_margin = (max_long-min_long) * 0.1

        if y_margin < x_margin:
            y_margin = x_margin
        else:
            x_margin = y_margin
        
        return x_margin, y_margin

    def plot_background_map(self,ax,min_long,max_long,min_lat,max_lat,for_animation=False):
        """ Obtain background map for some wgs points. """
        x_margin, y_margin = self.get_margin(min_long,max_long,min_lat,max_lat)

        if self.bg_image is None:
            background_map = get_background_image(min_long, max_long, min_lat, max_lat, x_margin, y_margin)
            self.bg_image = background_map            
        
        if for_animation and self.bg_image_animation is None:
            new_width = 800
            new_height = int(new_width/self.bg_image.size[0] * self.bg_image.size[1])
            self.bg_image_animation = self.bg_image.resize((new_width,new_height), Image.ANTIALIAS)
            
        min_utm_e,min_utm_n,_,_ = utm.from_latlon(min_lat - y_margin, min_long - x_margin)
        max_utm_e,max_utm_n,_,_ = utm.from_latlon(max_lat + y_margin, max_long + x_margin)

        im = self.bg_image_animation if for_animation else self.bg_image
        artist = ax.imshow(im, extent = [min_utm_e,\
                                    max_utm_e,\
                                    min_utm_n,\
                                    max_utm_n], alpha = 0.7, zorder = 0)

        ax.set_ylim([min_utm_n, max_utm_n])
        ax.set_xlim([min_utm_e, max_utm_e])

        if for_animation:
            return artist

        return [min_utm_n, max_utm_n],[min_utm_e, max_utm_e]

    def get_cmap(self,n, name='hsv'):
        return plt.cm.get_cmap(name, n)

    def init_animation(self):
        
        #self.artists.append(self.plot_background_map(self.ax_ani,self.min_long,self.max_long,self.min_lat,self.max_lat,ret_artist = True))
        #self.ax_ani.imshow(im,)

        for i in range(len(self.artists)):
            self.artists[i].set_data([], [])
            
        if self.fix_topic is None:
            return self.artists
        else:
            bg = self.plot_background_map(self.ax_ani,self.min_long,self.max_long,self.min_lat,self.max_lat,for_animation = True)
            return [bg] + self.artists

    def update_animation(self,i):
        for j in range(len(self.points_ani)):
            p = self.points_ani[j]
            num_points = np.max(p.shape)
            step = round(num_points/100)
            if step*i+step < num_points:
                #scat = self.ax_ani.scatter(p[0,10*i:10+10*i],p[1,10*i:10+10*i],c=self.colors[j],s=6)#label='_nolegend_'
                self.artists[j].set_data(p[0:step+step*i,0],p[0:step+step*i,1])
            
        if i == 0:
            self.legend = plt.legend(self.artists,self.legend)

        if i%9 == 0: 
            print("Animation rendering {}/100".format(i), end='\r')

        return self.artists + [self.legend]
    
    def visualise_animation(self):        
        self.points_ani = []
        self.legend = []
        self.artists = []

        self.cmap = self.get_cmap(len(self.odom_topics)+2)

        self.fig_ani, self.ax_ani = plt.subplots(figsize=(6,6), dpi=300)

        if self.fix_topic is not None:
            fix_points = self.fixes.positions_arr()
            trimmer = max([1,round(len(fix_points[:,0])/NUM_TRIMMED_POINTS)])
            fix_points = fix_points[::trimmer,:]

            self.points_ani.append(fix_points)
            self.legend.append('fix')
            self.artists += self.ax_ani.plot([],[],linestyle='None', color=self.cmap(0), markersize = 5,marker='x',zorder=20,alpha=0.8)

            for i,odom in enumerate(self.odom_topics):
                self.legend.append(odom)

                points = self.matched_points[odom][self.fix_topic][0]
                trimmer = max([1,round(len(points[:,0])/NUM_TRIMMED_POINTS)])
                points = points[::trimmer,:]

                self.points_ani.append(points)
                self.artists += self.ax_ani.plot([],[],linestyle='None', color=self.cmap(i+1), markersize = 5,marker='o',alpha=0.8)

        else:
            odom = self.odom_topics[0]
            self.legend.append(odom)

            x_min = np.inf
            x_max = -np.inf
            y_min = np.inf
            y_max = -np.inf

            for i,another_odom in enumerate(self.matched_points[odom].keys()):
                self.legend.append(another_odom)

                if i == 0:
                    points = self.matched_points[odom][another_odom][0]
                    trimmer = max([1,round(len(points[:,0])/NUM_TRIMMED_POINTS)])
                    points = points[::trimmer,:]

                    x_min = min(x_min,np.amin(points[:,0]))
                    x_max = max(x_max,np.amax(points[:,0]))
                    y_min = min(y_min,np.amin(points[:,1]))
                    y_max = max(y_max,np.amax(points[:,1]))

                    self.points_ani.append(points)
                    self.artists += self.ax_ani.plot([],[],linestyle='None', color=self.cmap(i), markersize = 5,marker='o',alpha=0.8)
                
                points = self.matched_points[odom][another_odom][1]
                trimmer = max([1,round(len(points[:,0])/NUM_TRIMMED_POINTS)])
                points = points[::trimmer,:]

                x_min = min(x_min,np.amin(points[:,0]))
                x_max = max(x_max,np.amax(points[:,0]))
                y_min = min(y_min,np.amin(points[:,1]))
                y_max = max(y_max,np.amax(points[:,1]))

                self.points_ani.append(points)
                self.artists += self.ax_ani.plot([],[],linestyle='None', color=self.cmap(i+1), markersize = 5,marker='o',alpha=0.8)
            
            self.ax_ani.set_xlim([x_min - 4,x_max + 4])
            self.ax_ani.set_ylim([y_min - 4,y_max + 4])

        plt.setp(self.ax_ani.get_xticklabels(), rotation=30, horizontalalignment='right')
        self.ax_ani.set_title("Visualisation of robot's movement\n via gps measurings and odometries", y=1.04)
        self.ax_ani.set_xlabel("easting [m]")
        self.ax_ani.set_ylabel("northing [m]")
        #plt.rcParams['axes.titlepad'] = 12
        ani = animation.FuncAnimation(self.fig_ani, self.update_animation, interval=50, frames=100, init_func=self.init_animation)#, blit=True)#,blit=True) #init_func=self.init_animation, blit=True
        
        ani.save(filename="{}.gif".format(self.fn), dpi=300. ,writer = animation.PillowWriter())
        #plt.savefig("{}.png".format(self.fn), dpi=300.)

    def visualise_matched(self):
        """ Visualise points of fix and odoms after their matching. """
        odom_colors_arr = ['orange', 'lawngreen', 'crimson', 'blue', 'gray'] # More than five odometries is not expected.
        odom_colors = {}
        for i,odom in enumerate(self.odom_topics):
            odom_colors[odom] = odom_colors_arr[i%len(odom_colors_arr)]

        for odom in self.odom_topics:
            for correspondence in self.matched_points[odom].keys(): 
                points1 = self.matched_points[odom][correspondence][0]
                points2 = self.matched_points[odom][correspondence][1]
                trimmer = max([1,round(len(points1[:,0])/NUM_TRIMMED_POINTS)])
                
                covs_with_none = self.matched_points[odom][correspondence][2]
                covs = covs_with_none[:,np.where(covs_with_none[0] != None)[0]]         # Get rid of None cols.
                use_covs = covs.size != 0

                if use_covs:
                    covs = covs.astype('float')
                    covs = np.mean(covs,axis=1)
                    trimmed_covs = covs[::trimmer]
                    trimmed_covs_with_none = covs_with_none[::trimmer,:]
                    weights = self.cov_to_weights(covs)
                    c = np.clip(trimmed_covs, MIN_COV, MAX_COV)
                    c = np.log(c)
                else:
                    c = np.ones((covs_with_none.shape[0],))[::trimmer]
                    weights = np.ones((covs_with_none.shape[0],))

                
                cmap = plt.cm.cool

                fig, ax = plt.subplots(figsize=(8,8), dpi=400)

                y_lim,x_lim = None,None

                if correspondence == self.fix_topic:
                    y_lim,x_lim = self.plot_background_map(ax,self.min_long,self.max_long,self.min_lat,self.max_lat)

                norm=mpl.colors.Normalize(vmin=np.log(MIN_COV), vmax=np.log(MAX_COV))

                x1 = points1[::trimmer,0]
                y1 = points1[::trimmer,1]
                z1 = points1[::trimmer,2]

                x2 = points2[::trimmer,0]
                y2 = points2[::trimmer,1]
                z2 = points2[::trimmer,2]
                
                ax.scatter(x1,y1,s=7+15,marker="o",c=odom_colors[odom],edgecolor='black',lw=0.3,zorder=20,)
                
                if correspondence == self.fix_topic:
                    ax.scatter(x2,y2,s=6+15,marker="x",c=c,cmap=cmap,lw=1,zorder=21,alpha=1, norm=norm)
                else:
                    ax.scatter(x2,y2,s=7+15,marker="o",c=odom_colors[correspondence],edgecolor='black',lw=0.3,zorder=21,alpha=0.8)                    

                if correspondence == self.fix_topic and use_covs:
                    trimmed_covs_with_none[trimmed_covs_with_none==None] = 0.01
                    for i in range(len(trimmed_covs_with_none)):
                        cov = trimmed_covs_with_none[i]
                        x = x2[i]
                        y = y2[i]

                        width = 2 * ELIPSE_SIGMA * math.sqrt(cov[0])
                        height = 2 * ELIPSE_SIGMA * math.sqrt(cov[1])

                        if self.switch_w_h:
                            old_w = np.copy(width)
                            width = height
                            height = old_w

                        ellipse = Ellipse((x,y),width=width, height=height, alpha=0.3,zorder=5,fill=False,ec=cmap(norm(c[i])), label='_nolegend_')
                        ax.add_patch(ellipse)

                        # Connect corresponding pairs of points:
                        x = [x1[i],x2[i]]
                        y = [y1[i],y2[i]]
                        ax.plot(x,y, lw=0.2,c='black',label='_nolegend_',zorder=3)

                coord_mds = self.get_coord_mds(points1,points2,weights)
                
                ax.set_title(
"Visualisation of matched points of topics\
\n {} and {} \n with 1 σ ellipses \
\n\n Mean distance between points: {} m\
\n Mean distance in x (longitude): {} m\
\n Mean distance in y (latitude): {} m\
\n Mean distance in z (altitude): {} m".format(odom,correspondence,      
                    round(self.md(points1,points2,weights),2),                                  
                    round(coord_mds[0],2),
                    round(coord_mds[1],2),
                    round(coord_mds[2],2),))
                
                """ ax.set_title(
"Visualisation of matched points of topics\
\n {} and {} \n with covariance ellipses".format(odom,correspondence,\
                    round(self.md(points1,points2,weights),2),                                  
                    round(coord_mds[0],2),
                    round(coord_mds[1],2),
                    round(coord_mds[2],2),)) """

                ax.legend([odom,correspondence])
                ax.axis('equal')
                ax.grid(zorder=0)

                plt.setp(ax.get_xticklabels(), visible=False)
                plt.setp(ax.get_yticklabels(), visible=False) 
                plt.setp(ax.get_yaxis().get_offset_text(), visible=False)

                divider = make_axes_locatable(ax)
                cax = divider.append_axes("right", size="5%", pad=0.05)

                ax_b = divider.append_axes("bottom", size="20%", pad=0.5, sharex = ax)
                ax_l = divider.append_axes("left", size="20%", pad=0.5, sharey = ax)

                ax_b.scatter(x1,z1,s=7+15,marker="o",c=odom_colors[odom],edgecolor='black',lw=0.3,zorder=20)        
                              
                ax_b.set_ylabel("z [m]")
                ax_l.scatter(z1,y1,s=7+15,marker="o",c=odom_colors[odom],edgecolor='black',lw=0.3,zorder=20)        
                  
                ax_l.set_xlabel("z [m]")

                if correspondence == self.fix_topic:
                    ax_b.scatter(x2,z2,s=6+15,marker="x",c=c,cmap=cmap,lw=1,zorder=21,alpha=0.8,norm=norm)
                    ax_l.scatter(z2,y2,s=6+15,marker="x",c=c,cmap=cmap,lw=1,zorder=21,alpha=0.8,norm=norm)
                else:
                    ax_b.scatter(x2,z2,s=7+15,marker="o",c=odom_colors[correspondence],edgecolor='black',lw=0.3,zorder=21)
                    ax_l.scatter(z2,y2,s=7+15,marker="o",c=odom_colors[correspondence],edgecolor='black',lw=0.3,zorder=21)


                plt.setp(ax_b.get_xticklabels(), rotation=30, horizontalalignment='right')

                ax_b.set_xlabel("easting [m]")
                ax_l.set_ylabel("northing [m]")

                ax_b.grid(zorder=0)
                ax_l.grid(zorder=0)

                if x_lim and y_lim:
                    ax_b.set_xlim(x_lim)
                    ax_l.set_ylim(y_lim)

                lognorm = mpl.colors.LogNorm(MIN_COV,MAX_COV)
                fig.colorbar(mpl.cm.ScalarMappable(norm=lognorm, cmap=plt.cm.cool),cax=cax, orientation='vertical', label='Covariance [$m^2$]')  
                fig_fn ="{}-{}-{}.png".format(self.fn,odom.replace('/', '_'),correspondence.replace('/', '_'))
                plt.savefig(fig_fn,dpi=800,bbox_inches='tight')
                print("Figure saved to {}.".format(fig_fn))
    
    def visualise_single(self):
        points = self.odometries[self.odom_topics[0]].positions_arr()
        trimmer = max([1,round(len(points[:,0])/NUM_TRIMMED_POINTS)])

        fig, ax = plt.subplots(figsize=(8,8), dpi=400)

        x1 = points[::trimmer,0]
        y1 = points[::trimmer,1]
        z1 = points[::trimmer,2]

        ax.scatter(x1,y1,s=7+15,marker="o",c='red',edgecolor='black',lw=0.3,zorder=20,)

        ax.set_title("Visualisation of odometry topic {}".format(self.odom_topics[0]))
        ax.axis('equal')
        ax.grid(zorder=0)

        plt.setp(ax.get_xticklabels(), rotation=30, horizontalalignment='right')
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        
        fig_fn ="{}.png".format(self.fn)
        plt.savefig(fig_fn,dpi=800,bbox_inches='tight')
        print("Figure saved to {}.".format(fig_fn))

    def run(self):
        print("Parsing topics.")
        """ try:
            self.parse_topics()
        except Exception as e:
            print(f"Error: {e}") """
        self.parse_topics()

        if len(self.odom_topics)==0 and self.fix_topic is None:
            print("No fix and no odometries.")
            return
        
        if len(self.odom_topics)==1 and self.fix_topic is None:
            print("No fix and only one odometry.")
            self.visualise_single()
            return

        print("Matching points.")
        self.find_correspondences()
        self.match_points()
        print("Creating figures.")
        self.visualise_matched()
        if self.produce_animation:
            self.visualise_animation()
        return

    def __del__(self):
        self.bag.close()

if __name__ == "__main__":
    bags = {
    'tradr':["/home/robot/gps_visualise/0630_bags/ugv_2022-06-30-11-30-57.no_sensors.bag",
    "/home/robot/gps_visualise/0630_bags/ugv_2022-06-30-11-47-31.no_sensors.bag",
    "/home/robot/gps_visualise/0630_bags/ugv_2022-06-30-12-05-14.no_sensors.bag",
    "/home/robot/gps_visualise/0630_bags/ugv_2022-06-30-14-23-28.no_sensors.bag"],

    'spot': ["/home/robot/gps_visualise/0630_bags/spot_2022-06-30-11-39-13.no_sensors.bag",
    "/home/robot/gps_visualise/0630_bags/spot_2022-06-30-13-10-36.no_sensors.bag"],

    'husky': ["/home/robot/gps_visualise/0630_bags/husky_2022-06-30-11-52-16.no_sensors.bag",
    "/home/robot/gps_visualise/0630_bags/husky_2022-06-30-12-20-08.no_sensors.bag",
    "/home/robot/gps_visualise/0630_bags/husky_2022-06-30-15-08-50.no_sensors.bag",
    "/home/robot/gps_visualise/0630_bags/husky_2022-06-30-15-11-49.no_sensors.bag",
    "/home/robot/gps_visualise/0630_bags/husky_2022-06-30-15-58-37.no_sensors.bag"]
    }

    odom_topics = {
        'tradr':["/icp_odom","/imu_odom"],
        'spot': ["/icp_odom","/spot/odometry"],
        'husky': ["/icp_odom","/imu_and_wheel_odom"]
    }

    for robot in bags.keys():
        for i,bag in enumerate(bags[robot]):
            bag = OdomMatcher(bag, bag[:-15], odom_topics[robot], use_weights=True, fix_topic = '/fix', switch_w_h = True, use_odom_covs=False, produce_animation=True, match_z = False)
            bag.run()


