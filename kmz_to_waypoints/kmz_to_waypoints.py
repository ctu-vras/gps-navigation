from audioop import mul
import zipfile
from fastkml import kml # https://fastkml.readthedocs.io/en/latest/usage_guide.html#read-a-kml-file-string
import shapely
import matplotlib.pyplot as plt
import numpy as np
import utm
from scipy.spatial import KDTree
from numpy.lib.recfunctions import structured_to_unstructured
from matplotlib.lines import Line2D
import matplotlib.patches as patches
from matplotlib.widgets import Button
import math
import gpxpy
from copy import copy
import os
import xml.etree.ElementTree as ET

from background_map import *

#MIN_DIST_WAYPOINTS = 1 # meters

""" class ButtonHandle:

    def save_path(self,event):
        fn = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gpx/path.gpx")
        c = 1
        while os.path.isfile(fn):
            fn = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gpx/path({}).gpx".format(c))
            c += 1

        gpx = gpxpy.gpx.GPX()

        gpx_track = gpxpy.gpx.GPXTrack()
        gpx.tracks.append(gpx_track)

        gpx_segment = gpxpy.gpx.GPXTrackSegment()
        gpx_track.segments.append(gpx_segment)

        for point in self.chosen_points:
            gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(point[0], point[1], elevation=0))

        xml = gpx.to_xml()
        with open(fn, "w") as f:
            f.write(xml)
            f.close()
        
        print("Path saved to {}.".format(fn))


    def undo(self,event):
        pass

    def reset(self,event):
        pass """

plt.ion()

class KmzParser:
    def __init__(self,kml_fn,kmz_fn=None):
        self.kml_fn = kml_fn
        self.kmz_fn = kmz_fn
        self.chosen_points = np.array([]).reshape(0,2)
        self.chosen_point_archive = [self.chosen_points]
        self.chosen_fills = []
        self.chosen_squares = []

        self.min_long = 10000 
        self.max_long = -10000 
        self.min_lat = 10000
        self.max_lat = -10000

        self.FLIPS = [[0,0],[0,1],[1,0],[1,1]]
        self.FILL_COLORS = ['#2DB421','#21B494','#219CB4']

        self.zone_number = 33
        self.zone_letter = 'U'

    def open_kmz(self):
        if not self.kmz_fn is None:
            self.kmz = zipfile.ZipFile(self.kmz_fn, 'r')
            self.kml_data = self.kmz.read(self.kml_fn)
        else:
            self.kmz = None
            self.kml_data = ET.parse(self.kml_fn)
            self.kml_data = self.kml_data.getroot()
            self.kml_data = ET.tostring(self.kml_data, encoding="utf-8", method = 'xml')
            #with open(self.kml_fn, 'rt', encoding="utf-8") as f:
            #    self.kml_data = f.read()
    
    def extract_elements_from_kml(self):
        self.kml = kml.KML()
        try:
            self.kml.from_string(self.kml_data)
        except:
            self.kml.from_string(self.kml_data)

        self.data = list(self.kml.features())
        self.data = list(self.data[0].features())
        self.data = list(self.data[0].features())

    def update_wgs_extremes(self, points):
        if np.amax(points[1,:]) > self.max_lat:
            self.max_lat = np.amax(points[1,:])
        
        if np.amax(points[0,:]) > self.max_long:
            self.max_long = np.amax(points[0,:])
        
        if np.amin(points[1,:]) < self.min_lat:
            self.min_lat = np.amin(points[1,:])
        
        if np.amin(points[0,:]) < self.min_long:
            self.min_long = np.amin(points[0,:])

    def process_data(self):
        self.points_poly = []
        for i in range(len(self.data)):
            element = self.data[i]
            if element.geometry.geom_type == "LineString":

                #self.points += list(element.geometry.coords)

                points = np.array(element.geometry.xy)
                self.update_wgs_extremes(points)
                points = utm.from_latlon(points[1,:],points[0,:])
                self.zone_number = points[2]
                self.zone_letter = points[3]
                points = np.concatenate([points[0].reshape(-1,1),points[1].reshape(-1,1)],axis=1)
                line = shapely.geometry.LineString(points)
                poly = line.buffer(3, resolution=1)
                self.points_poly.append([points,poly])

            if element.geometry.geom_type == "MultiLineString":
                points = np.array([]).reshape(2,0)
                for line in list(element.geometry.geoms):
                    if points.size:
                        if points[0,-1] == line.coords.xy[0][0] and points[1,-1] == line.coords.xy[1][0]:
                            points = np.concatenate((points[:,:-1],line.coords.xy),axis=1)
                    else:
                        points = np.concatenate((points,line.coords.xy),axis=1)

                self.update_wgs_extremes(points)
                points = utm.from_latlon(points[1,:],points[0,:])
                points = np.concatenate([points[0].reshape(-1,1),points[1].reshape(-1,1)],axis=1)
                line = shapely.geometry.LineString(points)
                poly = line.buffer(3)
                self.points_poly.append([points,poly])

        #self.points = np.array(self.points)
        #self.points = np.unique(self.points, axis=0)
        #self.points = utm.from_latlon(self.points[:,1], self.points[:,0])
        #self.points = np.concatenate([self.points[0].reshape(-1,1),self.points[1].reshape(-1,1)],axis=1)

    def visualize(self):
        fig,self.ax = plt.subplots()

        #button_handle = ButtonHandle()

        ax_save = plt.axes([0.75, 0.15, 0.10, 0.05])
        b_save = Button(ax_save, 'Save')
        b_save.on_clicked(self.button_save_path)

        ax_undo= plt.axes([0.75, 0.21, 0.10, 0.05])
        b_undo = Button(ax_undo, 'Undo')
        b_undo.on_clicked(self.button_undo)

        ax_reset = plt.axes([0.75, 0.27, 0.10, 0.05])
        b_reset = Button(ax_reset, 'Reset')
        b_reset.on_clicked(self.button_reset)

        self.artists = [None] * len(self.points_poly)
        #ax.plot(self.points[:,0],self.points[:,1],'o')

        for i in range(len(self.points_poly)):
            #ax.plot(line[:,0],line[:,1],lw=5, picker=True)
            poly = self.points_poly[i][1]
            artist = None

            fill_color = self.FILL_COLORS[i%3]

            if type(poly) == shapely.geometry.MultiPolygon:
                for p in poly.geoms:
                    x,y = p.exterior.xy
                    #ax.plot(x, y, c='#fdbf65', linewidth=5, zorder = 3)
                    artist, = self.ax.fill(x,y,c=fill_color, alpha=0.4, zorder = 2, picker=True)

            elif type(poly) == shapely.geometry.Polygon:
                x,y = poly.exterior.xy
                #ax.plot(x, y, c='#fdbf65', linewidth=5, zorder = 3)
                artist, = self.ax.fill(x,y,c=fill_color, alpha=0.4, zorder = 2, picker=True)

            self.artists[i] = artist

            print("{}/{}".format(i,len(self.points_poly)))
            #if i >100:
            #    break

        fig.canvas.mpl_connect('pick_event', self.onpick)

        bg_plotter = BackgroundMapPlotter()
        bg_plotter.plot_background_map(self.ax, self.min_long,self.max_long,self.min_lat,self.max_lat)

        plt.show(block=True)
    
    def point_picker(self, line, mouseevent):
        if mouseevent.xdata is None:
            return False, dict()
        xdata = line.get_xdata().reshape(-1,1)
        ydata = line.get_ydata().reshape(-1,1)
        tree = KDTree(np.concatenate([xdata,ydata],axis=1))
        nearest_point_ind = tree.query([mouseevent.xdata, mouseevent.ydata],distance_upper_bound=10)[1]

        if nearest_point_ind < len(xdata):
            pickx = np.take(xdata, nearest_point_ind)
            picky = np.take(ydata, nearest_point_ind)
            props = dict(pickx=pickx, picky=picky)
            return True, props
        else:
            return False, dict()

    def dist2(self,p1,p2):
        return np.sum((p1-p2)**2) 

    def squarify(self, points):
        squarish = [None] * len(points)
        for i,p in enumerate(points):
            x = p[0]
            y = p[1]
            p_squarish = shapely.geometry.Polygon([[x+1,y+1],[x-1,y+1],[x-1,y-1],[x+1,y-1]]) 
            squarish[i] = p_squarish
        return np.array(squarish)       

    def onpick(self,event):
        if isinstance(event.artist, patches.Polygon):
            points_to_add = None
            for i in range(len(self.artists)):
                if self.artists[i] == event.artist:
                    points_to_add = self.points_poly[i][0]
                    print(points_to_add)

                    if self.chosen_points.size:

                        flip_choices = [self.dist2(self.chosen_points[-1,:],points_to_add[0,:]),
                                self.dist2(self.chosen_points[-1,:],points_to_add[-1,:]),
                                self.dist2(self.chosen_points[0,:],points_to_add[0,:]),
                                self.dist2(self.chosen_points[0,:],points_to_add[-1,:]),]

                        flip_choice = np.argmin(flip_choices)

                        flip_choice = self.FLIPS[flip_choice]

                        if flip_choice[0]:
                            self.chosen_points = np.flip(self.chosen_points,axis=0)
                        if flip_choice[1]:
                            points_to_add = np.flip(points_to_add,axis=0)

                        if math.sqrt(self.dist2(points_to_add[0,:], self.chosen_points[-1,:])) <= 0.1:  # Remove one of the duplicate points between the arrays we want to concat.
                            points_to_add = points_to_add[1:,:]

                    self.chosen_points = np.concatenate([self.chosen_points,points_to_add],axis=0) 
                    self.chosen_point_archive.append(self.chosen_points)
                    
                    #self.artists[i].remove()
                    #points_to_add = None
                    #plt.draw()
                    
                    break
            if points_to_add is not None:
                x = event.artist.xy[:,0]
                y = event.artist.xy[:,1]
                art = self.ax.fill(x,y,c='#aaaaaa', alpha=0.7, zorder = 2, picker=True)
                self.chosen_fills.append(art)

                point_polys = self.squarify(points_to_add)
                t = np.arange(0,len(points_to_add))
                color = np.divide(t,len(t)).astype(float)

                chosen_squares_art = []
                for i,point_poly in enumerate(point_polys):
                    x,y = point_poly.exterior.xy
                    art = self.ax.fill(x,y,color = [color[i],0,color[i]], zorder = 10)
                    chosen_squares_art.append(art)
                    #plt.draw()
                
                self.chosen_squares.append(chosen_squares_art)
                """ line = event.artist
                xdata = line.get_xdata()
                ydata = line.get_ydata()
                ind = event.ind
                print('onpick1 line:', zip(np.take(xdata, ind), np.take(ydata, ind)))
                #self.chosen_points.append([x,y]) """
    
    def button_save_path(self,event):
        fn = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gpx/path.gpx")
        c = 1
        while os.path.isfile(fn):
            fn = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gpx/path({}).gpx".format(c))
            c += 1

        gpx = gpxpy.gpx.GPX()

        #gpx_track = gpxpy.gpx.GPXTrack()
        #gpx.tracks.append(gpx_track)

        #gpx_segment = gpxpy.gpx.GPXTrackSegment()
        #gpx_track.segments.append(gpx_segment)

        points_to_save = copy(self.chosen_points)
        points_to_save = utm.to_latlon(points_to_save[:,0],points_to_save[:,1],self.zone_number,self.zone_letter)
        points_to_save = np.array(points_to_save).T

        for point in points_to_save:
            gpx.waypoints.append(gpxpy.gpx.GPXWaypoint(point[0], point[1]))

        xml = gpx.to_xml()
        with open(fn, "w") as f:
            f.write(xml)
            f.close()

        #points_to_save = copy(self.chosen_points)
        #points_to_save = utm.to_latlon(points_to_save[:,0],points_to_save[:,1],self.zone_number,self.zone_letter)
        #points_to_save = np.array(points_to_save).T
        #for point in points_to_save:
        #    gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(point[0], point[1], elevation=0))

        #xml = gpx.to_xml()
        #with open(fn, "w") as f:
        #    f.write(xml)
        #    f.close()
        
        print("Path saved to {}.".format(fn))

        fig,ax = plt.subplots()
        x = self.chosen_points[:,0]
        y = self.chosen_points[:,1]
        t = np.arange(0,len(x))
        color = np.divide(t,len(t)).astype(float)
        ax.quiver(x[:-1], y[:-1], x[1:]-x[:-1], y[1:]-y[:-1], color, scale_units='xy', angles='xy', scale=1)


    def button_undo(self,event):
        for art in self.chosen_squares.pop(-1):
            art[0].remove()

        art = self.chosen_fills.pop(-1)
        art[0].remove()

        self.chosen_point_archive.pop(-1)
        self.chosen_points = self.chosen_point_archive[-1]

        print("Undo")

    def button_reset(self,event):
        self.chosen_points = np.array([]).reshape(0,2)
        self.chosen_point_archive = [self.chosen_points]
        for art in self.chosen_fills:
            art[0].remove()
        for arts in self.chosen_squares:
            for art in arts:
                art[0].remove()
        self.chosen_fills = []
        self.chosen_squares = []
        #plt.draw()
        print("Path reset")

    def run(self):
        self.open_kmz()
        self.extract_elements_from_kml()
        self.process_data()
        self.visualize()

if __name__ == "__main__":
    #kml_fn = "/home/robot/unhost.kml"
    #kmz_fn = None
    kmz_fn = "/home/robot/Downloads/001_p.kmz"
    kml_fn = "doc.kml"
    parser = KmzParser(kml_fn = kml_fn, kmz_fn = kmz_fn)
    parser.run()

