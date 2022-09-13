import csv
import matplotlib.pyplot as plt
import numpy as np
import sys
import re
import time
import datetime
import requests
from os import path
from PIL import Image
import geopy.distance
import utm

URL_PREF = "https://maps.geoapify.com/v1/staticmap?style=osm-carto"
API_KEY = "8f3be3c0c8484eceb15b0f50218c8c02"

    
class BackgroundMapPlotter:
    def get_url(self,w,h,corners):
        url = URL_PREF
        w_url = f'&width={w}'
        h_url = f'&height={h}'
        area_url = f'&area=rect:{corners[0]},{corners[1]},{corners[2]},{corners[3]}'
        api_url = f'&apiKey={API_KEY}'
        
        url = url + w_url + h_url + area_url + api_url

        return url

    def get_margin(self,min_long,max_long,min_lat,max_lat):
        y_margin = (max_lat-min_lat) * 0.1
        x_margin = (max_long-min_long) * 0.1

        if y_margin < x_margin:
            y_margin = x_margin
        else:
            x_margin = y_margin
        
        return x_margin, y_margin

    def get_background_image(self,min_long,max_long,min_lat,max_lat, x_margin, y_margin):
        

        width_m = geopy.distance.geodesic((max_lat + y_margin,max_long + x_margin),(min_lat - y_margin,max_long + x_margin))
        height_m = geopy.distance.geodesic((max_lat + y_margin,max_long + x_margin),(max_lat + y_margin,min_long - x_margin))
        ratio = width_m / height_m
        
        size_limit = min(4000,150*min(width_m.meters,height_m.meters))
        width = int(size_limit)
        height = int(width * ratio)

        while width > size_limit or height > size_limit:    # anything more can lead to wrongly cut background image (maybe only with large ratios)
            width = int(width*0.9)
            height = int(width * ratio)

        corners = [min_long - x_margin, max_lat + y_margin, max_long + x_margin, min_lat - y_margin]
        url = self.get_url(width, height, corners)
        #print(url)

        bg_map = Image.open(requests.get(url, stream=True).raw)

        return bg_map

    def plot_background_map(self,ax,min_long,max_long,min_lat,max_lat):

        x_margin, y_margin = self.get_margin(min_long,max_long,min_lat,max_lat)
        background_map = self.get_background_image(min_long, max_long, min_lat, max_lat, x_margin, y_margin)

        min_utm = utm.from_latlon(min_lat - y_margin, min_long - x_margin)
        max_utm = utm.from_latlon(max_lat + y_margin, max_long + x_margin)
        ax.imshow(background_map, extent = [min_utm[0],\
                                    max_utm[0],\
                                    min_utm[1],\
                                    max_utm[1]], alpha = 1, zorder = 0)

        ax.set_ylim([min_utm[1], max_utm[1]])
        ax.set_xlim([min_utm[0], max_utm[0]])