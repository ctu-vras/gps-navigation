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

URL_PREF = "https://maps.geoapify.com/v1/staticmap?style=osm-carto"
API_KEY = "8f3be3c0c8484eceb15b0f50218c8c02"

    
def get_url(w,h,corners):
    url = URL_PREF
    w_url = f'&width={w}'
    h_url = f'&height={h}'
    area_url = f'&area=rect:{corners[0]},{corners[1]},{corners[2]},{corners[3]}'
    api_url = f'&apiKey={API_KEY}'
    
    url = url + w_url + h_url + area_url + api_url

    return url

def get_background_image(min_long,max_long,min_lat,max_lat, x_margin, y_margin):
    

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
    url = get_url(width, height, corners)
    #print(url)

    bg_map = Image.open(requests.get(url, stream=True).raw)

    return bg_map