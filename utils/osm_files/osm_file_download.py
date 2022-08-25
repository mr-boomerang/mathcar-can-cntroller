#!/usr/bin/env python
# Usage: Enter the latitude and the longitude values of the map which is intended to be downloaded, then a set of map files will be downloaded covering the nearby region of that coordinates.

# Example: 
# enter the latitude value of the map file up to the first decimal
#	17.432
# enter the longiitude value of the map file up to the first decimal
#	78.564

# Map files from 17.40.osm to 17.49.osm will be downloaded in the folder 78.5/17/          


import urllib
import os
import math

lat_offset = 0.0100
print("enter the latitude value of the map file up to the first decimal")
lat_from = input()
print("enter the longiitude value of the map file up to the first decimal")
lon_from = input()

file_path = "/home/chaitanya/catkin_ws2/src/utils/osm_files/" + str(math.floor(lon_from * 10)/10) + "/" + str(int(lat_from)) + "/" + str(math.floor(lat_from * 100)/100) + ".osm"
directory = os.path.dirname(file_path)

dir_flag = os.path.exists(directory)
file_flag = os.path.isfile(file_path)

lat_from = (math.floor(lat_from * 100)/100) + 0.00
lat_till = (math.floor(lat_from * 10)/10) + 0.09

if not os.path.exists(directory):
    os.makedirs(directory)
    while (lat_from <= lat_till):
          file_path = "/home/chaitanya/catkin_ws2/src/utils/osm_files/" + str(math.floor(lon_from * 10)/10) + "/" + str(int(lat_from)) + "/" + str(math.floor(lat_from * 100)/100) + ".osm"
          min_lat = (math.floor(lat_from * 100)/100) + 0.0000
          max_lat = (math.floor(lat_from * 100)/100) + 0.0999
          min_lon = (math.floor(lon_from * 10)/10) + 0.0000
          max_lon = (math.floor(lon_from * 10)/10) + 0.0999
          testfile = urllib.URLopener() 
          testfile.retrieve("http://api.openstreetmap.org/api/0.6/map?bbox=" + str(min_lon) + "," + str(min_lat) + "," + str(max_lon) + "," + str(max_lat) + "", file_path)
          lat_from = lat_from + lat_offset

else:
    while (lat_from <= lat_till):
          file_path = "/home/chaitanya/catkin_ws2/src/utils/osm_files/" + str(math.floor(lon_from * 10)/10) + "/" + str(int(lat_from)) + "/" + str(math.floor(lat_from * 100)/100) + ".osm"
          min_lat = (math.floor(lat_from * 100)/100) + 0.0000
          print(min_lat)
          max_lat = (math.floor(lat_from * 100)/100) + 0.0999
          min_lon = (math.floor(lon_from * 10)/10) + 0.0000
          max_lon = (math.floor(lon_from * 10)/10) + 0.0999
          testfile = urllib.URLopener() 
          testfile.retrieve("http://api.openstreetmap.org/api/0.6/map?bbox=" + str(min_lon) + "," + str(min_lat) + "," + str(max_lon) + "," + str(max_lat) + "", file_path)
          lat_from = lat_from + lat_offset          
