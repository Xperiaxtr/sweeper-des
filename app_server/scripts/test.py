#!/usr/bin/env python
#coding=utf8
import requests
import json
import math
from pyproj import Proj
import conv_WGS84 as conv
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from sweeper_msgs.msg import StateReport
from sweeper_msgs.msg import SweeperChassisDetail
import threading
import time
import tf


def transform_utm_into_lat_lon(x, y, zone, hemisphere):
 
    # verify the hemisphere
    h_north = False
    h_south = False
    if (hemisphere == 'N'):
        h_north = True
    elif (hemisphere == 'S'):
        h_south = True
    else:
        print("Unknown hemisphere: " + hemisphere)
 
    proj_in = Proj(proj = 'utm', zone = zone, ellps = 'WGS84', south = h_south, north = h_north, errcheck = True)
 
    lon, lat = proj_in(x, y, inverse = True)
 
    # just printing the floating point number with 6 decimal points will round it
    lon = math.floor(lon * 100000000) / 100000000
    lat = math.floor(lat * 100000000) / 100000000
 
    lon,lat = conv.wgs_gcj(lon,lat)

    lon = "%.8f" % lon
    lat = "%.8f" % lat
    return lon, lat

class ros_node:
    def __init__(self):
        self.lat = 0.0
        self.lon = 0.0
        self.speed =0.0
        self.mode = 'human'
        self.battery = 100
        self.voltage = 50
        self.heading = 0

        rospy.init_node('client_cloud', anonymous=True)
        rospy.Subscriber('/sweeper/gps/fix',NavSatFix,self.callback)
        rospy.Subscriber('/sweeper/chassis/detail',SweeperChassisDetail,self.callback_speed)
        rospy.Subscriber('/sweeper/sweep/state_report',StateReport,self.callback_report)
        rospy.Subscriber('/sweeper/sensor/gnss',Odometry,self.callback_gnss)

        self.rev_socket = threading.Thread(target=self.send)
        self.rev_socket.setDaemon(True)
        self.rev_socket.start()

        rospy.spin()

    def callback(self,data):
        self.lat, self.lon = conv.wgs_gcj(data.latitude,data.longitude)

    def callback_gnss(self,data):
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.w
        z = data.pose.pose.orientation.w
        w = data.pose.pose.orientation.w
        angel = tf.transformations.euler_from_quaternion([x,y,z,w])
        self.heading = angel[2]
        
    def callback_speed(self,data):
        self.speed = data.vehicle_speed_output
        self.battery = data.chassis_vehicle_soc

    def callback_report(self,data):
        dic_state =[]
        for code in data.result_code:
            if code == 4000:
                self.mode = 'auto'
            if code == 4004:
                self.mode = 'human'
            
    def send(self):
        while not rospy.is_shutdown():
            time.sleep(3)
            str_time = time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(time.time()))
            print str_time
            datas = {"vin": "taaaa-cidi", "lat": self.lon,"lng":self.lat,\
                "speed":self.speed,"status":self.mode,\
                "timestamp":str_time,"heading":self.heading,\
                "battery":self.battery,"voltage":self.voltage}
            jsdata =json.dumps(datas)
            print jsdata
            #r = requests.post("http://182.61.59.75:8010/ugv/data", data=jsdata)
            r = requests.post("http://222.180.168.240:9003/vehicle/getinfo/send", data=jsdata)
            print(r.text,r.status_code)
            

if __name__ == '__main__':
    ros_node()
