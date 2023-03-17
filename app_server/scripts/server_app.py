#!/usr/bin/env python
#coding=utf8
import SocketServer
import json
from abc import ABCMeta, abstractmethod
import struct
import threading
import time
import setproctitle
import sys
reload(sys)
sys.setdefaultencoding('utf8')

from socket import *
import threading
# import _thread
import json
import time
import numpy as np 
import random
import os
import rospy
from sensor_msgs.msg import NavSatFix
import math
from pyproj import Proj
import conv_WGS84 as conv
from std_msgs.msg import String
from sweeper_msgs.msg import SweepMission
from sweeper_msgs.msg import StateReport
from sensor_msgs.msg import Image
import time
import gc
import cv2
import base64
import struct
from cv_bridge import CvBridge,CvBridgeError
from sweeper_msgs.msg import SensorFaultInformation
#import numpy as np

CARNAME ='zx_008'
#PATH = "/home/zx/gpslist"
PATH = "/home/cidi/sweeper_ws/src/sweeper_haide/data/path"
HOST,PORT = "192.168.1.108",8888

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



class sockets:
    def __init__(self):
        
        BUFSIZ = 1024
        ADDRESS = (HOST, PORT)
        self.tcpServerSocket = socket(AF_INET, SOCK_STREAM)
        self.tcpServerSocket.bind(ADDRESS)
        self.tcpServerSocket.listen(2)

        # self.isOpen =False
        self.path = PATH
        self.state = 0   #0 stop  1 run  2 pause
        self.showimage ='None'
        self.txt_name = ''
        self.side = -1
        self.mode = -1
        self.pub_miss = rospy.Publisher('/sweeper/sweep/mission', SweepMission, queue_size=10)

        # self.rev_socket = threading.Thread(target=self.listen)
        # self.rev_socket.setDaemon(True)
        # self.rev_socket.start()

        # self.listen()

    def Open(self):
        self.listen()
    #     self.isOpen = True

    def Close(self):
        self.tcpServerSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self.tcp_socket.disconnect()
        # self.tcpServerSocket.close()
        print 'close!!'

    def listen(self):
        while True:
            self.tcp_socket, client_address = self.tcpServerSocket.accept() 
            print client_address
            self.rev()
                
    def rev(self):
        print 'start rev'
        rev_msg = ''
        while True:
            try:
                rev_msg = self.tcp_socket.recv(2048).decode('UTF-8', 'ignore')
                print rev_msg
            except:
                print 'socket broken!'
                break
            
            # continue 

            if not rev_msg:
                self.tcp_socket.close()
                print 'client is close!'
                break

            
                     
            if rev_msg=="disconnect":
                self.tcp_socket.close()
                break
            
            

            try:    
                info_type = json.loads(rev_msg)
            except:
                print 'not json'
                continue
            '''
            load paths name
            '''
            if info_type['type']=='getpath':
                msg_dic ={'type':'pathlist'}
                diclist = []
                try:
                    fileList = os.listdir(self.path)
                    self.error_path = False
                except:
                    self.error_path = True
                    print 'path error!'
                    pass
                for f in fileList:
                    if '.txt' in f:
                        tmpdic = {'name':f}
                        diclist.append(tmpdic)
                msg_dic.update({'list' : diclist})
                print msg_dic
                msg_js = json.dumps(msg_dic)
                self.send(msg_js.encode())
            '''
            app get a path return one path
            '''
            if info_type['type']=='setpath':
                self.txt_name = info_type['name']
                dic_points =[]
                a=0
                lines = open(self.path+'/'+self.txt_name,"r").readlines()
                if len(lines)>20:
                    count =len(lines)/20
                else:
                    count =2
                for line in lines: 
                    #print line[:-1] 
                    a+=1
                    if a>count:
                        split = line.split('|')
                        x = float(split[1])
                        y = float(split[2])
                        lon, lat = transform_utm_into_lat_lon(x,y,49,'N')
                                            
                        dic_p ={'x':lat,'y':lon}
                        dic_points.append(dic_p)
                        a=0
                msg_dic ={'type':'onepath'}
                msg_dic.update({'pathlist':dic_points})
                print(msg_dic)
                msg_js = json.dumps(msg_dic)
                self.send(msg_js.encode())
            '''
            app click on/off
            '''
            if info_type['type']=='action':
                onoff = info_type['onoff']
                mode = info_type['mode']
                smiss = SweepMission()
                smiss.side = self.side
                if onoff =='on':
                    if self.state ==0:
                        if mode == 'tb':
                            smiss.mode = 7
                        if mode == 'gpsxj':
                            smiss.mode = 4
                            smiss.line = self.txt_name
                        if mode == 'ldxj':
                            smiss.mode = 5
                            smiss.line = self.txt_name
                        if mode == 'gpslz':
                            smiss.mode = 2
                        if mode == 'ldlz':
                            smiss.mode = 3
                        if mode == 'idle':
                            smiss.mode = 1
                        if mode == 'global':
                            smiss.mode = 6
                        if mode == 'update':
                            smiss.mode = 8
                        smiss.start_cmd =1
                    else:
                        smiss.start_cmd =1
                    self.state ==1
                if onoff =='pause':
                    smiss.start_cmd = 2
                    smiss.mode = -1
                if onoff =='cancel':
                    smiss.start_cmd = 0
                    smiss.mode = -1
                    self.state =0
                print 'pub'
                self.mode = smiss.mode
                self.pub_miss.publish(smiss)
            '''
            savepath
            '''
            if info_type['type']=='savepath':
                name = info_type['name']
                smiss = SweepMission()
                smiss.start_cmd = 3
                smiss.mode = -1
                smiss.line = name
                self.pub_miss.publish(smiss)
                print name  
            '''
            delpath
            '''  
            if info_type['type']=='delpath':
                name = info_type['name']
                smiss = SweepMission()
                smiss.start_cmd = 5
                smiss.mode = -1
                smiss.line = name
                self.pub_miss.publish(smiss)
                print name 
            '''
            delcross
            '''  
            if info_type['type']=='delcross':
                #name = info_type['name']
                smiss = SweepMission()
                smiss.start_cmd = 4
                smiss.mode = -1
                #smiss.line = name
                self.pub_miss.publish(smiss)
                print name 
            '''
            showimage
            '''
            if info_type['type']=='showimage':
                self.showimage = info_type['name']
            '''
            phonedisconnect
            '''
            if info_type['type'] == 'disconnect':
                self.showimage = ''
            '''
            right_left
            '''
            if info_type['type'] == 'tiebian':
                index = info_type['which']
                smiss = SweepMission()
                smiss.start_cmd = -1
                smiss.mode = -1
                #smiss.line = name
                smiss.side = index
                self.side =index
                self.pub_miss.publish(smiss)
            if info_type['type'] == 'tiebian_acc':
                print("tiebian_acc")
                smiss = SweepMission()
                smiss.start_cmd = -1
                smiss.mode = -1
                smiss.trim_accuracy = info_type['acc']
                smiss.side = self.side
                self.pub_miss.publish(smiss)
            '''
            saveconfig
            '''
            if info_type['type'] == 'config':
                mode =info_type['mode']
                smiss = SweepMission()
                smiss.start_cmd = 6
                smiss.line = info_type['line']
                if mode == 'tb':smiss.mode = 7
                if mode == 'gpsxj':smiss.mode = 4
                if mode == 'ldxj':smiss.mode = 5
                if mode == 'gpslz':smiss.mode = 2
                if mode == 'ldlz':smiss.mode = 3
                if mode == 'idle':smiss.mode = 1
                if mode == 'global':smiss.mode = 6
                
                smiss.side = info_type['side']
                self.pub_miss.publish(smiss)
                
    def send(self,jsondata):
        try:
            self.tcp_socket.sendall(b'\xaa\xff')
            self.tcp_socket.sendall(jsondata)
            print(len(jsondata))
            self.tcp_socket.sendall(b'\xff\xaa')
        except:
            print 'not connect!!'



class ros:
    def __init__(self):
        self.rev_count =0
        self.encode_param =[int(cv2.IMWRITE_JPEG_QUALITY),15]
        self.bridge = CvBridge()
        
        rospy.init_node('server_true', anonymous=True)

        global CARNAME
        CARNAME = rospy.get_param("/carname", default="nobody")
        global PATH
        PATH = rospy.get_param("/path", default="/home/zx/gpslist")
        global HOST
        HOST = rospy.get_param("/host", default="192.168.1.108")
        # self.thead_heart = threading.Thread(target=self.thead_server)
        # self.thead_heart.setDaemon(True)
        # self.thead_heart.start()
        self.sk = sockets()
        
        rospy.Subscriber('/sweeper/gps/fix',NavSatFix,self.callback)
        rospy.Subscriber('/sweeper/sweep/state_report',StateReport,self.callback_report)
        rospy.Subscriber('/sweeper/planning/image',Image,self.callback_image)
        rospy.Subscriber('/image_view/output',Image,self.callback_cam)
        # HOST,PORT = "172.16.41.160",8888
        # self.server=socketserver.ThreadingTCPServer((HOST,PORT),MyTCPHandler)#mutilt
        # self.server.serve_forever()
        self.sk.Open()

        rospy.spin()
        
        self.sk.Close()
        

    

    

    def callback(self,data):
        self.rev_count+=1
        if self.rev_count>5:
            lat, lon = conv.wgs_gcj(data.latitude,data.longitude)
            print lat, lon
            str_lat = str(lat)
            str_lon = str(lon)
            msg_dic ={'type':'pose','lat':str_lat,'lon':str_lon}
            msg_js = json.dumps(msg_dic)
            self.sk.send(msg_js) 

    def callback_report(self,data):
        # print(data)
        dic_state =[]
        for code in data.result_code:
            dic_p ={'state':str(code)}
            dic_state.append(dic_p)
        msg_dic ={'type':'report','mode':data.mode,'cmd':data.start_cmd,\
            'line':data.line,'side':data.side,'trim_accuracy':data.trim_accuracy}
        msg_dic.update({'list':dic_state})
        msg_js = json.dumps(msg_dic)
        # print(msg_js)
        self.sk.send(msg_js) 

    def callback_image(self,data):
        if self.sk.showimage == 'planning':
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data)
                frame= cv2.resize(cv_image,(400,400))
                res,imagecode = cv2.imencode('.jpg',frame,self.encode_param)
                npdata =np.array(imagecode)
                strdata =npdata.tostring()
                self.sk.send(strdata)
            except:
                print 'bridge wrong'
    
    def callback_cam(self,data):

        # self.rev_count+=1
        # if self.rev_count<2:
        #     return
        # self.rev_count = 0
        if self.sk.showimage == 'cam':
        #if True:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data)
                frame= cv2.resize(cv_image,(400,400))
                res,imagecode = cv2.imencode('.jpg',frame,self.encode_param)
                npdata =np.array(imagecode)
                strdata =npdata.tostring()
                #print strdata
                self.sk.send(strdata)
                
            except:
                print 'bridge wrong'
        

if __name__ == '__main__':
    ros()






