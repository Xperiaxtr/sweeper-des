#!/usr/bin/env python
#coding=utf8
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

setproctitle.setproctitle("app_server")

error_path =False
error_link =False
error_socket =True
LON=21.00

CARNAME ='test'
PATH = "/home/cidi/sweeper_ws/src/sweeper_haide/data/path"

import datetime
# with open('/home/cidi/app_server.txt','w') as f:
#     d = datetime.datetime.now().strftime("%Y-%m-%d %H:%M%S")
#     f.write(d+':')

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



class sock:
    def __init__(self):
        #self.path = "/home/zx/gpslist"
        self.path = PATH
        self.state = 0   #0 stop  1 run  2 pause
        self.showimage ='None'
        self.txt_name = ''
        self.side = -1
        self.mode = -1
        self.error_socket = False
        self.error_link = True
        self.error_path = False
	self.LON =10
        self.pub_miss = rospy.Publisher('/sweeper/sweep/mission', SweepMission, queue_size=10)
        self.pub_fault = rospy.Publisher('/sweeper/common/diagnose', SensorFaultInformation, queue_size=10)
        
        # self.report_thread = threading.Thread(target=self.report)
        # self.report_thread.setDaemon(True)
        # self.report_thread.start()

        # self.connect()

        

    def connect(self):
        time.sleep(2)
        needTry = True
        cout = 0
        while needTry:
            try:
                self.tcp_socket = socket(AF_INET,SOCK_STREAM)
                self.tcp_socket.connect(('111.23.140.244',8888))

                self.rev_socket = threading.Thread(target=self.rev)
                self.rev_socket.setDaemon(True)
                self.rev_socket.start()
                print CARNAME
                msg_dic ={'type':'init','which':'car','name':CARNAME}
                msg_js = json.dumps(msg_dic)
                self.tcp_socket.sendall(msg_js)
                # self.error_socket = False

                print('socket connect！！')
                print(self.error_socket)
                needTry =False
            except:
                cout+=1
                if cout>5:
                    self.error_socket = True
                
                needTry =True
                time.sleep(2)
                print('socket not connect')
                print(self.error_socket)
        

                   

    def rev(self):
        while True:
	    rev_msg = ''
            try:
                rev_msg = self.tcp_socket.recv(2048).decode('UTF-8', 'ignore')
                # if rev_msg!='alive' or rev_msg!='alive_state':
                #     print rev_msg
            except:
                print 'socket broken!'
                time.sleep(5)
            if len(rev_msg) ==0:
                try:
                    self.connect()
                    self.error_link = True
                    print 'reconnect!'
                    break
                except:
                    continue

                     
            if rev_msg=="alive":
                self.error_link = False
                continue
            
            # with open('/home/cidi/app_server.txt','a') as f:    #设置文件对象
            #     f.write(rev_msg) 

            try:    
                info_type = json.loads(rev_msg)
            except:
                continue
            '''
            load paths name
            '''
            if info_type['type']=='getpath':
                msg_dic ={'type':'pathlist'}
                diclist = []
                # try:
                #     fileList = os.listdir(self.path)
                #     self.error_path = False
                # except:
                #     self.error_path = True
                #     pass
                fileList = os.listdir(self.path)

                cout = 0
                for f in fileList:
                    if '.txt' in f:
                        tmpdic = {'name':f}
                        if cout <15:
                            diclist.append(tmpdic)
                            cout += 1

                msg_dic.update({'list' : diclist})
                print msg_dic
                msg_js = json.dumps(msg_dic)
                self.send(msg_js.encode())
            '''
            app get a path return one path
            '''
            if info_type['type']=='setpath':
                self.txt_name = info_type['name']
                if self.txt_name == '':
                    break
                dic_points =[]
                a=0
                lines = open(self.path+'/'+self.txt_name,"r").readlines()
                if len(lines)>80:
                    count =len(lines)/80
                else:
                    count =2
                for line in lines: 
                    #print line[:-1] 
                    a+=1
                    if a>count:
                        split = line.split('|')
                        x = float(split[1])
                        y = float(split[2])
			# print LON
                        lon, lat = transform_utm_into_lat_lon(x,y,self.LON/6+31,'N')
                                            
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
                    mode_lz = info_type['mode_lz']
                    smiss.point_attribute = mode_lz
                if onoff =='pause':
                    smiss.start_cmd = 2
                    smiss.mode = -1
                if onoff =='cancel':
                    smiss.start_cmd = 0
                    smiss.mode = -1
                    self.state =0
                if onoff == 'null':
                    mode_lz = info_type['mode_lz']
                    smiss.start_cmd = -1
                    smiss.mode = -1
                    smiss.side =-1
                    smiss.trim_accuracy =-1
                    smiss.point_attribute = mode_lz
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
            if info_type['type'] =='tiebian_ll':
                print("tiebian_ll")
                smiss = SweepMission()
                index = info_type['which']
                self.side =index
                smiss.side = self.side
                smiss.start_cmd = -1
                smiss.mode = -1
                smiss.trim_accuracy = info_type['acc']
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
            # print(len(jsondata))
            self.tcp_socket.sendall(b'\xff\xaa')
        except:
            self.connect()
            print 'not connect!!'

    def report(self):
        
        error_list =[]
        if self.error_link:
            error_list.append(4102)
        if self.error_path:
            error_list.append(4103)
        if self.error_socket:
            error_list.append(4101)
        if self.error_link and self.error_path and self.error_socket:
            pass
        else:
            error_list.append(4100)
        faultinfo = SensorFaultInformation()
        faultinfo.header.frame_id ='app'
        faultinfo.state_code = error_list
        faultinfo.header.stamp = rospy.Time.now()
        self.pub_fault.publish(faultinfo)
            
    # def send_struct(self,jsondata):
    #     try:
    #         ret = struct.pacl('i', len(jsondata))
    #         self.tcp_socket.sendall(ret)
            
    #     except:
    #         self.connect()
    #         print 'not connect!!'

class ros:
    def __init__(self):


        

        self.rev_count =0
        #self.cap =cv2.VideoCapture(0)
        self.encode_param =[int(cv2.IMWRITE_JPEG_QUALITY),15]
        self.bridge = CvBridge()
        

        rospy.init_node('client_car', anonymous=True)
        global CARNAME
        CARNAME = rospy.get_param("/carname", default="nobody")
        global PATH
        PATH = rospy.get_param("/path", default="/home/zx/gpslist")
        
        self.pub_fault = rospy.Publisher('/sweeper/common/diagnose', SensorFaultInformation, queue_size=10)
        
        self.sk =sock()

        self.thead_heart = threading.Thread(target=self.thead_heart)
        self.thead_heart.setDaemon(True)
        self.thead_heart.start()

        self.sk.connect()

        rospy.Subscriber('/sweeper/gps/fix',NavSatFix,self.callback)
        rospy.Subscriber('/sweeper/sweep/state_report',StateReport,self.callback_report)
        rospy.Subscriber('/sweeper/planning/image',Image,self.callback_image)
        #rospy.Subscriber('/sweeper/sensor/camera/front',Image,self.callback_cam)
        rospy.Subscriber('/image_view/output',Image,self.callback_cam)
        

        

        rospy.spin()
        self.sk.tcp_socket.close()
        print 'rosshutdown!!'

    def thead_heart(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            error_list =[]
            if self.sk.error_socket:
                error_list.append(4101)
            
            error_list.append(4100)
            faultinfo = SensorFaultInformation()
            faultinfo.header.frame_id ='app'
            faultinfo.state_code = error_list
            faultinfo.header.stamp = rospy.Time.now()
            self.pub_fault.publish(faultinfo)
            rate.sleep()

    def callback(self,data):
        # ret,frame = self.cap.read()
        # frame= cv2.resize(frame,(600,600))
        # res,imagecode = cv2.imencode('.jpg',frame,self.encode_param)
        # npdata =np.array(imagecode)
        # strdata =npdata.tostring()
        # cv2.imshow('video',frame)
        # cv2.waitKey(1)==ord('q')
         
        

        self.rev_count+=1
        if self.rev_count>5:
            lat, lon = conv.wgs_gcj(data.latitude,data.longitude)
	    self.sk.LON = lat
            # print lat, lon
	    # print self.sk.LON
            str_lat = str(lat)
            str_lon = str(lon)
            msg_dic ={'type':'pose','lat':str_lat,'lon':str_lon}
            msg_js = json.dumps(msg_dic)
            self.sk.send(msg_js)
            
            # if self.sk.showimage=='cam':
            #     ret,frame = self.cap.read()
            #     frame= cv2.resize(frame,(600,600))
            #     res,imagecode = cv2.imencode('.jpg',frame,self.encode_param)
            #     npdata =np.array(imagecode)
            #     strdata =npdata.tostring()
            #     cv2.imshow('video',frame)
            #     cv2.waitKey(1)==ord('q')
            #     self.sk.send(strdata)
            #     pass

            self.rev_count= 0

    def callback_report(self,data):
        # print(data)
        dic_state =[]
        for code in data.result_code:
            dic_p ={'state':str(code)}
            dic_state.append(dic_p)
        msg_dic ={'type':'report','mode':data.mode,'cmd':data.start_cmd,\
            'line':data.line,'side':data.side,'trim_accuracy':data.trim_accuracy,\
            'default_mode':data.default_mode,'default_line':data.default_line}
        msg_dic.update({'list':dic_state})
        msg_js = json.dumps(msg_dic)
        # print(msg_js)
         
        try:
            self.sk.send(msg_js) 
            error_link = False
        except:
            error_link = True
        
        # print(self.sk.error_socket ,self.sk.error_link,self.sk.error_path)

        # error_list =[]
        # if self.sk.error_link:
        #     error_list.append(4102)
        # if self.sk.error_path:
        #     error_list.append(4103)
        # if self.sk.error_socket:
        #     error_list.append(4101)
        # if self.sk.error_link and self.sk.error_path and self.sk.error_socket:
        #     pass
        # else:
        #     error_list.append(4100)
        # faultinfo = SensorFaultInformation()
        # faultinfo.header.frame_id ='app'
        # faultinfo.state_code = error_list
        
        # self.sk.pub_fault.publish(faultinfo)
            
        

    def callback_image(self,data):
        if self.sk.showimage == 'planning':
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data)
                frame= cv2.resize(cv_image,(800,800))
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
                frame= cv2.resize(cv_image,(800,800))
                res,imagecode = cv2.imencode('.jpg',frame,self.encode_param)
                npdata =np.array(imagecode)
                strdata =npdata.tostring()
                #print strdata
                self.sk.send(strdata)
                
            except:
                print 'bridge wrong'

if __name__ == '__main__':
    ros()
