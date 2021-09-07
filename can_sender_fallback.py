#!/usr/bin/env python
import sys, time
import numpy as np
import roslib
import rospy
from numpy import binary_repr
# Ros Messages
import datetime
from sensor_msgs.msg import *
from caselab_msgs.msg import VehicleStatus
from fallback_msgs.msg import fallback

from canlib import canlib, Frame
from canlib.canlib import ChannelData
from bitarray import bitarray

from collections import deque

class can_sender(object):

    def __init__(self,
                 channel=0,
                 openFlags=canlib.Open.ACCEPT_VIRTUAL,
                 bitrate=canlib.canBITRATE_500K,
                 bitrate_data=None,
                 outputControl=canlib.Driver.NORMAL, 
                 valid_protocol_interval=30, 
                 print_errors=True,
                 debugging=True,
                 view_figure=True):

        rospy.init_node('can_data_sender', anonymous=True)

        self.channel = channel
        self.openFlags = openFlags
        self.bitrate = bitrate
        self.bitrate_data = bitrate_data
        self.outputControl = outputControl
        self.print_errors = print_errors
        self.debugging = debugging
        self.view_figure = view_figure
        # getting all_protocol in one set
        self.total_protocol = {}
        
        # this count to check the protocols are valid in current
        # in total_protocol, we need to remove old information
        # set this count range to [0, 1e+4]
        # To initialize this, we need to reset this value in the loop
        self.running_count = 0
        self.valid_protocol_interval = valid_protocol_interval

        self.cur_num = self.check_number_of_channel()

        self.can_channel = None

        self.can_channel = self.setUpChannel(channel=self.channel,
                                             openFlags=self.openFlags,
                                             bitrate=self.bitrate,
                                             bitrate_data=self.bitrate_data,
                                             outputControl=self.outputControl)

        self.fix_subscriber = rospy.Subscriber("/fix", NavSatFix, self.fix_callback,  queue_size = 1)
        self.vehicle_subscriber = rospy.Subscriber("/vehicle_status",VehicleStatus,self.vehicle_status_callback, queue_size=1)
        
        self.lidar_r_subscriber = rospy.Subscriber("/fallback/gnss",fallback,self.lidar_r_callback, queue_size=1)
        self.lidar_f_subscriber = rospy.Subscriber("/fallback/gnss",fallback,self.lidar_f_callback, queue_size=1)
        self.lidar_l_subscriber = rospy.Subscriber("/fallback/gnss",fallback,self.lidar_l_callback, queue_size=1)
        self.lidar_b_subscriber = rospy.Subscriber("/fallback/gnss",fallback,self.lidar_b_callback, queue_size=1)
        self.camera_subscriber = rospy.Subscriber("/fallback/gnss",fallback,self.camera_callback, queue_size=1)
        self.mobileye_subscriber = rospy.Subscriber("/fallback/gnss",fallback,self.mobileye_callback, queue_size=1)
        self.gnss_subscriber = rospy.Subscriber("/fallback/gnss",fallback,self.gnss_callback, queue_size=1)
        self.steering_subscriber = rospy.Subscriber("/fallback/gnss",fallback,self.steering_callback, queue_size=1)
        self.dcc_subscriber = rospy.Subscriber("/fallback/gnss",fallback,self.dcc_callback, queue_size=1)
        self.lane_subscriber = rospy.Subscriber("/fallback/gnss",fallback,self.lane_callback, queue_size=1)
        self.traffic_subscriber = rospy.Subscriber("/fallback/gnss",fallback,self.traffic_callback, queue_size=1)
        
        self.gnss_data_slot = {}
        self.gnss_data_slot['gps_time'] = str(datetime.datetime.now())
        self.vehicle_slot = {}

        # fallback slot >> 
        self.fallback_slot = {}
        self.fallback_slot['lidar_f'] = 0 #1
        self.fallback_slot['lidar_r'] = 0 #2
        self.fallback_slot['lidar_l'] = 0 #3
        self.fallback_slot['lidar_b'] = 0 #4
        self.fallback_slot['camera'] = 0 #5
        self.fallback_slot['mobileye'] = 0 #6
        self.fallback_slot['rane'] = 0 #7
        self.fallback_slot['gnss'] = 0 #8
        self.fallback_slot['traffic'] = 0 #9
        self.fallback_slot['steer'] = 0 #10
        self.fallback_slot['dcc'] = 0 #11
        # << fallback slot 
    
        self.rate = rospy.Rate(20)

    def fix_callback(self, ros_data):
        # lat/lng ddmmssss data >>
        lat_data = str(ros_data.latitude)
        lng_data = str(ros_data.longitude)
        lat_dd = str(lat_data[:2])
        lat_mm = str(int(( float(lat_data) - float(lat_dd) )* 60))
        lat_ss = str(((float(lat_data) - float(lat_dd))*60 - float(lat_mm)) * 60)[:5]
        lng_dd = str(lng_data[:3])
        lng_mm = str(int(( float(lng_data) - float(lng_dd) )* 60))
        lng_ss = str(((float(lng_data) - float(lng_dd))*60 - float(lng_mm)) * 60)[:5]
        lat_ss = lat_ss.replace('.','')
        lng_ss = lng_ss.replace('.','')

        self.gnss_data_slot['lat_data'] = [lat_dd,lat_mm,lat_ss[:2],lat_ss[-2:]]
        self.gnss_data_slot['lng_data'] = [lng_dd,lng_mm,lng_ss[:2],lng_ss[-2:]]
        
        # << lat/lng ddmmssss data
        # rospy.loginfo("{}".format(self.gnss_data_slot['lat_data']))
        # rospy.loginfo("\nlat_dd : {}\nlat_mm : {}\nlat_ss : {}"\
        # .format(lat_dd,lat_mm,lat_ss))
        # rospy.loginfo("{}".format(lng_data))
        # rospy.loginfo("\nlng_dd : {}\nlng_mm : {}\nlng_ss : {}"\
        # .format(lng_dd,lng_mm,lng_ss))
        # time_stamp data >>
        # unix_time = ros_data.header.stamp
        # unix_time = str(unix_time)[:12]
        # gps_time = str(str(int(unix_time[:10]) - 315964782) + unix_time[-2:])[-8:]
        # rospy.loginfo("\tgps_time : {}".format(gps_time))
        self.gnss_data_slot['gps_time'] = str(datetime.datetime.now())
        # << time_stamp data

    def check_number_of_channel(self):
        cur_num = 0
        while cur_num == 0:
            cur_num = canlib.getNumberOfChannels()
        return cur_num
    def setUpChannel(self,
                     channel=0,
                     openFlags=canlib.Open.ACCEPT_VIRTUAL,
                     bitrate=canlib.canBITRATE_500K,
                     bitrate_data=None,
                     outputControl=canlib.Driver.NORMAL):
        # ch = -1
        # while ch < 0:
        #     ch = canlib.openChannel(channel, openFlags)
        #     print(ch)
        # canlib.getNumberOfChannels()

        ch = canlib.openChannel(channel, openFlags)
        print("Using channel: %s, EAN: %s" % (ChannelData(channel).channel_name,
                                              ChannelData(channel).card_upc_no))
        ch.setBusOutputControl(outputControl)
        ch.setBusParams(bitrate)
        if bitrate_data is not None:
            # This will be set when Fd setting
            ch.setBusParamsFd(bitrate_data)
        ch.busOn()
        return ch
    def make_frame(self, id, data):
        return Frame(id_=id, data=data)
    def tearDownChannel(self):
        self.can_channel.busOff()
        self.can_channel.close()
    def bytearray_to_bit_str(self, byte_array):
        return ''.join(format(byte, '08b') for byte in byte_array)
    def binary_str_to_hex(self, bit_str):
        return hex(int(bit_str, 2))
    def hex_to_int(self, hex_val):
        return int(hex_val, 16)
    def hex_to_signed_int(self, hexstr, bits):
        value = int(hexstr,16)
        if value & (1 << (bits-1)):
            value -= 1 << bits
        return value
    def signed_10(self, value):
        return -(value & 0x200) | (value & 0x1ff)
    def signed_12(self, value):
        return -(value & 0x800) | (value & 0x7ff)
    def signed_16(self, value):
        return -(value & 0x8000) | (value & 0x7fff)
    def check_protocol_validation(self):
        # Copy total_protocol temporary
        copy_protocol = self.total_protocol.copy()
        for i, each in enumerate(copy_protocol):
            if abs(copy_protocol[each]['running_count'] - self.running_count) > self.valid_protocol_interval:
                del self.total_protocol[each]

        copy_protocol_vehicle = self.total_vehicle_status.copy()
        if 'running_count' in copy_protocol_vehicle:
            if abs(copy_protocol_vehicle['running_count'] - self.running_count) > self.valid_protocol_interval:
                self.total_vehicle_status = {}

    def vehicle_status_callback(self, ros_data):
        if ros_data.Speed>0:
            self.vehicle_slot['speed'] = ros_data.Speed
        else: # minus case
            self.vehicle_slot['speed'] = 0
        self.vehicle_slot['steer'] = ros_data.SteearingAngle
        # print("steer ",self.vehicle_slot['steer'])

    # fallback >> 
    def lidar_r_callback(self, ros_data):
        self.fallback_slot['lidar_r'] = ros_data.gps_fallback
    def lidar_f_callback(self, ros_data):
        self.fallback_slot['lidar_f'] = ros_data.gps_fallback
    def lidar_l_callback(self, ros_data):
        self.fallback_slot['lidar_l'] = ros_data.gps_fallback
    def lidar_b_callback(self, ros_data):
        self.fallback_slot['lidar_b'] = ros_data.gps_fallback
    def camera_callback(self, ros_data):
        self.fallback_slot['camera'] = ros_data.gps_fallback
    def mobileye_callback(self, ros_data):
        self.fallback_slot['mobileye'] = ros_data.gps_fallback
    def gnss_callback(self, ros_data):
        self.fallback_slot['gnss'] = ros_data.gps_fallback
    def steering_callback(self, ros_data):
        self.fallback_slot['steer'] = ros_data.gps_fallback
    def dcc_callback(self, ros_data):
        self.fallback_slot['dcc'] = ros_data.gps_fallback
    def lane_callback(self, ros_data):
        self.fallback_slot['lane'] = ros_data.gps_fallback
    def traffic_callback(self, ros_data):
        self.fallback_slot['traffic'] = ros_data.gps_fallback
    # << fallback
    
    def send_x208_can_data(self,id):
        # lat/lng : [dd, mm, ss, ss]
        if 'lat_data' in self.gnss_data_slot and 'lng_data' in self.gnss_data_slot  :
            byte_0 = abs(int(self.gnss_data_slot['lat_data'][0]))
            byte_1 = abs(int(self.gnss_data_slot['lat_data'][1]))
            byte_2 = abs(int(self.gnss_data_slot['lat_data'][2]))
            byte_3 = abs(int(self.gnss_data_slot['lat_data'][3]))
            
            byte_4 = abs(int(self.gnss_data_slot['lng_data'][0]))
            byte_5 = abs(int(self.gnss_data_slot['lng_data'][1]))
            byte_6 = abs(int(self.gnss_data_slot['lng_data'][2]))
            byte_7 = abs(int(self.gnss_data_slot['lng_data'][3]))
            return self.make_frame(id=id, data=[byte_0, byte_1, byte_2, byte_3, byte_4, byte_5, byte_6, byte_7])
        else:
            return self.make_frame(id=id, data=[0,0,0,0,0,0,0,0])

    def send_x209_can_data(self,id,steer=None):
        if 'speed' in self.vehicle_slot:
            byte_4 = abs(int(self.vehicle_slot['speed']))
            speed_ands = abs(int(self.vehicle_slot['speed']*100)) #to ands
            byte_0 = speed_ands & 0x00ff # to ands
            byte_1 = (speed_ands & 0xff00) >> 8 # to ands

            if 'steer' in self.vehicle_slot:
                steer = int(self.vehicle_slot['steer'])
                # if steer < 0:
                #     preprocessed_steer = 0x8000 | (steer+0x8000)
                # else:
                #     preprocessed_steer = steer
                # byte_5 = (preprocessed_steer & 0x00ff)
                # byte_6 = (preprocessed_steer & 0xff00) >> 8
                # # print(byte_6, byte_5, preprocessed_steer)
                # # print(binary_repr(steer, width=4))
                if steer < 0:
                    preprocessed_steer = 0x8000 | (steer+0x8000) # 맨 앞에 1을 붙여줌
                else:
                    preprocessed_steer = steer

                if preprocessed_steer & 0x8000 > 0 :
                    byte_5 = (preprocessed_steer & 0x00ff)
                    byte_6 = (preprocessed_steer & 0xff00) >> 8
                else:
                    byte_5 = (preprocessed_steer & 0x00ff)
                    byte_6 = (preprocessed_steer & 0xff00) >> 8
                return self.make_frame(id=id, data=[byte_0,byte_1,0,0,byte_4,byte_5,byte_6,0])
            else:
                speed_ands = abs(int(self.vehicle_slot['speed']*100)) 
                byte_0 = speed_ands & 0x00ff
                byte_1 = (speed_ands & 0xff00) >> 8
                return self.make_frame(id=id, data=[byte_0,byte_1,0,0,byte_4,0,0,0])
        else:
            if 'steer' in self.vehicle_slot:
                steer = int(self.vehicle_slot['steer'])
                # if steer < 0:
                #     preprocessed_steer = 0x8000 | (steer+0x8000)
                # else:
                #     preprocessed_steer = steer
                # byte_5 = (preprocessed_steer & 0x00ff)
                # byte_6 = (preprocessed_steer & 0xff00) >> 8
                if steer < 0:
                    preprocessed_steer = 0x8000 | (steer+0x8000) # 맨 앞에 1을 붙여줌
                else:
                    preprocessed_steer = steer

                if preprocessed_steer & 0x8000 > 0 :
                    byte_5 = (preprocessed_steer & 0x00ff)
                    byte_6 = (preprocessed_steer & 0xff00) >> 8
                else:
                    byte_5 = (preprocessed_steer & 0x00ff)
                    byte_6 = (preprocessed_steer & 0xff00) >> 8
                return self.make_frame(id=id, data=[0,0,0,0,0,byte_5,byte_6,0])
            else:
                return self.make_frame(id=id, data=[0,0,0,0,0,0,0,0])

    def send_x210_can_data(self,id):
        if self.fallback_slot['lidar_f']==1:
            bit_0=1
        else:
            bit_0=0
        
        if self.fallback_slot['lidar_b']==1:
            bit_1=1
        else:
            bit_1=0
            
        if self.fallback_slot['lidar_r']==1:
            bit_2=1
        else:
            bit_2=0
            
        if self.fallback_slot['lidar_l']==1:
            bit_3=1
        else:
            bit_3=0
            
        if self.fallback_slot['camera']==1:
            bit_4=1
        else:
            bit_4=0
            
        if self.fallback_slot['mobileye']==1:
            bit_5=1
        else:
            bit_5=0
            
        if self.fallback_slot['gnss']==1:
            bit_6=1
        else:
            bit_6=0
            
        if self.fallback_slot['steer']==1:
            bit_7=1
        else:
            bit_7=0
            
        if self.fallback_slot['dcc']==1:
            bit_8=1
        else:
            bit_8=0
            
        if self.allback_slot['rane']==1:
            bit_9=1
        else:
            bit_9=0
            
        if self.allback_slot['traffic']==1:
            bit_10=1
        else:
            bit_10=0
            
        temp_byte0 = str(bit_7)+str(bit_6)+str(bit_5)+str(bit_4)+str(bit_3)+str(bit_2)+str(bit_1)+str(bit_0)
        #sample >> 2015-04-19 12:11:32.6690
        self.gnss_data_slot['gps_time'] = str(datetime.datetime.now())
        self.gnss_data_slot['hh'] = str(self.gnss_data_slot['gps_time']).split(" ")[-1].split(":")[0]
        self.gnss_data_slot['mm'] = str(self.gnss_data_slot['gps_time']).split(" ")[-1].split(":")[1]
        self.gnss_data_slot['ss1'] = str(self.gnss_data_slot['gps_time']).split(" ")[-1].split(":")[-1].split(".")[0]
        self.gnss_data_slot['ss2'] = str(self.gnss_data_slot['gps_time']).split(" ")[-1].split(":")[-1].split(".")[-1][:2]
        
        byte0 = int(temp_byte0,2)
        temp_byte1 = "00000"+str(self.traffic_fb)+str(self.lane_fb)+str(self.dcc_fb)
        byte1 = int(temp_byte1,2)
        
        byte4 = abs(int(self.gnss_data_slot['hh']))
        byte5 = abs(int(self.gnss_data_slot['mm']))
        byte6 = abs(int(self.gnss_data_slot['ss1']))
        byte7 = abs(int(self.gnss_data_slot['ss2']))
        return self.make_frame(id=id, data=[byte0,byte1,0,0,byte4,byte5,byte6,byte7])

    def send_all_can_data(self):
            self.can_channel.write(self.send_x208_can_data(id=520))
            self.can_channel.write(self.send_x209_can_data(id=521))
            self.can_channel.write(self.send_x210_can_data(id=528))
            
    def main(self):
        while not rospy.is_shutdown():
            try:
                self.send_all_can_data()
                self.can_channel.writeSync(None)
                # recv_frame = self.can_channel.read()
                # protocol_func = self.protocol_switcher(recv_frame.id)
                # if protocol_func is not None:
                #     protocol_func(recv_frame)
                # self.check_protocol_validation()
                self.running_count += 1
                if self.running_count > 1e+4:
                    self.running_count = 0
                self.rate.sleep()

            except (canlib.canNoMsg) as ex:
                    pass
            except (canlib.canError) as ex:
                print(ex)
    # rospy.spin()
if __name__ == '__main__':

    cur_open_flags = canlib.Open.EXCLUSIVE
    # cur_open_flags = canlib.Open.CAN_FD
    cur_bitrate = canlib.canBITRATE_500K
    # cur_bitrate = canlib.canFD_BITRATE_500K_80P
    # cur_bitrate_data = canlib.canFD_BITRATE_2M_80P
    cur_output_control = canlib.Driver.NORMAL
    
    sender = can_sender(channel=1,
                        openFlags=canlib.Open.ACCEPT_VIRTUAL,
                        bitrate=canlib.canBITRATE_500K,
                        bitrate_data=None,
                        outputControl=canlib.Driver.NORMAL, 
                        valid_protocol_interval=30, 
                        print_errors=True,
                        debugging=False,
                        view_figure=False)

    sender.main()
    sender.tearDownChannel()