from canlib import canlib, Frame
from canlib.canlib import ChannelData
from bitarray import bitarray
import matplotlib.pyplot as plt
import numpy as np

import rospy
from std_msgs.msg import Int16

class vehicle_parser(object):
    
    def __init__(self,
                 channel=0,
                 openFlags=canlib.Open.ACCEPT_VIRTUAL,
                 bitrate=canlib.canBITRATE_500K,
                 bitrate_data=None,
                 outputControl=canlib.Driver.NORMAL,
                 valid_protocol_interval=30):

        rospy.init_node('car_can_reader', anonymous=True)

        
        self.channel = channel
        self.openFlags = openFlags
        self.bitrate = bitrate
        self.bitrate_data = bitrate_data
        self.outputControl = outputControl
        
        self.switcher = {
            882: self.vehicel_0x372_protocol,
            902: self.vehicel_0x386_protocol,
        }
        
        self.total_protocol = {}
        self.running_count = 0
        self.valid_protocol_interval = valid_protocol_interval
        self.cur_num = self.check_number_of_channel()
        self.can_channel = None
        self.can_channel = self.setUpChannel(channel=self.channel,
                                             openFlags=self.openFlags,
                                             bitrate=self.bitrate,
                                             bitrate_data=self.bitrate_data,
                                             outputControl=self.outputControl)
        self.total_vehicle_status = {}

        self.prnd_pub = rospy.Publisher('car_can_prnd', Int16, queue_size=10)
        self.rpm_pub = rospy.Publisher('car_can_rpm', Int16, queue_size=10)
        self.rate = rospy.Rate(10)
        
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
        
    def protocol_switcher(self, id):
        
        func = self.switcher.get(id, None)
        
        return func
        
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
    
    def signed_8(self, value):
        return -(value & 0x80) | (value & 0x7f)
    
    def signed_9(self, value):
        return -(value & 0x100) | (value & 0x0ff)
        
    def signed_10(self, value):
        return -(value & 0x200) | (value & 0x1ff)
    
    def signed_11(self, value):
        return -(value & 0x400) | (value & 0x3ff)
    
    def signed_12(self, value):
        return -(value & 0x800) | (value & 0x7ff)
    
    def signed_16(self, value):
        return -(value & 0x8000) | (value & 0x7fff)
                
    def vehicel_0x372_protocol(self, frame):
        if len(frame.data) != 8:
            if self.print_errors is True:
                print('data length is not matched with predefined protocol')
            return None
        
        total_0x372_protocol_results = {}
        speed_bit = (frame.data[0] & 0x0f)
        speed = speed_bit
        total_0x372_protocol_results['speed'] = speed
        prnd_bit = frame.data[2] & 0x03
        prnd = prnd_bit
        total_0x372_protocol_results['prnd'] = prnd

        self.prnd_pub.publish(prnd)
        
        # rpm_bit = ((frame.data[6] & 0xff) << 8) + (frame.data[5] & 0xff)
        accel_pedal_bit = (frame.data[7] & 0xff)
        accel_pedal = accel_pedal_bit
        total_0x372_protocol_results['accel_pedal'] = accel_pedal
        total_0x372_protocol_results['running_count'] = self.running_count
        self.total_protocol[frame.id] = total_0x372_protocol_results
        
    def vehicel_0x386_protocol(self, frame):
        if len(frame.data) != 8:
            if self.print_errors is True:
                print('data length is not matched with predefined protocol')
            return None
        
        total_0x386_protocol_results = {}
        rpm_bit = ((frame.data[1] & 0x0f) << 8) + (frame.data[0] & 0xff)
        # rpm_bit = frame.data[0]
        # rpm_bit = (frame.data[2] << 8) + frame.data[1]
        # rpm = self.signed_10(rpm_bit)
        rpm = rpm_bit
        total_0x386_protocol_results['rpm'] = rpm
        total_0x386_protocol_results['running_count'] = self.running_count
        self.total_protocol[frame.id] = total_0x386_protocol_results
        
        self.rpm_pub.publish(rpm)

    def receive_frames(self):
        while not rospy.is_shutdown():
            try:
                recv_frame = self.can_channel.read()
                protocol_func = self.protocol_switcher(recv_frame.id)
                if protocol_func is not None:
                    protocol_func(recv_frame)
                print(self.total_protocol)
                self.check_protocol_validation()
                self.running_count += 1
                if self.running_count > 1e+4:
                    self.running_count = 0
            except (canlib.canNoMsg) as ex:
                pass
            except (canlib.canError) as ex:
                print(ex)

if __name__ == '__main__':
    print("canlib version:", canlib.dllversion())
    cur_open_flags = canlib.Open.EXCLUSIVE
    # cur_open_flags = canlib.Open.CAN_FD
    cur_bitrate = canlib.canBITRATE_500K
    # cur_bitrate = canlib.canFD_BITRATE_500K_80P
    # cur_bitrate_data = canlib.canFD_BITRATE_2M_80P
    cur_output_control = canlib.Driver.NORMAL
    vehicle = vehicle_parser(channel=0,
                             openFlags=cur_open_flags,
                             bitrate=cur_bitrate,
                             bitrate_data=None,
                             outputControl=cur_output_control,
                             valid_protocol_interval=30,
                             )
    vehicle.receive_frames()
    vehicle.tearDownChannel()
