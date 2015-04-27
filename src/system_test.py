#!/usr/bin/env python

import rospy
import socket
import roscopter.msg


class MasterBrain(object):
    def __init__(self):
        rospy.init_node("master_brain")
        rospy.Subscriber("/filtered_pos", roscopter.msg.FilteredPosition,
                        self._position_callback) 
        self.heading = 0
        self.lat = 0
        self.lon = 0

    def send_udp(self, left_speed,right_speed,udp_port=7070,udp_ip="192.168.17.150"):
        message = str(left_speed)+"%"+str(right_speed)
        # print "UDP target IP:", udp_ip
        # print "UDP target port:", udp_port
        # print "message:", message
        sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        sock.sendto(message, (udp_ip,udp_port))
    
    def pick_motor_cmd(self):
        return self.heading / 100.0
        # 8000 20000

    def heading_demo(self):
        speed = self.pick_motor_cmd()
        self.send_udp(speed, speed);
    
    def _position_callback(self, data):
        print "HEADING", data.heading
        self.heading = data.heading
        self.lat = data.latitude
        self.lon = data.longitude        
        
        self.heading_demo()


if __name__ == "__main__":
    bender = MasterBrain()
    rospy.spin()
    
