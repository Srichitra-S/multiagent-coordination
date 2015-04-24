#!/usr/bin/env python

import socket
import time
import rospy
from std_msgs.msg import String

def send_udp(left_speed,right_speed,udp_port=7070,udp_ip="192.168.17.150"):
    message = str(left_speed)+"%"+str(right_speed)
    
    print "UDP target IP:", udp_ip
    print "UDP target port:", udp_port
    print "message:", message
    
    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.sendto(message, (udp_ip,udp_port))

left = ''
right = ''
while (left!='-1') and (right!='-1'):
    left = raw_input("left")
    right = raw_input("right")

