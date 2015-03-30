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
    sock.recvfrom(0)

while 1:
    #send_udp(100,100)
    #time.sleep(10)
    talker()
    if raw_input()=="q":
        break


"""def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass"""

       
    