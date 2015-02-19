#!/usr/bin/env python

import rospy
import roslib
import socket
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import time


def IPSendPack(stringdata):
    global lastMessageTime

    UDP_IP = "192.168.50.9"
    UDP_Port = 7070
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
    sock.sendto(stringdata, (UDP_IP, UDP_Port))
    lastMessageTime = time.clock()    
    pub.publish(String(stringdata))

def IsVelocityCommandBelowThreshold ():
    global lastVelocityCommand
    velocityThreshold = 30
    if abs(lastVelocityCommand[0]) < velocityThreshold:
        lastVelocityCommand[0] = 0
    if abs(lastVelocityCommand[1]) < velocityThreshold:
        lastVelocityCommand[1] = 0

def SendLastVelocityCommand():
    global lastVelocityCommand
    IsVelocityCommandBelowThreshold()
    IPSendPack(str(lastVelocityCommand[0]) + '%' + str(lastVelocityCommand[1]))


def IsAheadUsed(AheadCheck):
    return AheadCheck != 1:


def IsReverseUsed(ReverseCheck):
    return ReverseCheck != 1:
 

def CalculateAheadCommand(throtleCommand):
    treadsAhead = 500*(-1*throtleCommand+1)
    return [treadsAhead,treadsAhead]    

def CalculateReverseCommand(throtleCommand):
    treadsReverse = -500*(-1*throtleCommand+1)
    return [treadsReverse,treadsReverse]

def StopMotionCommand():
    return [0,0]

def ReceiveAndSendToRecipient(joy_data):
    global lastVelocityCommand
    global timeBetweenCommands

    leftTread = float(joy_data.axes[1])
    rightTread = float(joy_data.axes[4])
    rightThrotle = float(joy_data.axes[5])
    leftThrotle = float(joy_data.axes[2])
    precisionMotionButton = int(joy_data.buttons[5])
    stopMotionButton = int(joy_data.buttons[1])
    
    if stopMotionButton != 0:
        lastVelocityCommand = StopMotionCommand()
    elif IsAheadUsed(rightThrotle):
        lastVelocityCommand = CalculateAheadCommand(rightThrotle)
    elif IsReverseUsed(leftThrotle): 
        lastVelocityCommand = CalculateReverseCommand(leftThrotle)
    elif precisionMotionButton == 1: 
        precisionFactor = 200
        lastVelocityCommand = [precisionFactor*leftTread,precisionFactor*rightTread]
    else:
        velocityFactor = 500 
        lastVelocityCommand = [velocityFactor*leftTread,velocityFactor*rightTread]
    
    if IsTimerExpired(timeBetweenCommands):        
        SendLastVelocityCommand()

def IsTimerExpired(timeBetweenCommands):
    return time.clock() - lastMessageTime > timeBetweenCommands:
    

def main(): 
    rospy.init_node('joy_joy')
    pub = rospy.Publisher('joy_joy', String)
    lastVelocityCommand = [0,0]
    lastMessageTime = time.clock()
    rospy.Subscriber('/joy', Joy, ReceiveAndSendToRecipient)
    
    while not rospy.is_shutdown():
        timeBetweenCommands = 0.3
        if IsTimerExpired(timeBetweenCommands):
            SendLastVelocityCommand()    
    
if __name__ == "__main__":
    main()
