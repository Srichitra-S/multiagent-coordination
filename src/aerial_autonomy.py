#!/usr/bin/env python

import time
import socket, select
import json
import math
import os

import rospkg
import rospy
import rosbag
import roscopter
import roscopter.msg
import roscopter.srv
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geodesy import utm


class QuadcopterBrain(object):
    '''
    High-level quadcopter controller.
    '''
    def __init__(self):
        self.clear_waypoints_service = rospy.ServiceProxy(
            'clear_waypoints', Empty)
        self.command_service = rospy.ServiceProxy(
            'command', roscopter.srv.APMCommand)
        self.waypoint_service = rospy.ServiceProxy(
            'waypoint', roscopter.srv.SendWaypoint)
        self.trigger_auto_service = rospy.ServiceProxy(
            'trigger_auto', Empty)
        self.adjust_throttle_service = rospy.ServiceProxy(
            'adjust_throttle', Empty)

    def arm(self):
        self.command_service(roscopter.srv.APMCommandRequest.CMD_ARM)
        print('Armed')

    def launch(self):
        self.command_service(roscopter.srv.APMCommandRequest.CMD_LAUNCH)
        print('Launched')
        time.sleep(5)

    def go_to_waypoints(self, waypoint_data):
        waypoints = [build_waypoint(datum) for datum in waypoint_data]
        for waypoint in waypoints:
            self.send_waypoint(waypoint)

    def land(self):
        self.command_service(roscopter.srv.APMCommandRequest.CMD_LAND)
        print('Landing')

    def send_waypoint(self, waypoint):
        self.trigger_auto_service()
        self.adjust_throttle_service()
        successfully_sent_waypoint = False
        tries = 0

        while not successfully_sent_waypoint and tries < 5:
            res = self.waypoint_service(waypoint)
            successfully_sent_waypoint = res.result
            tries += 1
            if successfully_sent_waypoint:
                print('Sent waypoint %d, %d' % (waypoint.latitude,
                                                waypoint.longitude))
                time.sleep(15)
                print "should have reached waypoint"
                # print self.check_reached_waypoint(waypoint)
            else:
                print("Failed to send waypoint %d, %d" % (waypoint.latitude,
                                                          waypoint.longitude))
                time.sleep(0.1)
                if tries == 5:
                    print("Tried %d times and giving up" % (tries))
                else:
                    print("Retrying. Tries: %d" % (tries))

    def fly_path(self, waypoint_data):
        self.arm()
        self.launch()
        self.go_to_waypoints(waypoint_data)
        self.land()


def build_waypoint(data):
    '''
    data: dictionary with latitude and longitude
          (altitude and hold_time optional)
    '''
    latitude = data['latitude']
    longitude = data['longitude']
    altitude = data.get('altitude', 8)
    hold_time = data.get('hold_time', 3.0)

    waypoint = roscopter.msg.Waypoint()
    waypoint.latitude = gps_to_mavlink(latitude)
    waypoint.longitude = gps_to_mavlink(longitude)
    waypoint.altitude = int(altitude * 1000)
    waypoint.hold_time = int(hold_time * 1000)  # in ms
    waypoint.waypoint_type = roscopter.msg.Waypoint.TYPE_NAV
    return waypoint


def gps_to_mavlink(coordinate):
    '''
    coordinate: decimal degrees
    '''
    return int(coordinate * 1e7)


def open_waypoint_file(filename):
    f = open(filename)
    waypoints = json.load(f)
    rospack = rospkg.RosPack()
    quadcopter_brain_path = rospack.get_path("quadcopter_brain")
    source_path = "src"
    file_path = os.path.join(quadcopter_brain_path, source_path, filename)
    with open(file_path, "r") as f:
        waypoints = json.load(f)
    return waypoints


def get_trackbot_gps_coord():
    print "Attempting to get gps from trackbot"
    UDP_Port = 61557  # port trackbot writes to
    bufferSize = 1024 
    
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('<broadcast>', UDP_Port))
    s.setblocking(0)

    lat = 0
    lon = 0
    wait_for_data=True

    while wait_for_data:
        print "Waiting for data"
        result = select.select([s],[],[])
        msg = result[0][0].recv(bufferSize) 
        print "GPS msg:", msg
        # split on commas, indexes 7 and 9
        gps_data = msg.split(',')
        lat = float(gps_data[6])
        lon = float(gps_data[8])
        if lat > 0 or lon > 0:
            wait_for_data = False

    return lat, lon
    

def main():
    rospy.init_node("quadcopter_brain")
    outside = rospy.get_param("outside", False)
    nibbler = QuadcopterBrain()
    nibbler.clear_waypoints_service()
    print "Outside: ", outside
    raw_input("Press enter to continue once system is ready")
    #great_lawn_waypoints = open_waypoint_file(
    #    "waypoint_data/great_lawn_waypoints.json")
    if outside:
        carl.arm()

    lat, lon = get_trackbot_gps_coord()
    print "recieved", lat, lon
    trackbot_waypt = {} 
    trackbot_waypt['latitude'] = lat
    trackbot_waypt['longitude'] = lon
    trackbot_waypt['altitude'] = 6

    nibbler.fly_path([trackbot_waypt])

if __name__ == '__main__':
    main()

