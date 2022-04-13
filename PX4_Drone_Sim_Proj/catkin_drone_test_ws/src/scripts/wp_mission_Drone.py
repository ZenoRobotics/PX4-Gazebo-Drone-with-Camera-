#!/usr/bin/env python3
#***************************************************************************
#Original Base code:
#
#   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#
# @author Andreas Antener <andreas@uaventure.com>
#

# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
#***************************************************************************/

from __future__ import division

PKG = 'px4'

#import mav_cmd_codes
import time
import rospy
import glob
#import json
import math
import os
import sys
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros import mavlink
from mavros_msgs.msg import Mavlink, Waypoint, WaypointReached
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from threading import Thread
from std_msgs.msg import Header
import numpy as np
from tf.transformations import quaternion_from_euler

class wpMissionCnt:

    def __init__(self):
        self.wp =Waypoint()
        
    def setWaypoints(self,frame,command,is_current,autocontinue,param1,param2,param3,param4,x_lat,y_long,z_alt):
        self.wp.frame =frame #  FRAME_GLOBAL_REL_ALT = 3 for more visit http://docs.ros.org/api/mavros_msgs/html/msg/Waypoint.html
        self.wp.command = command #'''VTOL TAKEOFF = 84,NAV_WAYPOINT = 16, TAKE_OFF=22 for checking out other parameters go to https://github.com/mavlink/mavros/blob/master/mavros_msgs/msg/CommandCode.msg'''
        self.wp.is_current= is_current
        self.wp.autocontinue = autocontinue # enable taking and following upcoming waypoints automatically 
        self.wp.param1=param1 # no idea what these are for but the script will work so go ahead
        self.wp.param2=param2
        self.wp.param3=param3
        self.wp.param4=param4
        self.wp.x_lat= x_lat 
        self.wp.y_long=y_long
        self.wp.z_alt= z_alt #relative altitude.

        return self.wp


class MavrosMissionTest(MavrosTestCommon):
    """
    Run a mission
    """

    def setUp(self):
        super(self.__class__, self).setUp()

        self.mission_item_reached = -1  # first mission item is 0
        self.mission_name = ""

        self.mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)
        self.mission_item_reached_sub = rospy.Subscriber(
            'mavros/mission/reached', WaypointReached,
            self.mission_item_reached_callback)

#-------
        self.pos = PoseStamped()
        self.radius = 5     #PZ Changed from 1

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()
#-------

        # need to simulate heartbeat to prevent datalink loss detection
        self.hb_mav_msg = mavutil.mavlink.MAVLink_heartbeat_message(
           mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
        self.hb_mav_msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
        self.hb_ros_msg = mavlink.convert_to_rosmsg(self.hb_mav_msg)
        self.hb_thread = Thread(target=self.send_heartbeat, args=())
        self.hb_thread.daemon = True
        self.hb_thread.start()

   
    def tearDown(self):
        super(MavrosMissionTest, self).tearDown()

    #
    # Helper methods
    #
    def send_heartbeat(self):
        rate = rospy.Rate(2)  # Hz
        #Debug
        rospy.loginfo("******* Inside send_heartbeat *******")
        while not rospy.is_shutdown():
            self.mavlink_pub.publish(self.hb_ros_msg)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def mission_item_reached_callback(self, data):
        if self.mission_item_reached != data.wp_seq:
            rospy.loginfo("mission item reached: {0}".format(data.wp_seq))
            self.mission_item_reached = data.wp_seq

    def send_pos(self):
        
        rate = rospy.Rate(5)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"
        #Debug
        rospy.loginfo("******* Inside send_pos *******")

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset


    def next_lat_long(self, nlat, nlong):
        start_lat  = self.global_position.latitude
        start_long = self.global_position.longitude
        
        while start_lat == 0.0 :
           start_lat  = self.global_position.latitude
           start_long = self.global_position.longitude
           
        next_lat  = start_lat  + nlat       #all in degrees
        next_long = start_long + nlong      #all in degrees
         
        return next_lat, next_long
    
    
    # Generate lawn mower pattern waypoint mission
    # inputs: start lat, long, targeted altitude & speed, box orientation direction vector (degrees off North),
    #         start box scan location (top R/L, bottom R/L), sensor FOV, scan overlap, scan box x range (l to r side, or r to l side), 
    #         number of legs before return.
    #         Similar parameters to those found at https://met.nps.edu/~ldm/track/mry/
    # 
    def gen_lm_ptn_mission(self):
        #will pass these in as variable argument, using this as test parameters for now
        alt                = 50.0       #altitude in meters
        
        wps = [] #List to story waypoints
        
        start_lat  = self.global_position.latitude
        start_long = self.global_position.longitude
        
        while start_lat == 0.0 :
           start_lat  = self.global_position.latitude
           start_long = self.global_position.longitude
        
        lat_offset         = 0.001
        long_offset        = 0.001

        #assuming: 1) initial bottom left of lawn mower box area start, 2) then scanning L-R-L & up, 3) due north
        NAV_TAKEOFF     = 22            # Takeoff from ground / hand.
        NAV_WAYPOINT    = 16            # Navigate to waypoint.
        DO_LAND_START   = 189           # Mission command to perform a landing.
        NAV_LOITER_TO_ALT =  31         # Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current
        NAV_LAND        = 21            # Land at location.

        #takeoff point
        wps.append(wpMissionCnt().setWaypoints(3,NAV_TAKEOFF,True,True,0.0,0.0,0.0,float('nan'),start_lat,start_long,alt))

        #set waypoints -
        #starting point
        wps.append(wpMissionCnt().setWaypoints(3,NAV_WAYPOINT,False,True,0.0,0.0,0.0,float('nan'),start_lat,start_long,alt))
        
        next_lat, next_long  = self.next_lat_long(0,long_offset)       #all in degrees
        pos_xy_d, pos_z_d = self.distance_to_wp(next_lat,next_long, alt)
        rospy.loginfo("*** Distance between start lat, long and next lat, long = %f",pos_xy_d)
        wps.append(wpMissionCnt().setWaypoints(3,NAV_WAYPOINT,False,True,0.0,0.0,0.0,float('nan'),next_lat,next_long,alt))
        
        next_lat, next_long  = self.next_lat_long(lat_offset,0)       #all in degrees
        pos_xy_d, pos_z_d = self.distance_to_wp(next_lat,next_long, alt)
        rospy.loginfo("*** Distance between start lat, long and next lat, long = %f",pos_xy_d)
        wps.append(wpMissionCnt().setWaypoints(3,NAV_WAYPOINT,False,True,0.0,0.0,0.0,float('nan'),next_lat,next_long,alt))
        
        wps.append(wpMissionCnt().setWaypoints(3,NAV_WAYPOINT,False,True,0.0,0.0,0.0,float('nan'),start_lat,start_long,alt))
        
      
        #last 3 waypoint required for plane to take off
        wps.append(wpMissionCnt().setWaypoints(2,DO_LAND_START,False,True,0.0,0.0,0.0,float('nan'),0,0,0))

        #wps.append(wpMissionCnt().setWaypoints(3,NAV_LOITER_TO_ALT,False,True,1.0,alt/2,0.0,float('nan'),start_lat,start_long,0))
       
        wps.append(wpMissionCnt().setWaypoints(3,NAV_LAND,False,False,0.0,0.0,0.0,float('nan'),start_lat,start_long,0))
        
        return wps
        

    def distance_to_wp(self, lat, lon, alt):
        """alt(amsl): meters"""
        R = 6371000  # metres = mean radius of earth
        rlat1 = math.radians(lat)
        
        curr_lat  = self.global_position.latitude
        curr_long = self.global_position.longitude
        
        
        while curr_lat == 0.0 :
           curr_lat  =  self.global_position.latitude
           curr_long =  self.global_position.longitude
           
           
        rlat2 = math.radians(self.global_position.latitude)

        rlat_d = math.radians(curr_lat - lat)
        rlon_d = math.radians(curr_long - lon)
        #Haversine formula
        a = (math.sin(rlat_d / 2) * math.sin(rlat_d / 2) + math.cos(rlat1) *
             math.cos(rlat2) * math.sin(rlon_d / 2) * math.sin(rlon_d / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        d = R * c
        alt_d = abs(alt - self.altitude.amsl)

        rospy.logdebug("d: {0}, alt_d: {1}".format(d, alt_d))
        return d, alt_d

    def reach_position_coord(self, lat, lon, alt, timeout, index):
        """alt(amsl): meters, timeout(int): seconds"""
        rospy.loginfo(
           "trying to reach waypoint | lat: {0:.9f}, lon: {1:.9f}, alt: {2:.2f}, index: {3}".
           format(lat, lon, alt, index))
        best_pos_xy_d = None
        best_pos_z_d = None
        reached = False
        mission_length = len(self.mission_wp.waypoints)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
           pos_xy_d, pos_z_d = self.distance_to_wp(lat, lon, alt)

            # remember best distances
           if not best_pos_xy_d or best_pos_xy_d > pos_xy_d:
               best_pos_xy_d = pos_xy_d
           if not best_pos_z_d or best_pos_z_d > pos_z_d:
               best_pos_z_d = pos_z_d

            # FCU advanced to the next mission item, or finished mission
           reached = (
                # advanced to next wp
               (index < self.mission_wp.current_seq)
                # end of mission
               or (index == (mission_length - 1) and
                   self.mission_item_reached == index))

           if reached:
               rospy.loginfo(
                   "position reached | pos_xy_d: {0:.2f}, pos_z_d: {1:.2f}, index: {2} | seconds: {3} of {4}".
                   format(pos_xy_d, pos_z_d, index, i / loop_freq, timeout))
               break
           elif i == 0 or ((i / loop_freq) % 10) == 0:
                # log distance first iteration and every 10 sec
               rospy.loginfo(
                   "current distance to waypoint | pos_xy_d: {0:.2f}, pos_z_d: {1:.2f}, index: {2}".
                   format(pos_xy_d, pos_z_d, index))

           try:
               rate.sleep()
           except rospy.ROSException as e:
               self.fail(e)

        self.assertTrue(
           reached,
           "position not reached | lat: {0:.9f}, lon: {1:.9f}, alt: {2:.2f}, current pos_xy_d: {3:.2f}, current pos_z_d: {4:.2f}, best pos_xy_d: {5:.2f}, best pos_z_d: {6:.2f}, index: {7} | timeout(seconds): {8}".
           format(lat, lon, alt, pos_xy_d, pos_z_d, best_pos_xy_d,
                  best_pos_z_d, index, timeout))


    #def path_tracker():
    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(x, y, yaw)          #euler_to_quaternion(x, y, yaw) 
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))


    #
    # Test method
    #
    def test_mission(self):
        """Test mission"""
        
        try:
           #call new waypoint gen function for wp object list
           wps = self.gen_lm_ptn_mission()
        except IOError as e:
           self.fail(e)

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                  10, -1)
        self.wait_for_mav_type(10)

        # push waypoints to FCU and start mission
        self.send_wps(wps, 30)
        self.log_topic_vars()

        self.set_mode("AUTO.MISSION", 10)
        self.set_arm(True, 10)
        
        #head for first waypoint
        index = 0
        alt = 25
        
        #run wp mission
        for index in xrange(len(wps)):
           waypoint = wps[index]
           self.reach_position_coord(waypoint.x_lat, waypoint.y_long, alt, 100,
                                   index)


        # cleanup & disarm
        
        self.clear_wps(5) 
        self.set_arm(False, 5)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)

    name = "mavros_mission_test"

    #print ("Arguments count: ", len(sys.argv))
    #for i, arg in enumerate(sys.argv):
    #    print ("Argument ",i , " = ", sys.argv[i]) 

   
    rostest.rosrun(PKG, name, MavrosMissionTest)
      
    
    
