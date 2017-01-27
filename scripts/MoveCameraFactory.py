#!/usr/bin/env python

import roslib
roslib.load_manifest("view_controller_msgs")

import rospy
from math import *
from view_controller_msgs.msg import CameraPlacement
from geometry_msgs.msg import Point, Vector3

rospy.init_node("camera_test", anonymous = True)

pub = rospy.Publisher("/rviz/camera_placement", CameraPlacement, queue_size = 1)

rate_float = 0.5
rate = rospy.Rate(rate_float)

time_to_sleep = 1
rospy.sleep(time_to_sleep)
counter = 10

while not rospy.is_shutdown():

  #print "Top of loop!"

  t = rospy.get_time()
  cp = CameraPlacement()
  r = 5.7

  f_center = 3.5
  f_range = 2
  time_to_sleep = 1

  counter = counter-1
  if counter<=0:
    p = Point(0.016195,-4.8639,2.0896)
    f = Point(2.9244,-1.0542,0.12057)
  elif counter==1:
    p = Point(0,-4.8,3)
    time_to_sleep = 2
    f = Point(f_center, -0.65, 0)
  elif counter==2:
    p = Point(7,-4.8,3)
  elif counter==3:
    f = Point(f_center + f_range, -0.65, 0)
  elif counter==4:
    p = Point(7,4.2,3)
    time_to_sleep = 2
  elif counter==5:
    f = Point(f_center, -0.65, 0)
  elif counter==6:
    p = Point(0,4.2,3)
  elif counter==7:
    f = Point(f_center - f_range, -0.65, 0)
  elif counter==8:
    p = Point(0,-4.8,3)
  else:
    p = Point(0,-4.8,3)
    f = Point(f_center, -0.65, 0)
  # p = Point(r*cos(2*pi*t/20), r*sin(2*pi*t/20), 2)
  #p = Point(5,5,0)
  cp.eye.point = p
  cp.eye.header.frame_id = "world"

  # f = Point(0, 0, 0)
  # f = Point(3.5, -0.65, 0)
  cp.focus.point = f
  cp.focus.header.frame_id = "world"

  up = Vector3(0, 0, 1)
  cp.up.vector = up
  cp.up.header.frame_id = "world"

  cp.time_from_start = rospy.Duration(time_to_sleep)
  print "Publishing a message!"
  pub.publish(cp)
  #print "Sleeping..."
  rospy.sleep(time_to_sleep)
  # rate.sleep()
  #print "End of loop!"

