#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random
PI = 3.14159265358971

def callback(msg):
 rospy.set_param('ran',msg.ranges)
 print("message")
 ransacalg()

def listen():
 rospy.init_node('perception_node')
 rospy.Subscriber('/base_scan', LaserScan, callback)
 rospy.spin()

def ransacalg():
 print("ransac called")
 r = rospy.get_param("/ran")
 print(r)
 pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
 line_strip = Marker()
 line_strip.header.frame_id = "base_laser_link"
 line_strip.header.stamp = rospy.Time.now()
 line_strip.ns = "lines"
 line_strip.action = Marker.ADD
 line_strip.pose.orientation.w = 1.0
 line_strip.id = 1
 line_strip.type = Marker.LINE_STRIP
 line_strip.scale.x = 0.1
 line_strip.color.b = 1.0
 line_strip.color.a = 1.0
 if (min(r)<3.0):
  print("ransac started")
  i = 0
  scans_x = list()
  scans_y = list()
  while (i<361):
   if (r[i]<3.0):
    theta = (i/2) - 90
    theta = theta*PI/180
    x = r[i]*math.cos(theta)
    y = r[i]*math.sin(theta)
    scans_x.append(x)
    scans_y.append(y)
   i = i + 1
  l = len(scans_x)
  print(l)
  lim = math.factorial(l)/(math.factorial(2)*math.factorial(l-2))
  print(lim)
  j = 0
  for j in range(100):
   p1x,p2x = random.sample(scans_x,2)
   ix1 = scans_x.index(p1x)
   p1y = scans_y[ix1]
   ix2 = scans_x.index(p2x)
   p2y = scans_y[ix2]
   m = (p1y - p2y)/(p1x - p2x)
   c = p1y - m*p1x
   k = 0
   inl = 0
   ot = 0
   line = np.zeros((lim,4))
   for k in range(len(scans_x)):
    px = scans_x[k]
    py = scans_y[k]
    e = abs(py - m*px - c)/(1 + (m**2))
    if e<=1.0:
     inl = inl + 1
    else: ot = ot + 1
   line[j] = [m,c,inl,ot]
  max_in = 0
  n = 0
  ln = np.transpose(line)
  inc = np.array(ln[2,:])
  for n in range(lim):
   if inc[n]>=max_in:
    max_in = inc[n]
    index = n
  bestline = line[index,:]
  slope = bestline[0]
  intercept = bestline[1]
  sc = 0
  for sc in range(len(scans_x)):
   p_x = scans_x[sc]
   p_y = scans_y[sc]
   d = abs(p_y - slope*p_x - intercept)/(1 + (slope**2))
   if d<=1.0:
    pnts = Point()
    pnts.x = p_x
    pnts.y = p_y
    pnts.z = 0
    line_strip.points.append(pnts)
  print(sc)
  pub.publish(line_strip)

if __name__ == '__main__':
 listen()
