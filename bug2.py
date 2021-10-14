#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
PI = 3.14159265358971

def listen():
 rospy.init_node('bug_2_implementation')
 rospy.Subscriber('/base_scan',LaserScan,callback)
 rospy.Subscriber('/base_pose_ground_truth',Odometry,call_back)
 rospy.spin()

def callback(msg):
 rospy.set_param('ran',msg.ranges)
 
def call_back(data):
 call_back.px = data.pose.pose.position.x
 call_back.py = data.pose.pose.position.y
 call_back.oz = data.pose.pose.orientation.z
 call_back.robotAngle = 2*np.arcsin(call_back.oz)
 call_back.d = math.sqrt((goalPos.pose.pose.position.y - call_back.py)**2 + (goalPos.pose.pose.position.x - call_back.px)**2)
 bug2algo()

def wall_in_front():
 global front
 r = rospy.get_param('/ran')
 i = 0
 c = 0
 if (min(r)<3):
  for i in range(361):
   if r[i]<0.6:
    theta = (i/2) - 90
    if (theta>=-45) or (theta<=45):
     c = c + 1
  if c>0:
   return True
  else:
   return False 
 else:
  return False

def wall_on_left():
 global front,left
 r = rospy.get_param('/ran')
 i = 0
 c = 0
 if (min(r)<3):
  for i in range(361):
   if r[i]<1.5:
    theta = (i/2) - 90
    if (theta>45):
     c = c + 1
  if c>0:
   return True
  else:
   return False
 else:
  return False

def robot_on_goal_line():
 m1 = (goalPos.pose.pose.position.y - startPos.pose.pose.position.y)/(goalPos.pose.pose.position.x - startPos.pose.pose.position.x)
 c = startPos.pose.pose.position.y - (m1*startPos.pose.pose.position.x)
 line = call_back.py - (m1*call_back.px) - c
 if line==0:
  return True
 else:
  return False

def rotation(goalAngle):
 if (robot_on_goal_line()==True):
  return min(goalAngle,1)
 else:
  return -1

def wallFollowRotation():
 if (wall_in_front()==True):
  return -30 * PI / 180
 if (wall_on_left()==True):
  return 0
 else:
  return 65 * PI / 180
  
def bug2algo():
 try:
  pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
  #Pos = rospy.Subscriber('/base_pose_ground_truth',Odometry,call_back)
  minDist = 0.5
  global atGoal
  global state
  if (atGoal==False):
   m = math.atan((goalPos.pose.pose.position.y - startPos.pose.pose.position.y)/(goalPos.pose.pose.position.x - startPos.pose.pose.position.x))
   goalAngle = m - call_back.robotAngle
   if (call_back.d<=minDist):
    vel.linear.x = 0
    vel.angular.z = 0
    pub.publish(vel)
    atGoal = True
    state = "DONE"
   else:
    if (state=="GOALSEEK"):
     if (robot_on_goal_line()==True and (wall_in_front()==True or wall_on_left()==True)):
      vel.linear.x = 0
      r = call_back.robotAngle
      if (r!=goalAngle):
       ang = min(goalAngle,0.1)
       vel.angular.z = ang
       r = r - ang
     elif (wall_in_front()==True or wall_on_left()==True):
      vel.linear.x = 0
      vel.angular.z = -0.2
      state = "WALLFOLLOW"
     else:
      vel.linear.x = 1
      vel.angular.z = 0
    else:
     if (robot_on_goal_line()==True and (wall_in_front()==True or wall_on_left()==True)):
      state = "GOALSEEK"
     elif (wall_in_front()==False and wall_on_left()==True):
      vel.linear.x = 1
      vel.angular.z = 0
      state = "WALLFOLLOW"
     else:
      vel.linear.x = 0
      ang = wallFollowRotation()
      vel.angular.z = ang
      state = "WALLFOLLOW"
   print(state) 
   pub.publish(vel) 
 except rospy.ROSInterruptException: pass

if __name__ == '__main__':
 try:
  startPos = Odometry()
  startPos.pose.pose.position.x = -8.0
  startPos.pose.pose.position.y = -2.0
  goalPos = Odometry()
  goalPos.pose.pose.position.x = 4.5
  goalPos.pose.pose.position.y = 9.0
  atGoal = False
  state = "GOALSEEK"
  vel = Twist()
  listen()
 except rospy.ROSInterruptException: pass
 
