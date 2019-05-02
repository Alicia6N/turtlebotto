#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import pygame, sys
from pygame.locals import *
import numpy as np
from sensor_msgs.msg import LaserScan
import math
class Camera:
  def __init__(self):
    self.odom_topic = "/odom"
    self.vel_topic = "/mobile_base/commands/velocity"
    self.bridge = CvBridge()
    self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
    self.cmd_vel = rospy.Publisher(self.vel_topic, Twist, queue_size=1)
    
    self.lastLinearVelocityX = None
    self.lastAngularVelocityZ = None

  def odom_callback(self, data):
    self.lastLinearVelocityX = data.twist.twist.linear.x
    self.lastAngularVelocityZ = data.twist.twist.angular.z

if __name__ == '__main__':
    pygame.init()
    pygame.display.set_mode((100,100))
    camera = Camera()
    rospy.init_node('capture_data', anonymous=True)

    move_cmd = Twist()
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    try:
      while True:
        for event in pygame.event.get():
          if event.type == QUIT:
            sys.exit()
          if event.type == KEYDOWN and event.key == 275: # left
            move_cmd.angular.z = -0.2
            move_cmd.linear.x = 0.0
          elif event.type == KEYDOWN and event.key == 276: # right
            move_cmd.angular.z = 0.2
            move_cmd.linear.x = 0.0
          elif event.type == KEYDOWN and event.key == 273: # forward
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0.0
          elif event.type == KEYDOWN and event.key == 274: # stop
            if move_cmd.linear.x > 0:
              move_cmd.linear.x = 0.0
              move_cmd.angular.z = 0.0
		
        pygame.event.pump()
        camera.cmd_vel.publish(move_cmd)
    except KeyboardInterrupt:
      print("Shutting down")
    cv.destroyAllWindows()
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    camera.cmd_vel.publish(move_cmd)
