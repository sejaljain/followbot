#!/usr/bin/env python
# BEGIN ALL
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import math


class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.twist = Twist()

    self.stop = False

  def image_callback(self, msg):

    image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # yellow bounds are definitely correct
    lower_yellow = numpy.array([10, 10, 10])
    upper_yellow = numpy.array([30, 255, 250])

    # red bounds right
    lower_red = numpy.array([0, 10, 10])
    upper_red = numpy.array([10, 255, 250])

    # green bounds are definitely correct
    lower_green = numpy.array([50, 10, 10])
    upper_green = numpy.array([80, 255, 255])

    # blue bounds are definitely correct
    lower_blue = numpy.array([110, 10, 10])
    upper_blue = numpy.array([130, 255, 250])

    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    h, w, d = image.shape

    search_top = 3 * h / 4
    search_bot = 3 * h / 4 + 20

    yellow_mask[0:search_top, 0:w] = 0
    yellow_mask[search_bot:h, 0:w] = 0
    red_mask[0:search_top, 0:w] = 0
    red_mask[search_bot:h, 0:w] = 0
    green_mask[0:search_top, 0:w] = 0
    green_mask[search_bot:h, 0:w] = 0
    blue_mask[0:search_top, 0:w] = 0
    blue_mask[search_bot:h, 0:w] = 0

    M_yellow = cv2.moments(yellow_mask)
    M_red = cv2.moments(red_mask)
    M_green = cv2.moments(green_mask)
    M_blue = cv2.moments(blue_mask)

    # stop at the red block, which is now out of view (right below the robot)
    if self.stop and (M_yellow['m00'] > 0) and (M_red['m00'] == 0):
      cx = int(M_yellow['m10'] / M_yellow['m00'])
      cy = int(M_yellow['m01'] / M_yellow['m00'])
      cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
      # BEGIN CONTROL
      err = cx - w / 2
      self.twist.linear.x = 0.8
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      rospy.signal_shutdown('red object ecountered')
      # END CONTROL

    # have seen the red block, but need to move closer to it
    elif self.stop and (M_yellow['m00'] > 0) and (M_red['m00'] != 0):
      cx = int(M_yellow['m10'] / M_yellow['m00'])
      cy = int(M_yellow['m01'] / M_yellow['m00'])
      cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
      # BEGIN CONTROL
      err = cx - w / 2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL

    # follow yellow path, have not seen any red yet
    elif (M_yellow['m00'] > 0) and (M_blue['m00'] == 0) and (M_green['m00'] == 0) and (M_red['m00'] == 0):
      cx = int(M_yellow['m10'] / M_yellow['m00'])
      cy = int(M_yellow['m01'] / M_yellow['m00'])
      cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
      # BEGIN CONTROL
      err = cx - w / 2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL

    # turn right at blue, have not seen any red yet
    elif (M_blue['m00'] > 0) and (M_green['m00'] == 0) and (M_red['m00'] == 0):
      print('sensed a blue object, turning right')
      cx = int(M_yellow['m10'] / M_yellow['m00'])
      cy = int(M_yellow['m01'] / M_yellow['m00'])
      cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
      # BEGIN CONTROL
      err = cx - w / 2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100 - (2 * math.pi) / 3
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL

    # turn left at green, have not seen any red yet
    elif (M_green['m00'] > 0) and (M_blue['m00'] == 0) and (M_red['m00'] == 0):
      print('sensed a green object, turning left')
      cx = int(M_yellow['m10'] / M_yellow['m00'])
      cy = int(M_yellow['m01'] / M_yellow['m00'])
      cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
      # BEGIN CONTROL
      err = cx - w / 2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100 + (2 * math.pi) / 3
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL

    # detect the red, send the signal to begin to stop
    elif (M_red['m00'] > 0) and (M_blue['m00'] == 0) and (M_green['m00'] == 0):
      print("sensed a red object, stopping")
      cx = int(M_yellow['m10'] / M_yellow['m00'])
      cy = int(M_yellow['m01'] / M_yellow['m00'])
      cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
      # BEGIN CONTROL
      err = cx - w / 2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      self.stop = True

    cv2.imshow("window", image)
    cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
