#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from smach import State,StateMachine
from time import sleep
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

    self.encountered_red = False

  def image_callback(self, msg):
    try:
      image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
      lower_yellow = numpy.array([ 10,  10,  10])
      upper_yellow = numpy.array([30, 255, 250])
      lower_green = numpy.array([50, 10, 10])
      upper_green = numpy.array([80, 255, 250])
      lower_blue = numpy.array([110, 10, 10])
      upper_blue = numpy.array([130, 255, 250])
      lower_red = numpy.array([0,  10,  10])
      upper_red = numpy.array([10, 255, 250])


      mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
      mask_green = cv2.inRange(hsv, lower_green, upper_green)
      mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
      mask_red = cv2.inRange(hsv, lower_red, upper_red)

      h, w, d = image.shape
      search_top = 3*h/4
      search_bot = 3*h/4 + 20

      mask_yellow[0:search_top, 0:w] = 0
      mask_yellow[search_bot:h, 0:w] = 0
      mask_green[0:search_top, 0:w] = 0
      mask_green[search_bot:h, 0:w] = 0
      mask_blue[0:search_top, 0:w] = 0
      mask_blue[search_bot:h, 0:w] = 0
      mask_red[0:search_top, 0:w] = 0
      mask_red[search_bot:h, 0:w] = 0

      Y = cv2.moments(mask_yellow)
      G = cv2.moments(mask_green)
      B = cv2.moments(mask_blue)
      R = cv2.moments(mask_red)

      if Y['m00'] > 0 and G['m00'] == 0 and B['m00'] == 0 and R['m00'] == 0 and not self.encountered_red:
        cx = int(Y['m10']/Y['m00'])
        cy = int(Y['m01']/Y['m00'])
        cv2.circle(image, (cx, cy), 20, (255,255,255), -1)
        # BEGIN CONTROL
        err = cx - w/2
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 120
        self.cmd_vel_pub.publish(self.twist)
        # END CONTROL
      elif Y['m00'] > 0 and G['m00'] == 0 and B['m00'] == 0 and R['m00'] == 0 and self.encountered_red:
        print 'here'
        cx = int(Y['m10']/Y['m00'])
        cy = int(Y['m01']/Y['m00'])
        cv2.circle(image, (cx, cy), 20, (255,255,255), -1)
        # BEGIN CONTROL
        err = cx - w/2
        self.twist.linear.x = 0.7
        self.twist.angular.z = -float(err) / 120
        self.cmd_vel_pub.publish(self.twist)
        rospy.signal_shutdown('stopping at red block')
      elif G['m00'] > 0 and B['m00'] == 0:
        print 'encountered a green object'
        cx = int(Y['m10']/Y['m00'])
        cy = int(Y['m01']/Y['m00'])
        cv2.circle(image, (cx, cy), 20, (255,255,255), -1)
        # BEGIN CONTROL
        err = cx - w/2
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 120 + (118*math.pi)/180
        self.cmd_vel_pub.publish(self.twist)
      elif G['m00'] == 0 and B['m00'] > 0:
        print 'encounterd a blue object'
        cx = int(Y['m10']/Y['m00'])
        cy = int(Y['m01']/Y['m00'])
        cv2.circle(image, (cx, cy), 20, (255,255,255), -1)
        # BEGIN CONTROL
        err = cx - w/2
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 120 - (118*math.pi)/180
        self.cmd_vel_pub.publish(self.twist)

      elif R['m00'] > 0:
        print 'encounterd a red object'
        cx = int(Y['m10']/Y['m00'])
        cy = int(Y['m01']/Y['m00'])
        cv2.circle(image, (cx, cy), 20, (255,255,255), -1)
        # BEGIN CONTROL
        err = cx - w/2
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 120
        self.cmd_vel_pub.publish(self.twist)
        self.encountered_red = True

    except:
      print 'terminated'


    cv2.imshow("window", image)
    cv2.waitKey(3)



rospy.init_node('follower', disable_signals=True)
try:
  follower = Follower()
  rospy.spin()
except:
  print 'terminated'
# END ALL
