#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
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
    
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # yellow bounds are definitely correct
    lower_yellow = numpy.array([ 10,  10,  10])
    upper_yellow = numpy.array([30, 255, 250])

    # red bounds right
    lower_red = numpy.array([ 0,  10,  10])
    upper_red = numpy.array([10, 255, 250])


    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    
    h, w, d = image.shape

    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    
    yellow_mask[0:search_top, 0:w] = 0
    yellow_mask[search_bot:h, 0:w] = 0
    red_mask[0:search_top, 0:w] = 0
    red_mask[search_bot:h, 0:w] = 0
    
    M_yellow = cv2.moments(yellow_mask)
    M_red = cv2.moments(red_mask)
    
    # follow yellow path
    if not self.stop and M_yellow['m00'] > 0 and M_red['m00'] == 0:
      cx = int(M_yellow['m10']/M_yellow['m00'])
      cy = int(M_yellow['m01']/M_yellow['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL

    # stopped at red block
    elif self.stop and M_yellow['m00'] > 0:
      cx = int(M_yellow['m10']/M_yellow['m00'])
      cy = int(M_yellow['m01']/M_yellow['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      self.twist.linear.x = 0.6
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      rospy.signal_shutdown('red object ecountered')
      # END CONTROL

    # turn 
    elif not self.stop and M_red['m00'] > 0 and M_yellow['m00'] > 0:
      #print("red encountered")
      cx = int(M_yellow['m10']/M_yellow['m00'])
      cy = int(M_yellow['m01']/M_yellow['m00'])

      rx = int(M_red['m10']/M_red['m00'])
      ry = int(M_red['m01']/M_red['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

      err = cx - w/2
      r_err = rx - w/2
      print(r_err)
   
      # turn right
      if r_err < 0:
        #print("turning right")
        self.twist.linear.x = 0.2
        self.twist.angular.z =  -(float(r_err))/500
        self.cmd_vel_pub.publish(self.twist)
     
      # turn left
      else:
        #print("turning left")
        self.twist.linear.x = 0.2
        self.twist.angular.z =  -(float(r_err))/500
        self.cmd_vel_pub.publish(self.twist)

      

    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
