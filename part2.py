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
    self.searching_blue = False
    self.searching_green = True
    self.searching_red = True


  def rotate(self, a, clockwise):
    #Starts a new node

    # Receiveing the user's input
    print("Let's rotate your robot")
    speed = 30
    angle = a

    #Converting from angles to radians
    angular_speed = speed*2*math.pi/360
    relative_angle = angle*2*math.pi/360
    #print(angular_speed, relative_angle)

    #We wont use linear components
    self.twist.linear.x=0
    self.twist.linear.y=0
    self.twist.linear.z=0
    self.twist.angular.x = 0
    self.twist.angular.y = 0
    r = rospy.Rate(50)

    # Checking if our movement is CW or CCW
    if clockwise:
        self.twist.angular.z = -abs(angular_speed)
    else:
        self.twist.angular.z = abs(angular_speed)
    #print(self.twist.angular.z)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    #print(t0)
    i = 0
    while(current_angle < relative_angle):
      i += 1
      #print(i)
      #print(current_angle)
      self.cmd_vel_pub.publish(self.twist)
      t1 = rospy.Time.now().to_sec()
      current_angle = angular_speed*(t1-t0)
      r.sleep()
    print("stopping")
    self.twist = Twist()
    self.cmd_vel_pub.publish(self.twist)
    print("going straight")
    self.twist = Twist()
    self.twist.linear.x = 0.2
    x = 0
    while (x < 30):
      x += 1
      self.cmd_vel_pub.publish(self.twist)
      r.sleep()
    print("turning other way")
    self.twist = Twist()
    if clockwise:
        self.twist.angular.z = abs(angular_speed)
    else:
        self.twist.angular.z = -abs(angular_speed)

    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    #print(t0)
    i = 0
    while(current_angle < relative_angle):
      i += 1
      #print(i)
      #print(current_angle)
      self.cmd_vel_pub.publish(self.twist)
      t1 = rospy.Time.now().to_sec()
      current_angle = angular_speed*(t1-t0)
      r.sleep()

    print("never do it")

    #Forcing our robot to stop
    self.twist.angular.z = 0
    self.cmd_vel_pub.publish(self.twist)
    #rospy.spin()


  def image_callback(self, msg):
    
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # yellow bounds are definitely correct
    lower_yellow = numpy.array([ 15,  0,  0])
    upper_yellow = numpy.array([36, 255, 250])
    lower_red = numpy.array([ 0,  0,  0])
    upper_red = numpy.array([15, 255, 250])

    # green bounds are definitely correct
    lower_green = numpy.array([20,10,10])
    upper_green = numpy.array([70, 255,255])

    # blue bounds are definitely correct
    lower_blue = numpy.array([ 75,  10,  10])
    upper_blue = numpy.array([130, 255, 250])

    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    h, w, d = image.shape
    search_under = 10*h/11
    search_under_top = 10*h/11 + 20
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    
    yellow_mask[0:search_top, 0:w] = 0
    yellow_mask[search_bot:h, 0:w] = 0
    
    red_mask[0:search_under, 0:w] = 0
    red_mask[search_under_top:h, 0:w] = 0
    
    green_mask[0:search_under, 0:w] = 0
    green_mask[search_under_top:h, 0:w] = 0
    
    blue_mask[0:search_under, 0:w] = 0
    blue_mask[search_under_top:h, 0:w] = 0

    
    M_yellow = cv2.moments(yellow_mask)
    M_red = cv2.moments(red_mask)
    M_green = cv2.moments(green_mask)
    M_blue = cv2.moments(blue_mask)
    #print("yellow: ", numpy.sum(yellow_mask))
    print("green: ", numpy.sum(green_mask))
    #print("red: ", numpy.sum(red_mask))
    #print("blue: ", numpy.sum(blue_mask))
   
    if M_blue['m00'] > 0 and self.searching_blue:
        print("Blue found, TURNING")
        cx = int(M_blue['m10']/M_blue['m00'])
        cy = int(M_blue['m01']/M_blue['m00'])
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
        self.rotate(90, True)
        self.searching_blue = False
        self.searching_green = True

    elif numpy.sum(green_mask) > 45000 and self.searching_green:
        print("Green found, TURNING")
        cx = int(M_green['m10']/M_green['m00'])
        cy = int(M_green['m01']/M_green['m00'])
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
        self.rotate(90, False)
        self.searching_green = False
        self.searching_blue = True


    elif M_yellow['m00'] > 0:
      cx = int(M_yellow['m10']/M_yellow['m00'])
      cy = int(M_yellow['m01']/M_yellow['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL
    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
