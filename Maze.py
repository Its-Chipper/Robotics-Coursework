
from cmath import pi
from math import floor
from turtle import distance
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import cv2 
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Chatter:


    def __init__(self):
        rospy.init_node('chatter')
        self.odometry = Odometry()
        self.laserScan = LaserScan()
        self.bridge = CvBridge()
        self.Turnning = False
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)
        self.Odom_sub = rospy.Subscriber('/odom', Odometry, self.Odom_cb)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.heading = 0
        cv2.namedWindow("window", 1)
        
        self.blue_pos = None
        
        self.left_sense = 1000
        self.right_sense = 1000
        self.Start_bool = False
        self.run_angleCheck = False
        self.red_turn = False
        self.junction = False
        self.junction_right = False
        self.junction_pos = 0
        rospy.Timer(rospy.Duration(1.0/50), self.runner)
        
        
            
    def runner(self, event):
        if self.run_angleCheck:
            self.Check_walls()
            self.run_angleCheck = False
        if self.Start_bool:
            self.Start_Spin()

    def laser_cb(self, laser_msg):
        if self.Turnning:
            self.laserScan = laser_msg
        
        elif self.junction:
            if laser_msg.ranges[320] < self.junction_pos:
                self.left_sense = 1000
                self.right_sense = 1000
                if self.blue_pos == None and self.green_pos == None:
                    self.Turn_X(self.junction_right, pi/2)
                self.junction = False
            else:
                self.heading = 0
                t = Twist()
                t.angular.z = 5 * (laser_msg.ranges[310] - laser_msg.ranges[330])
                
                t.linear.x = 0.2
                self.publisher.publish(t)
        
        elif self.red_turn:
            if laser_msg.ranges[320] < self.junction_pos:
                self.left_sense = 1000
                self.right_sense = 1000
                self.run_angleCheck = True
                self.Turnning = True
                self.red_turn = False
            else:
                self.heading = 0
                t = Twist()
                t.angular.z = 5 * (laser_msg.ranges[310] - laser_msg.ranges[330])
                
                t.linear.x = 0.2
                self.publisher.publish(t)
        
        else:            
            if laser_msg.ranges[320] < 0.7:
                self.run_angleCheck = True
                self.Turnning = True
                self.left_sense = 1000
                self.right_sense = 1000
            else:
                if(self.left_sense < 1.1 and laser_msg.ranges[-1] > 1.5 and laser_msg.ranges[320] > 2):
                    self.junction = True
                    self.junction_right = False
                    self.junction_pos = floor(laser_msg.ranges[320]) - 0.3
                if(self.right_sense < 1.1 and laser_msg.ranges[0] > 1.5 and laser_msg.ranges[320] > 2):
                    self.junction = True
                    self.junction_right = True
                    self.junction_pos = floor(laser_msg.ranges[320]) - 0.3
                if(self.red_pos != None):
                    self.red_turn = True
                    self.junction_pos = floor(laser_msg.ranges[320]) + 0.7
                self.left_sense = laser_msg.ranges[-1]
                self.right_sense = laser_msg.ranges[0]
                self.heading = 0
                t = Twist()
                t.angular.z = 5 * (laser_msg.ranges[315] - laser_msg.ranges[325])
                
                print(laser_msg.ranges[310] , ":" , laser_msg.ranges[330])
                
                t.linear.x = 0.2
                self.publisher.publish(t)
            
    
    def Odom_cb(self, Odom):
        self.odometry = Odom
    
    def image_callback(self, data): 
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([80, 50, 50])
        upper_blue = np.array([130, 255, 255])
        bluemask = cv2.inRange(hsv, lower_blue, upper_blue)

        lower_red = np.array([0, 50, 50])
        upper_red = np.array([20, 255, 255])
        redmask = cv2.inRange(hsv, lower_red, upper_red)
        
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([100, 255, 255])
        greenmask = cv2.inRange(hsv, lower_green, upper_green)

        h, w, d = image.shape
        
        
        red_top = 7*(h/8) - 20
        red_bot = h
        redmask[0:red_top, 0:w] = 0
        redmask[red_bot:h, 0:w] = 0
        
        kernel = np.ones((10, 10), np.uint8)
        redmask = cv2.erode(redmask, kernel)

        blue_top = (h/2) - 20
        blue_bot = (h/2) + 20
        bluemask[0:blue_top, 0:w] = 0
        bluemask[blue_bot:h, 0:w] = 0
    
        green_top = 0
        green_bot = h
        greenmask[0:green_top, 0:w] = 0
        greenmask[green_bot:h, 0:w] = 0

        B = cv2.moments(bluemask)
        if B['m00'] > 0:
            bx = int(B['m10']/B['m00'])
            by = int(B['m01']/B['m00'])
            cv2.circle(image, (bx, by), 20, (0, 0, 255), -1)
            
            self.blue_pos = float(((w/2)- bx))
        else:
            self.blue_pos = None
        
        G = cv2.moments(greenmask)
        if G['m00'] > 0:
            gx = int(G['m10']/G['m00'])
            gy = int(G['m01']/G['m00'])
            cv2.circle(image, (gx, gy), 20, (0, 0, 255), -1)
            
            self.green_pos = float(((w/2)- gx))
        else:
            self.green_pos = None


        R = cv2.moments(redmask)

        if R['m00'] > 0:
            rx = int(R['m10']/R['m00'])
            ry = int(R['m01']/R['m00'])
            cv2.circle(image, (rx, ry), 20, (0, 0, 255), -1)
            self.red_pos = float(((w/2)- rx))
        else:
            self.red_pos = None
            

        cv2.imshow("window", image)
        cv2.waitKey(1)
    
    def Start_Spin(self):
        for i in range(4):
            self.Turn_X(True, pi/2)
            if(self.blue_pos != None and self.blue_pos < 0):
                break
        self.Start_bool = False
    
    def Check_walls(self):
        clear_walls = [0,0]
        distances = [0,0]
        
        self.Turn_X(True, pi/2)
        if(self.laserScan.ranges[320] > 1 and (self.red_pos == None)):
            clear_walls[0] = 1
            distances[0] = self.laserScan.ranges[320]
            if(self.blue_pos != None):
                clear_walls[0] += 1
            if(self.green_pos != None):
                clear_walls[0] += 5
        self.Turn_X(False, pi)
        if(self.laserScan.ranges[320] > 1 and (self.red_pos == None)):
            clear_walls[1] = 1
            distances[1] = self.laserScan.ranges[320]
            if(self.blue_pos != None):
                clear_walls[1] += 1
            if(self.green_pos != None):
                clear_walls[1] += 5
        
        print(distances)
        
        if(clear_walls[1] >= clear_walls[0] and clear_walls[1] > 0 and not distances[1] < distances[0] - 4):
            pass
        elif(clear_walls[0] >= 1):
            self.Turn_X(True, pi)
        else:
            print("Test")
            self.Turn_X(False, pi/2)
                  
        print(clear_walls)
        self.Turnning = False
    
    def Turn_X(self, Dir, ang):
        spin = self.odometry.pose.pose.orientation
        quaternion_array = [spin.x, spin.y, spin.z, spin.w]
        role, pitch, OrientationHold = euler_from_quaternion(quaternion_array)
        role, pitch, CurrentOrientation = euler_from_quaternion(quaternion_array)
        while (abs(OrientationHold - CurrentOrientation) < (ang) or abs(OrientationHold - CurrentOrientation) > (2*pi - ang)) and (round(abs(OrientationHold - CurrentOrientation) - ang, 2) != 0):
            t = Twist()
            angle_change = 0
            if(abs(ang - abs(OrientationHold - CurrentOrientation)) > abs((2*pi - ang - abs(OrientationHold - CurrentOrientation)))):
                angle_change = abs(2*pi - ang - abs(OrientationHold - CurrentOrientation))/2
            else:
                angle_change = abs(ang - abs(OrientationHold - CurrentOrientation))/2
                
            if Dir:
                t.angular.z = -0.1 - angle_change
            else:
                t.angular.z = 0.1 + angle_change
            if self.Start_bool:
                if self.blue_pos != None:
                    t.angular.z = self.blue_pos/300
                    if(abs(self.blue_pos) < 1):
                        self.Start_bool = False
                        break
            self.publisher.publish(t)
            
            spin = self.odometry.pose.pose.orientation
            quaternion_array = [spin.x, spin.y, spin.z, spin.w]
            role, pitch, CurrentOrientation = euler_from_quaternion(quaternion_array)
    def run(self):
        rospy.spin()

c = Chatter()
c.run()