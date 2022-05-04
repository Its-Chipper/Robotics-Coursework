
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
        #subscribers and publishers
        rospy.init_node('chatter')
        self.odometry = Odometry()
        self.laserScan = LaserScan()
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)
        self.Odom_sub = rospy.Subscriber('/odom', Odometry, self.Odom_cb)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.heading = 0
        
        #global variables intial values
        self.Turnning = False
        self.left_sense = 1000
        self.right_sense = 1000
        self.run_angleCheck = False
        self.red_turn = False
        self.junction = False
        self.junction_right = False
        self.junction_pos = 0
        rospy.Timer(rospy.Duration(1.0/50), self.runner)
        
        
    #loop to run the turning function
    def runner(self, event):
        if self.run_angleCheck:
            self.Check_walls()
            self.run_angleCheck = False

    #laser call back
    def laser_cb(self, laser_msg):
        #if the robot is turning keeps the laser scan up to date for the global variable
        if self.Turnning:
            self.laserScan = laser_msg
        
        #if there is a junction either side of the robot
        elif self.junction:
            #turning when it reaches the correct point for the junction
            if laser_msg.ranges[320] < self.junction_pos:
                #reset the sense values
                self.left_sense = 1000
                self.right_sense = 1000
                #turns unless there is blue or green infront of it
                if self.blue_pos == None and self.green_pos == None:
                    self.Turn_X(self.junction_right, pi/2)
                self.junction = False
            else:
                #basic moving forward code
                self.heading = 0
                t = Twist()
                t.angular.z = 5 * (laser_msg.ranges[310] - laser_msg.ranges[330])
                
                t.linear.x = 0.2
                self.publisher.publish(t)
        
        elif self.red_turn:
            if laser_msg.ranges[320] < self.junction_pos:
                #turn away from red
                self.left_sense = 1000
                self.right_sense = 1000
                self.run_angleCheck = True
                self.Turnning = True
                self.red_turn = False
            else:
                #basic moving forward code
                self.heading = 0
                t = Twist()
                t.angular.z = 5 * (laser_msg.ranges[310] - laser_msg.ranges[330])
                
                t.linear.x = 0.2
                self.publisher.publish(t)
        
        else:        
            #check if there is a wall infront    
            if laser_msg.ranges[320] < 0.7:
                self.run_angleCheck = True
                self.Turnning = True
                self.left_sense = 1000
                self.right_sense = 1000
            else:
                #check for left junction
                if(self.left_sense < 1.1 and laser_msg.ranges[-1] > 1.5 and laser_msg.ranges[320] > 2):
                    self.junction = True
                    self.junction_right = False
                    self.junction_pos = floor(laser_msg.ranges[320]) - 0.3
                #check for right junction
                if(self.right_sense < 1.1 and laser_msg.ranges[0] > 1.5 and laser_msg.ranges[320] > 2):
                    self.junction = True
                    self.junction_right = True
                    self.junction_pos = floor(laser_msg.ranges[320]) - 0.3
                #check if there is red infront
                if(self.red_pos != None):
                    self.red_turn = True
                    self.junction_pos = floor(laser_msg.ranges[320]) + 0.7
                
                #updates the sense values
                self.left_sense = laser_msg.ranges[-1]
                self.right_sense = laser_msg.ranges[0]
                
                #basic moving forward code
                self.heading = 0
                t = Twist()
                t.angular.z = 5 * (laser_msg.ranges[315] - laser_msg.ranges[325])
                
                print(laser_msg.ranges[310] , ":" , laser_msg.ranges[330])
                
                t.linear.x = 0.2
                self.publisher.publish(t)
            
    
    #odmetry call back updates the global odom value
    def Odom_cb(self, Odom):
        self.odometry = Odom
    
    def image_callback(self, data): 
        #converting image to hsv
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #values for the 3 diffrent colours
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
        
        #mask values
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

        #moments for each of the colour and the position of mean of each colour compaired to the center of the camera
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
    
    def Check_walls(self):
        #sets values for the walls and the distances
        clear_walls = [0,0]
        distances = [0,0]
        
        #turns to face the right
        self.Turn_X(True, pi/2)
        #checks if there is a wall infront
        if(self.laserScan.ranges[320] > 1 and (self.red_pos == None)):
            #if no wall give a weight of 1
            clear_walls[0] = 1
            distances[0] = self.laserScan.ranges[320]
            #add 1 if there is blue and 5 for green as weights
            if(self.blue_pos != None):
                clear_walls[0] += 1
            if(self.green_pos != None):
                clear_walls[0] += 5
                
        #turns to face the left
        self.Turn_X(False, pi)
        if(self.laserScan.ranges[320] > 1 and (self.red_pos == None)):
            clear_walls[1] = 1
            distances[1] = self.laserScan.ranges[320]
            if(self.blue_pos != None):
                clear_walls[1] += 1
            if(self.green_pos != None):
                clear_walls[1] += 5
        
        print(distances)
        
        #if left is wieghted high enough move left
        if(clear_walls[1] >= clear_walls[0] and clear_walls[1] > 0 and not distances[1] < distances[0] - 4):
            pass
        #if right is clear and left isn't move right
        elif(clear_walls[0] >= 1):
            self.Turn_X(True, pi)
        #otherwise spin to go back
        else:
            print("Test")
            self.Turn_X(False, pi/2)
                  
        self.Turnning = False
    
    def Turn_X(self, Dir, ang):
        #sets the values for the initail position and the current position in the turn
        spin = self.odometry.pose.pose.orientation
        quaternion_array = [spin.x, spin.y, spin.z, spin.w]
        role, pitch, OrientationHold = euler_from_quaternion(quaternion_array)
        role, pitch, CurrentOrientation = euler_from_quaternion(quaternion_array)
        #while the tunr isn't complete
        while (abs(OrientationHold - CurrentOrientation) < (ang) or abs(OrientationHold - CurrentOrientation) > (2*pi - ang)) and (round(abs(OrientationHold - CurrentOrientation) - ang, 2) != 0):
            t = Twist()
            angle_change = 0
            
            #turn is propotional to the amount of turn left
            if(abs(ang - abs(OrientationHold - CurrentOrientation)) > abs((2*pi - ang - abs(OrientationHold - CurrentOrientation)))):
                angle_change = abs(2*pi - ang - abs(OrientationHold - CurrentOrientation))/2
            else:
                angle_change = abs(ang - abs(OrientationHold - CurrentOrientation))/2
                
            #turn speed is always atleast 0.1
            if Dir:
                t.angular.z = -0.1 - angle_change
            else:
                t.angular.z = 0.1 + angle_change
            self.publisher.publish(t)
            
            #resets the current position
            spin = self.odometry.pose.pose.orientation
            quaternion_array = [spin.x, spin.y, spin.z, spin.w]
            role, pitch, CurrentOrientation = euler_from_quaternion(quaternion_array)
    def run(self):
        rospy.spin()

c = Chatter()
c.run()