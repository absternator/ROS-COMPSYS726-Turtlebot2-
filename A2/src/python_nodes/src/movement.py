#! /usr/bin/env python
import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import cv2
from sensor_msgs.msg import Image
from takePhoto import TakePhoto
from cv_bridge import CvBridge, CvBridgeError
class MovementNode():
    def __init__(self):
        # Initiliaze Node 
        rospy.init_node('MovementNode', anonymous=False, disable_signals=True)

        # Tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        # Create a publisher which can "talk" to TurtleBot and tell
        # it to move
        self.cmd_vel=rospy.Publisher(
                '/cmd_vel_mux/input/teleop',
                Twist,
                queue_size=10)
	
	# create subscriber oon Bumper!! to detect collisions 
	rospy.Subscriber("/mobile_base/events/bumper",
                            BumperEvent, 
                            self.RobotCollideCallback)

	# create Laser scan subsriber to detect objects
	rospy.Subscriber ("/scan",LaserScan, self.OnLaserScanCallback )

	# create odometry sub to keep track of relative pos to start!!
	rospy.Subscriber('/odom', Odometry, self.odomCallback)
	
        # TurtleBot will stop if we don't keep telling it to move.  
        # How often should we tell it to move? 10 HZ?
        r = rospy.Rate(10);

        # Twist is a datatype for velocity
        self.move_cmd = Twist()

        # Let's go forward at 0.2 m/s at base 
        self.move_cmd.linear.x = 0.2
	
	# if collision has occured !!!
	self.collision = False
	
	# max x pos moved so far 
	self.max_x = 0
	# Image count !! will use to increment name of image!!!
	self.img_count = 0
        # As long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
	    
            # publish the velocity
            self.cmd_vel.publish(self.move_cmd)
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
 
    #callback for odom to keep track of position relative to start!!!
    def odomCallback(self, data):
	# keep track of pos x and orientation z
	x = data.pose.pose.position.x
	z = data.pose.pose.orientation.z
	#store max value of x
	self.max_x = max(x, self.max_x)
	# check if going backwards, then stop !!! moved back .5 metres 
	if (z > .92 or z < -.92) and x-self.max_x < -.2:
	  self.move_cmd.linear.x = -.5
	  rospy.sleep(1)
	  rospy.signal_shutdown(' Going backwards now reached end')

    # callback for bumber subscriber, if collide then do logic!!! for bricks and small objs
    def RobotCollideCallback(self,data):	
	#if collide then rotate robot
        if data.state == 1:
	    self.collision = True
            rospy.loginfo('Collision Bitch!!!! On bumber number: %d',data.bumper)
	    #SLEEP while turning away
	    rospy.sleep(1) # keep true for a second
	    self.collision = False
			
    # callback for laser scan !!! Do stuff depending on direction
    def OnLaserScanCallback(self, data):
	# distance to object we use to indicate turning 
	dist_obj = 1
	# ranges array has 640 values from -30 to 30 degrees, so get left right and front cone
	# if too far or too close(nan), it becomes 10
	front = min(data.ranges[251:401]) # 320 is front
	left =  min(data.ranges[401:550]) # left is actually end of ranges array
	right = min(data.ranges[100:250]) # right is start of array
	# if nan, then make it max value (10)
	if np.isnan(front): front = 10
	if np.isnan(left): left = 10
	if np.isnan(right): right = 10
	# make move function depending on obstacles seen
	self.make_move(front,left,right,dist_obj)

	 # log front value for debug
     #rospy.loginfo ('front: {:.2f}, left: {:.2f}, right: {:.2f}'.format(front,left,right))   

    # make move depending on obstacles in the ways!! 8 possible combinations + 1 bumber collision
    def make_move(self,front, left,right, dist_obj):
	# assume dist to turn is 1 m
	#check for bumper hit case, of hit go back a bit and turn right
	# take photo if encountered object !!! and increment photo name
	if self.collision:
		self.move_cmd.linear.x= -.5
		rospy.sleep(1)
		self.move_cmd.angular.z = 0.2
		self.move_cmd.linear.x= 0
		rospy.sleep(1)
		self.take_photo('photo{}.jpg'.format(self.img_count))
		self.img_count+=1
	# if nothing in front => move straight
	if front > dist_obj and left > dist_obj and right > dist_obj:
		self.move_cmd.linear.x = 0.3
		self.move_cmd.angular.z = 0
	# if in front only => move right
	elif front < dist_obj and left > dist_obj and right > dist_obj:
	 	self.move_cmd.linear.x = 0
		self.move_cmd.angular.z = -0.2
		self.take_photo('photo{}.jpg'.format(self.img_count))
		self.img_count+=1
	# if right only=> move left
	elif front > dist_obj and left > dist_obj and right < dist_obj:
		self.move_cmd.linear.x = 0
		self.move_cmd.angular.z = 0.2
		self.take_photo('photo{}.jpg'.format(self.img_count))
		self.img_count+=1
	# if left only=> move right
	elif front > dist_obj and left < dist_obj and right > dist_obj:
		self.move_cmd.linear.x = 0
		self.move_cmd.angular.z = -0.2
		self.take_photo('photo{}.jpg'.format(self.img_count))
		self.img_count+=1
	# if front and right => move left 
	elif front < dist_obj and left > dist_obj and right < dist_obj:
		self.move_cmd.linear.x = 0
		self.move_cmd.angular.z = 0.2
		self.take_photo('photo{}.jpg'.format(self.img_count))
		self.img_count+=1
	# if front and left => move right
	elif front < dist_obj and left < dist_obj and right > dist_obj:
		self.move_cmd.linear.x = 0
		self.move_cmd.angular.z = -0.2	
		self.take_photo('photo{}.jpg'.format(self.img_count))
		self.img_count+=1
	#if front , left and right => turn right
	else :	
		self.move_cmd.angular.z = -0.2
		self.move_cmd.linear.x = 0
		self.take_photo('photo{}.jpg'.format(self.img_count))
		self.img_count+=1
	# this function takes photo of object when encountered !!
    def take_photo(self,img_title):
	camera = TakePhoto() # class setup with subscriber
   	# Take a photo
    	# Use '_image_title' parameter from command line
    	# Default value is 'photo.jpg'
        img_title = rospy.get_param('~image_title', img_title)
        if camera.take_picture(img_title):
          rospy.loginfo("Saved image " + img_title)
        else:
          rospy.loginfo("No images received")
		
    # shutdown turtlebot2
    def shutdown(self):
        # Stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # A default Twist has linear.x of 0 and angular.z of 0.  
        # So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # Sleep just makes sure TurtleBot receives the stop command
        # prior to shutting
        # down the script
        rospy.sleep(1)
    
if __name__ == '__main__':
    print('I am starting')
    MovementNode()
	# to keep going until shutdown
    rospy.spin()
    
