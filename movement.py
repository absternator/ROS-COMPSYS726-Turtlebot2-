#! /usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan
class MovementNode():
    def __init__(self):
        # Initiliaze
        rospy.init_node('MovementNode', anonymous=False)

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
	# create subscriber on bumper also  
	rospy.Subscriber("/mobile_base/events/bumper",
                            BumperEvent, 
                            self.RobotCollideCallback)

	# create Laser scan subsriber
	rospy.Subscriber ("/scan",LaserScan, self.OnLaserScanCallback )
        
        # TurtleBot will stop if we don't keep telling it to move.  
        # How often should we tell it to move? 10 HZ?
        r = rospy.Rate(10);

        # Twist is a datatype for velocity
        self.move_cmd = Twist()

        # Let's go forward at 0.2 m/s
        self.move_cmd.linear.x = 0.2
	

        # As long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            # publish the velocity
            self.cmd_vel.publish(self.move_cmd)
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
    # callback for bumber subscriber
    def RobotCollideCallback(self,data):	
	#if collide then rotate robot
        if data.state == 1:
            rospy.loginfo('Collision Bitch!!!! On bumber number: %d',data.bumper)
    # callback for laser scan !!! 
    def OnLaserScanCallback(self, data):
	# distance to object 
	dist_obj = 1
	# Assuming the middle point is directly in front of the robot
        mid_point = len(data.ranges)/2
	# if > 1 metre then go straight, else turn away until not 1m
	if data.ranges[mid_point] > dist_obj:
	    self.move_cmd.linear.x = 0.2
	    self.move_cmd.angular.z = 0
	else:
	    self.move_cmd.linear.x = 0
	    self.move_cmd.angular.z = 0.5
	
        rospy.loginfo (" Front value is %f", data.ranges [mid_point])   

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

    rospy.spin()
    
