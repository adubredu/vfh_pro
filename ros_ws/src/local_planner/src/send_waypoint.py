#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from math import sqrt, pow

class transmit_routine:
	def __init__(self):
		rospy.init_node('transmit_routine')
		self.robotX = 0; self.robotY = 0;
		self.pub = rospy.Publisher("/way_point", PointStamped, queue_size=1)
		rospy.Subscriber("/state_estimation", Odometry, self.handleOdometry)
		self.tolerance = 1.0
		
		self.x = [0.0094, -0.3799, -0.8583, -2.1060, -2.7487, -3.2521, -3.11226, 
				-2.88039, -1.3556, -0.1411, 1.4428, 1.5756, 1.16816, -0.08995, -1.0518999, -2.71745,
					-3.90927, -4.09326, -4.047599, -3.20676, -1.876729, -0.2724, 1.308188, 
					1.42493, 1.6086, 1.39013, 1.204932, 1.12451, 1.173966, -0.042788]
		self.y = [1.0174, 2.2003, 2.5957, 2.7859, 3.4010, 4.7677, 5.9639, 7.2639, 
				7.6046, 7.5410, 7.6446, 9.062469, 11.48215, 11.48655, 10.7707, 10.69302,
					11.052, 12.0904, 13.5355, 13.7500, 13.5488, 13.22313, 11.9623, 
					10.5411, 8.526895, 6.74429, 4.8596, 3.3084788, 1.489633, -1.05975]
		self.count = 0
		self.size = 
		self.transmit_routine()
		rospy.spin()


	def handleOdometry(self,odom):
		self.robotX = odom.pose.pose.position.x 
		self.robotY = odom.pose.pose.position.y 


	def at_goal(self,goalX,goalY):
		distance = sqrt(pow((self.robotX - goalX),2) + pow((self.robotY - goalY),2));
		if (distance < self.tolerance):
			return True;

		return False;

	def transmit_routine(self):
		while not rospy.is_shutdown():
			wp = PointStamped()
			wp.header.frame_id="odom"
			x = self.x[self.count%self.size]
			y = self.y[self.count%self.size]
			wp.point.x = x
			wp.point.y = y

			self.pub.publish(wp)
			if self.at_goal(x,y):
				self.count+=1




if __name__=='__main__':
    try:
        send_goal = transmit_routine()

    except rospy.ROSInterruptException: pass