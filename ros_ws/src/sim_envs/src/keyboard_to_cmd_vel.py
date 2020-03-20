#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

speedpub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

def callback(data):
	speedpub.publish(data)

if __name__ == '__main__':
	rospy.init_node('to_cmd_vel')
	rospy.Subscriber('/cmd_vel_mux/input/teleop', Twist, callback)
	rospy.spin()
