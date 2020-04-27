#! /usr/bin/env python

import rospy
# import roslib; roslib.load_manifest('gazebo')
from gazebo_msgs.srv import *
from geometry_msgs.msg import Pose


def gms_client(model_name,relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name,relative_entity_name)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == '__main__':
	rospy.init_node('publish_object_poses')
	bluepb = rospy.Publisher('/blue_cube', Pose, queue_size=1)
	greenpb = rospy.Publisher('/green_cube', Pose, queue_size=1)
	redpb = rospy.Publisher('/red_cube', Pose, queue_size=1)
	yellowpb = rospy.Publisher('/yellow_cube', Pose, queue_size=1)
	brownpb = rospy.Publisher('/brown_cube', Pose, queue_size=1)
	violetpb = rospy.Publisher('/violet_cube', Pose, queue_size=1)
	blackpb = rospy.Publisher('/black_cube', Pose, queue_size=1)

	rate = rospy.Rate(2)

	while not rospy.is_shutdown():
		blue = gms_client('demo_cube', 'fetch')
		green = gms_client('green_cube', 'fetch')
		red = gms_client('red_cube', 'fetch')
		yellow = gms_client('yellow_cube', 'fetch')
		brown = gms_client('brown_cube', 'fetch')
		violet = gms_client('violet_cube', 'fetch')
		black = gms_client('black_cube', 'fetch')

		bluepb.publish(blue.pose)
		greenpb.publish(green.pose)
		redpb.publish(red.pose)
		yellowpb.publish(yellow.pose)
		brownpb.publish(brown.pose)
		violetpb.publish(violet.pose)
		blackpb.publish(black.pose)

		rate.sleep()


