import rospy
from geometry_msgs.msg import Pose, PoseStamped
from global_planner.srv import *
import time

def send_pose(pose):
	rospy.wait_for_service('goal_channel')
	print('service is ready')
	try:
		channel = rospy.ServiceProxy('goal_channel', Goal)
		
		response = channel(pose)
		if response.status:
			print("success")
		else:
			print("failure")
	except rospy.ServiceException as e:
		print(e)

if __name__=="__main__":
	rospy.init_node('send_goal_node')
	pose = Pose()
	pose.position.x = -4.047599
	pose.position.y = 13.5355

	send_pose(pose)
	
	rospy.spin()