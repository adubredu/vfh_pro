#! /usr/bin/env python

import rospy
import numpy as np 
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
import time

class Slim:
	def __init__(self):
		rospy.init_node('slim_node')
		self.particle_pub = rospy.Publisher('/particles', PoseArray, queue_size=1)
		self.goto_pub = rospy.Publisher('/go_to', String, queue_size=1)
		self.top_left_x = -8.6
		self.top_left_y = 5.5
		self.bottom_right_x = 8.7
		self.bottom_right_y = -4.7
		self.num_particles = 100

		self.commonsense = {
		'coke': {'kitchen':0.3, 'diningroom':0.3, 'gym':0.05, 'bathroom':0.0, 
		'bedroom':0.15, 'livingroom':0.20},

		'drill': {'kitchen':0.0, 'diningroom':0.0, 'gym':0.0, 'bathroom':0.3, 
		'bedroom':0.6, 'livingroom':0.1},

		'saucepan': {'kitchen':0.6, 'diningroom':0.3, 'gym':0.0, 'bathroom':0.0, 
		'bedroom':0.0, 'livingroom':0.1},

		'beer': {'kitchen':0.3, 'diningroom':0.3, 'gym':0.05, 'bathroom':0.0, 
		'bedroom':0.05, 'livingroom':0.30},
		}

		self.places = {
		'kitchen':{'x':7.18, 'y':-3.15}, 'diningroom':{'x':4.67, 'y':0.98},
		'gym':{'x':2.08, 'y':4.18},'bedroom':{'x':-5.90, 'y':-0.18}, 
		'livingroom':{'x':1.72, 'y':-2.84}, 'bathroom':{'x':-7.37 , 'y':-3.42 }
		}

		rospy.Subscriber('/search_for', String, self.target_handler)

		# while not rospy.is_shutdown():
		for i in range(5):
			self.uniform_sample()
			time.sleep(1)

		rospy.spin()


	def uniform_sample(self):
		xs = np.random.uniform(self.top_left_x, self.bottom_right_x, self.num_particles)
		ys = np.random.uniform(self.bottom_right_y, self.top_left_y,  self.num_particles)

		particles = PoseArray()
		particles.header.frame_id = "map"
		particles.header.stamp  = rospy.Time.now()

		for i in range(self.num_particles):
			pose = Pose()
			pose.position.x = xs[i]
			pose.position.y = ys[i]
			particles.poses.append(pose)

		self.particle_pub.publish(particles)


	def target_handler(self, data):
		target = data.data
		probabilities = self.commonsense[target]
		choice = self.importance_sample(probabilities)
		self.resample(choice)
		self.send_robot_to(choice)


	def importance_sample(self, prob_dict):
		places = list(prob_dict.keys())
		probs = list(prob_dict.values())
		choice = np.random.choice(places, size=1, p=probs)

		return choice[0]


	def resample(self, choice):
		coords = self.places[choice]
		xs = np.random.normal(coords['x'], 1.0, size=self.num_particles)
		ys = np.random.normal(coords['y'], 1.0, size = self.num_particles)

		particles = PoseArray()
		particles.header.frame_id = "map"
		particles.header.stamp  = rospy.Time.now()

		for i in range(self.num_particles):
			pose = Pose()
			pose.position.x = xs[i]
			pose.position.y = ys[i]
			particles.poses.append(pose)

		self.particle_pub.publish(particles)


	def send_robot_to(self, place):
		pl = String()
		pl.data = place
		self.goto_pub.publish(pl)
		time.sleep(1)




if __name__ == '__main__':
	s = Slim()