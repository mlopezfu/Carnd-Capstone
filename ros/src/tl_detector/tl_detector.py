#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import os
import uuid

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
	def __init__(self):
		rospy.init_node('tl_detector')

		self.pose = None
		self.waypoints = None
		self.camera_image = None
		self.lights = []

		sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

		'''
		/vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
		helps you acquire an accurate ground truth data source for the traffic light
		classifier by sending the current color state of all traffic lights in the
		simulator. When testing on the vehicle, the color state will not be available. You'll need to
		rely on the position of the light and the camera image to predict it.
		'''
		sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
		sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

		config_string = rospy.get_param("/traffic_light_config")
		self.config = yaml.load(config_string)

		self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

		self.bridge = CvBridge()
		self.light_classifier = TLClassifier()
		self.listener = tf.TransformListener()

		self.state = TrafficLight.UNKNOWN
		self.last_state = TrafficLight.UNKNOWN
		self.last_wp = -1
		self.state_count = 0

		rospy.spin()

	def pose_cb(self, msg):
		self.pose = msg

	def waypoints_cb(self, waypoints):
		if self.waypoints is None:
			self.waypoints = waypoints.waypoints

	def traffic_cb(self, msg):
		self.lights = msg.lights

	def image_cb(self, msg):
		"""Identifies red lights in the incoming camera image and publishes the index
			of the waypoint closest to the red light's stop line to /traffic_waypoint

		Args:
			msg (Image): image from car-mounted camera

		"""
		self.has_image = True
		self.camera_image = msg
		light_wp, state = self.process_traffic_lights()
		#cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
		#cv2.imwrite(os.path.join(str(state),str(uuid.uuid4())+".png"),cv_image)

		'''
		Publish upcoming red lights at camera frequency.
		Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
		of times till we start using it. Otherwise the previous stable state is
		used.
		'''		
		if self.state != state:
			self.state_count = 0
			self.state = state
		elif self.state_count >= STATE_COUNT_THRESHOLD:
			self.last_state = self.state
			light_wp = light_wp if state == TrafficLight.RED else -1
			self.last_wp = light_wp
			self.upcoming_red_light_pub.publish(Int32(light_wp))
		else:
			self.upcoming_red_light_pub.publish(Int32(self.last_wp))
		self.state_count += 1

		self.upcoming_red_light_pub.publish(Int32(light_wp))
	def get_closest_waypoint(self, pose):
		"""Identifies the closest path waypoint to the given position
			https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
		Args:
			pose (Pose): position to match a waypoint to

		Returns:
			int: index of the closest waypoint in self.waypoints

		"""
		#TODO implement
		if self.waypoints is None:
			return
		min_dist = 10000
		min_loc = None

		pos_x = pose.position.x
		pos_y = pose.position.y
		# check all the waypoints to see which one is the closest to our current position
		for i, waypoint in enumerate(self.waypoints):
			wp_x = waypoint.pose.pose.position.x
			wp_y = waypoint.pose.pose.position.y
			dist = math.sqrt((pos_x - wp_x)**2 + (pos_y - wp_y)**2)
			#rospy.loginfo('dist %s', dist)
			if (dist < min_dist): #we found a closer wp
				min_loc = i     # we store the index of the closest waypoint
				min_dist = dist     # we save the distance of the closest waypoint

		# returns the index of the closest waypoint
		return min_loc

	def get_light_state(self, light):
		"""Determines the current color of the traffic light

		Args:
			light (TrafficLight): light to classify

		Returns:
			int: ID of traffic light color (specified in styx_msgs/TrafficLight)

		"""
		if(not self.has_image):
			self.prev_light_loc = None
			return False

		cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

		#Get classification
		return self.light_classifier.get_classification(cv_image)

	def process_traffic_lights(self):
		"""Finds closest visible traffic light, if one exists, and determines its
			location and color

		Returns:
			int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
			int: ID of traffic light color (specified in styx_msgs/TrafficLight)

		"""
		light = None
		closest_light_wp = None
		closest_light_stop_wp = None
		dist_to_light = 10000   #initialize to high value

		# List of positions that correspond to the line to stop in front of for a given intersection
		stop_line_positions = self.config['stop_line_positions']
		#rospy.loginfo('stop_line_positions %s', stop_line_positions)
		if(self.pose):
			car_position = self.get_closest_waypoint(self.pose.pose)
			#rospy.loginfo('car_position %s', car_position)
		else:
			return -1, TrafficLight.UNKNOWN

		for light_stop_position in stop_line_positions:
			light_stop_pose = Pose()
			light_stop_pose.position.x = light_stop_position[0]
			light_stop_pose.position.y = light_stop_position[1]
			light_stop_wp = self.get_closest_waypoint(light_stop_pose)     #get the wp closest to each light_position
			#
			if light_stop_wp >= car_position :    #it found a waypoint close to the traffic light and ahead of the car
				if closest_light_stop_wp is None:    #check if this is the first light we process
					closest_light_stop_wp = light_stop_wp
					light = light_stop_pose
				elif light_stop_wp < closest_light_stop_wp:
					closest_light_stop_wp = light_stop_wp    #if we have a closer light_wp ahead of the car position, we allocate closer value
					light = light_stop_pose
			
		#rospy.loginfo('car_position %s %s %s light_stop_wp %s %s %s',car_position,self.pose.pose.position.x,self.pose.pose.position.y, closest_light_stop_wp,light.position.x,light.position.y)
		#
		light_state_via_msg=0
		light_position_idx=-1
		state=TrafficLight.UNKNOWN
		#TODO find the closest visible traffic light (if one exists)
		#get the ground truth traffic light states through the traffic light messages
		if(closest_light_stop_wp!=None):
			if(closest_light_stop_wp-car_position<250): #We are near the stop
				#rospy.loginfo('self.lights %s', self.lights)
				for tl in self.lights:
					#rospy.loginfo('lights x %s light %s state %s',tl.pose.pose.position.x,light.position.x,tl.state);
					
					#rospy.loginfo('indice %s', indice)
					dist = math.sqrt((tl.pose.pose.position.x - light.position.x)**2 + (tl.pose.pose.position.y - light.position.y)**2)
					#rospy.loginfo('distance %s %s', dist, dist < 50)

					if (dist < 50): #means we found the light close to the stop line
						#rospy.loginfo('lights x %s light %s state %s',tl.pose.pose.position.x,light.position.x,tl.state)
						light_state_via_msg = tl.state
						#rospy.loginfo('light_state_via_msg %s %s %s', tl.pose.pose.position.x,tl.pose.pose.position.y,light_state_via_msg)
						
						state=tl.state
						light_wp=Pose()
						light_wp.position.x=tl.pose.pose.position.x
						light_wp.position.y=tl.pose.pose.position.y
						light_position_idx = self.get_closest_waypoint(light_wp)
						#rospy.loginfo('closest_light_stop_wp %s light_position_idx %s',closest_light_stop_wp, light_position_idx)
						break
						#break #no need to parse other lights once light was found

		
		#rospy.loginfo('car_position %s closest_light_stop_wp %s light_position_idx %s state %s',car_position,closest_light_stop_wp, light_position_idx,light_state_via_msg)
		if light:
			#state = light_state_via_msg #self.get_light_state(light)
			state = self.get_light_state(light)
			return closest_light_stop_wp, state
		#self.waypoints = None
		return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
	try:
		TLDetector()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start traffic node.')
