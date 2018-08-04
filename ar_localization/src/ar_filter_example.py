#!/usr/bin/env python
'''
Authored by Caleb Trotz, August 2018
This node does the following:
- subscribes to ar_track_alvar
- filters each AlvarMarkers message to contain only markers within a certain distance
- publishes an updated AlvarMarkers message with the filtered markers
'''
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

_FILTER_DIST = 0.75 # anything less than _FILTER_DIST meters away will be captured

class ARFilter:
	def __init__(self):
		self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb)
		self.ar_pose_pub = rospy.Publisher("/filtered_markers", AlvarMarkers, queue_size=1)

	def ar_pose_cb(self,msg):
		markers = msg.markers # an array of AlvarMarkers

		def pose_filter(marker):
			if marker.pose.pose.position.z > _FILTER_DIST: # z can be considered the same as body up x
				return False # it's not a good tag because it's too far
			return True # it's a good tag because it's close enough

		filtered_markers = filter(pose_filter,markers) # calls pose_filter on every item in markers

		if len(filtered_markers) < 1: return # if no valid markers, don't publish anything

		output_message = AlvarMarkers() # initalizes a new AlvarMarkers message
		output_message.markers = filtered_markers # sets the array of markers to the filtered list
		self.ar_pose_pub.publish(output_message) # publishes the filtered message

