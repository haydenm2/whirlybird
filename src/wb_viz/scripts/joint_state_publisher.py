#!/usr/bin/env python3

import rospy
import time
from sensor_msgs.msg import JointState
from wb_msgs.msg import WbStates

class JointStatePublisher():

	def __init__(self):
		self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size = 1)
		self.whirlybird_sub = rospy.Subscriber('/wb_states', WbStates, self.whirlybird_callback)
		
		time.sleep(1)

		state = JointState()
		state.header.stamp = rospy.Time.now()
		state.name = ['yaw_joint', 'pitch_joint', 'roll_joint']
		state.position = [0, 0, 0] # switch from NED to NWU
		self.joint_state_pub.publish(state)

		rospy.spin()

	def whirlybird_callback(self, msg):
		# self.whirlybird_sub.getTopic()
		state = JointState()
		state.header.stamp = rospy.Time.now()
		state.name = ['yaw_joint', 'pitch_joint', 'roll_joint']
		state.position = [-msg.yaw, -msg.pitch, msg.roll] # switch from NED to NWU
		self.joint_state_pub.publish(state)

if __name__ == '__main__':
	rospy.init_node('joint_state_publisher')
	try:
		jsp = JointStatePublisher()
	except:
		rospy.ROSInterruptException
	pass
