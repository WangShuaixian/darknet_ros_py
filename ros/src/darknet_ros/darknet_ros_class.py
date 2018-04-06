#!/usr/bin/env python

import rospy, yaml
from std_msgs.msg import String, Empty

import os
from subprocess import Popen, PIPE


class DarknetRosPy(object):
	
	def __init__(self):
		
		self.request_subscriber = rospy.Subscriber("detection_request", Empty, self.detection_request_callback)
		self.result_publisher = rospy.Publisher("detection_result", String, queue_size=10)
		self.proc = None
		self.detection_requested = False
		
		darknet_bin_dir_param = rospy.get_param("darknet_bin_dir", "")
		self.default_stdin_line = rospy.get_param("default_stdin_line", "")
		
		# check darknet path exists
		darknet_bin_dir = os.path.expanduser(darknet_bin_dir_param)
		if not os.path.exists(darknet_bin_dir):
			rospy.logerr("darknet_bin_dir [%s] not valid", darknet_bin_dir)
			return
		
		else:
			rospy.logdebug("Found [%s]", darknet_bin_dir)
		
		darknet_bin_path = os.path.join(darknet_bin_dir, "darknet")
		
		if not os.path.exists(darknet_bin_path):
			rospy.logerr("executable not found [%s]", darknet_bin_path)
			return
		
		else:
			rospy.logdebug("executable found [%s]", darknet_bin_path)
		
		os.chdir(darknet_bin_dir)
		
		self.proc = Popen(['./darknet', 'detect', 'cfg/yolov3.cfg', 'yolov3.weights'], stdin=PIPE, stdout=PIPE, stderr=PIPE)
		
		
		
	def loop(self):
	
		r = rospy.Rate(10)
		
		while not rospy.is_shutdown():
			
			if self.detection_requested:
				
				self.detection_requested = False
				
				self.proc.stdin.write(self.default_stdin_line+'\n')
				rospy.logdebug("stdin written to pipe [%s]", self.default_stdin_line)
			
				stdout_line = self.proc.stdout.readline()
			
				if stdout_line != '':
				
					## example yaml format of stdout
#					stdout_line = "[{\"label\": 'A', \"confidence\": 0.1381, \"x_min\": 0.543, \"x_max\": 0.546834, \"y_min\": 0.843, \"y_max\": 0.541834}, {\"label\": 'B', \"confidence\": 0.9999999, \"x_min\": 0.2, \"x_max\": 0.1, \"y_min\": 0.1, \"y_max\": 0.0}]"
				
					try:
						result = yaml.load(stdout_line.rstrip())
					except yaml.YAMLError:
						rospy.logerr("Invalid format for detections result from darknet executable")
						return
				
					for detection in result:
						rospy.loginfo('''\n'''
						'''class:      %s\n'''
						'''confidence: %s\n'''
						'''x_min:      %s\n'''
						'''x_max:      %s\n'''
						'''y_min:      %s\n'''
						'''y_max:      %s''', detection['class'],detection['confidence'],detection['x_min'],detection['x_max'],detection['y_min'],detection['y_max'])
				
					self.result_publisher.publish(yaml.dump(result))
				
				else:
					rospy.logerr("empty stdout line")
					return
				
		
			r.sleep()

	def detection_request_callback(self, msg):
		rospy.logdebug("detection_request_callback")
		self.detection_requested = True


if __name__ == '__main__':
	try:
		rospy.init_node('dark_ros_py_node')
		rospy.logdebug("Node initialised")
		
		DarknetRosPy().loop()
		
	except rospy.ROSInterruptException:
		pass
