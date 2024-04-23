#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Int16MultiArray, Float32MultiArray, Int8
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
import time
from tf.transformations import euler_from_quaternion

class CmdVelConverter:

	def __init__(self):

		rospy.init_node("cmd_vel_converter", anonymous=True)

		## ROS params ##
		self.pwm_left_max_db = 1500
		self.pwm_right_max_db = 1500
		self.pwm_left_min_db = 1500
		self.pwm_right_min_db = 1500
		self.vx_max = 2.0
		self.wz_max = 2.0
		self.show_log = True

		### Cart params ###
		self.pwm_max = 2000
		self.pwm_min = 1000
		self.pwm_mid = 1500
		self.prev_y = 0.0

		## Pub/Sub ##
		rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

		self.pwm_cmd_pub = rospy.Publisher("/md100a/pwm_cmd", Int16MultiArray, queue_size=10)
		
		print("start cmd_vel_converter")
		rospy.spin()


	####################
	### ROS callback ###
	####################
	def cmd_vel_callback(self, msg):
		
		if msg.linear.x > self.vx_max:
			vx = self.vx_max
		elif msg.linear.x < -self.vx_max:
			vx = -self.vx_max
		else:
			vx = msg.linear.x

		if msg.angular.z > self.wz_max:
			wz = self.wz_max
		elif msg.angular.z < -self.wz_max:
			wz = -self.wz_max
		else:
			wz = msg.angular.z

		if ((abs(vx) > 0.0) and (abs(wz) > 0.0)):
			y_percent = self.map(vx, -self.vx_max, self.vx_max, -100.0, 100.0)
			# x_percent = self.map(wz, -self.wz_max, self.wz_max, y_percent, -y_percent)
			if vx >= 0.0:
				x_percent = self.map(wz, -self.wz_max, self.wz_max, 100.0, -100.0)
			else:
				x_percent = self.map(wz, -self.wz_max, self.wz_max, 100.0, -100.0)
		else:
			y_percent = self.map(vx, -self.vx_max, self.vx_max, -100.0, 100.0)
			x_percent = self.map(wz, -self.wz_max, self.wz_max, 100.0, -100.0)
			

		left_200_per, right_200_per = self.xy_mixing(x_percent, y_percent)
		left_pwm, right_pwm = self.wheels_percent_to_wheels_pwm(left_200_per, right_200_per)

		# print(int(left_pwm), int(right_pwm))

		if (self.show_log):
			print("vx: {:.1f} wz: {:.1f} left_pwm: {:d} right_pwm: {:d}".format(\
				vx, wz, int(left_pwm), int(right_pwm)))

		pwm_cmd_msg = Int16MultiArray()
		pwm_cmd_msg.data = [int(left_pwm), int(right_pwm)]
		self.pwm_cmd_pub.publish(pwm_cmd_msg)

	####################
	### Cart control ###
	####################
	def map(self, val, in_min, in_max, out_min, out_max):
		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min
		return out

	def xy_mixing(self, x, y):
		## x, y must be in the range of -100 to 100

		left = y+x
		right = y-x

		diff = abs(x) - abs(y)

		if (left < 0.0):
			left = left - abs(diff)
		else:
			left = left + abs(diff)

		if (right < 0.0):
			right = right - abs(diff)
		else:
			right = right + abs(diff)

		if (self.prev_y < 0.0):
			swap = left
			left = right
			right = swap
		
		self.prev_y = y

		## left and right are in -200 to 200 ranges

		return left, right


	def wheels_percent_to_wheels_pwm(self, left_per, right_per):

		if left_per > 0.0:
			left_pwm = self.map(left_per, 0.0, 200.0, self.pwm_left_max_db, self.pwm_max)
		elif left_per < 0.0:
			left_pwm = self.map(left_per, -200.0, 0.0, self.pwm_min, self.pwm_left_min_db)
		else:
			left_pwm = self.pwm_mid

		if right_per > 0.0:
			right_pwm = self.map(right_per, 0.0, 200.0, self.pwm_right_max_db, self.pwm_max)
		elif right_per < 0.0:
			right_pwm = self.map(right_per, -200.0, 0.0, self.pwm_min, self.pwm_right_min_db)
		else:
			right_pwm = self.pwm_mid

		return left_pwm, right_pwm


if __name__ == "__main__":
	CmdVelConverter()