#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO # import GPIO librery
from test import MotorController, vw2rr



m1A = 17
m1B = 27
m1E = 4

mc1 = MotorController(m1A,m1B,m1E)

m2A = 17
m2B = 27
m2E = 4

mc2 = MotorController(m2A,m2B,m2E)


buffcmd = list()
def callbackCOMMAND(cmd) :
	buffcmd.append(cmd)

def shuttingdown() :
	GPIO.cleanup()



if __name__=="__main__":
    parser = argparse.ArgumentParser()
	parser.add_argument("--debug", action='store_true', help="debug mode.")
	parser.add_argument("-number", action='store', dest='number', type=int, default=0, help="index number of the teleoperated robot.")
	
	args = parser.parse_args()
	

	sub = rospy.Subscriber('/robot_model_teleop_{}/cmd_vel'.format(args.number), Twist, callbackCOMMAND )
	rospy.init_node('OPUSim_MotorDriver_{}'.format(args.number))
	rospy.on_shutdown(shuttingdown)

	rate = rospy.Rate(100)
	twist = Twist()
	tcmd = None

	try:
		while not rospy.is_shutdown():
			
			if len(buffcmd) :
				tcmd = buffcmd[-1]
				del buffcmd[:]

			if tcmd is not None :
				twist = tcmd

			wheelvelocities = vw2rr(twist.linear.x, twist.angular.z, l=0.10, R=0.025)
			
			mc1.setThrust(wheelvelocities[0])
			mc1.setThrust(wheelvelocities[1])
			
			#rospy.loginfo(wheelvelocities)
			

	except Exception as e:
		print e