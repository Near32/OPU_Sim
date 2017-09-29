#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO # import GPIO librery
from test import MotorController, vw2rr
import argparse


def rr2vw(rr, l=0.10,R=0.05) :
	twist = Twist()
	v = R * ( rr[0]+rr[1] ) / 2.0
	w = R * (rr[0] - rr[1] )	
	twist.linear.x = v
	twist.angular.z = w
	return twist




buffcmd = list()
def callbackCOMMAND(cmd) :
	buffcmd.append(cmd)

def shuttingdown() :
	GPIO.cleanup()



def main() :
	m1A = 18
	m1B = 23
	m1E = 4

	mc1 = MotorController(m1A,m1B,m1E)
	mc1.setThrust(0)

	m2A = 17
	m2B = 27
	m2E = 4

	mc2 = MotorController(m2A,m2B,m2E)
	mc2.setThrust(0)

	print('INITIALIZED...')
	parser = argparse.ArgumentParser()
	parser.add_argument("--debug", action='store_true', help="debug mode.")
	parser.add_argument("-number", action='store', dest='number', type=int, default=0, help="index number of the teleoperated robot.")
	parser.add_argument("-scaler", action='store', dest='scaler', type=float, default=1.0, help="scaler.")
	parser.add_argument("-max", action='store', dest='max', type=float, default=0.025, help="max vel.")
	parser.add_argument("-maxThrust", action='store', dest='maxThrust', type=float, default=20, help="max Thrust.")
	
	args = parser.parse_args()

	sub = rospy.Subscriber('/robot_model_teleop_{}/cmd_vel'.format(args.number), Twist, callbackCOMMAND )
	pubCmd = rospy.Publisher( '/robot_model_teleop_{}/cmd_vel_odometry'.format(args.number), Twist, queue_size=10 )

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
			
				twist.linear.x *= args.scaler
				twist.angular.z *= args.scaler
			
				wvel = vw2rr(twist.linear.x, twist.angular.z, l=0.10, R=0.025)
				wvel = ( min([wvel[0], args.max]), min([wvel[1], args.max] ) )
				wheelvelocities = ( wvel[0]*args.maxThrust/args.max , wvel[1]*args.maxThrust/args.max )
				 
				realtwist = rr2vw(wheelvelocities, l=0.10, R=0.025)
			
				if args.debug == False :
					mc1.setThrust( int(wheelvelocities[0]) )
					mc2.setThrust( int(wheelvelocities[1]) )
			
					#pubCmd.publish(realtwist)
			
				rospy.loginfo(wheelvelocities)
				rospy.loginfo( 'REAL TWIST : v={} // w={}'.format(realtwist.linear.x, realtwist.angular.z ) )
			rate.sleep()

	except Exception as e:
		print e

main()
