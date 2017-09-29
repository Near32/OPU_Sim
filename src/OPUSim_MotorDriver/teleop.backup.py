#!/usr/bin/env python
#import roslib; roslib.load_manifest('teleop_twist_keyboard')
#import rospy

import RPi.GPIO as GPIO # import GPIO librery
from test import MotorController, vw2rr
#from geometry_msgs.msg import Twist

import sys, select, termios, tty

rangemax = 100

m1A = 18
m1B = 23
m1E = 4

mc1 = MotorController(m1A,m1B,m1E,rangeMax=rangemax)

m2A = 17
m2B = 27
m2E = 22

mc2 = MotorController(m2A,m2B,m2E,rangeMax=rangemax)

def shutdown() :
	GPIO.cleanup()
	
#rospy.on_shutdown(shutdown)


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	#pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	#rospy.init_node('teleop_twist_keyboard')

	speed = 1.5#rospy.get_param("~speed", 0.5)
	turn = 10.0#rospy.get_param("~turn", 1.0)
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(key,speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			twist = dict()#Twist()
			#twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			twist['x'] = x*speed
			#twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			twist['rz'] = th*turn			
			#pub.publish(twist)
			
			#wheelvelocities = vw2rr(twist.linear.x, twist.angular.z, l=0.10, R=0.025)
			wheelvelocities = vw2rr(twist['x'], twist['rz'], l=0.10, R=0.025)
			
			mc1.setThrust(int(wheelvelocities[0]))
			mc2.setThrust(int(wheelvelocities[1]))
			
			print wheelvelocities
			#rospy.loginfo(wheelvelocities)
			

	except Exception as e:
		print e

	#finally:
		#twist = Twist()
		#twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		#twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		#pub.publish(twist)

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

GPIO.cleanup()
