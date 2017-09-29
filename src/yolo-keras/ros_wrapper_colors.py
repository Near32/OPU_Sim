import numpy as np
import os
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist

NORM_H = 480
NORM_W = 640

def publishModelStates(results,publisher) :
	global buffstate
	
	modelstate = ModelStates()
	dummytwist = Twist()
	state = None
	while state is None :
		if len(buffstate) :
			state = buffstate[-1]
			buffstate = []
	# for synchro purpose : add the current state of the robot in the modelstate :
	robotstateindex = [ index for index in range(len(state.name)) if 'self' in state.name[index] ][0]
	modelstate.name.append('self')
	modelstate.pose.append( state.pose[robotstateindex] )
	modelstate.twist.append(dummytwist)

	for el in results :
		name = el[0]
		pos = el[1]

		if 'bottle' in name :
			#let us add this to the modelstate as the target :
			#Compute its distance :
			minx = np.min( [pos[0],pos[2] ])
			meany = np.mean( [ pos[1], pos[3] ] )

			r, theta = uv2rtheta(minx,meany)

			rospy.loginfo(' target is at : r= {} // thetadeg= {}'.format(r,theta*180.0/np.pi) )

			x = r*np.cos(theta)
			y = r*np.sin(theta)

			rospy.loginfo(' target is at : x= {} // y= {}'.format(x,y) )
			
			p = Pose()
			p.position.x = x
			p.position.y = y
			modelstate.name.append( 'target' )
			modelstate.pose.append( p)
			modelstate.twist.append(dummytwist)

	publisher.publish(modelstate)






def uv2rtheta(u,v,rangeu=NORM_H/2,rangev=NORM_W,offsettheta=75) :
	# u \in [rangeu, 2*rangeu]
	u = -(u-2*rangeu)
	# u \in [0, rangeu] u--> 0 == object near
	r = 1.0/(u-rangeu)

	rangetheta = 170
	thetadeg = v/rangev*rangetheta + offsettheta
	thetarad = thetadeg*np.pi/180.0

	return r,thetarad


import argparse

continuer = True
def shuttingdown() :
	continuer = False

buffstate = list()
def callbackState(state) :
	buffstate.append(state)

parser = argparse.ArgumentParser(description="OPUSim_YOLO_KERAS_ros_wrapper")
parser.add_argument('-number', help='index number of the teleoperated robot.', dest='number', default=0 )
args, unknown = parser.parse_known_args()

#ROS :
rospy.init_node('OPUSim_YOLO_KERAS_{}'.format(args.number), anonymous=False)
rospy.on_shutdown(shuttingdown)

subState = rospy.Subscriber( '/robot_model_teleop_{}/DATMO'.format(args.number), ModelStates, callbackState )
pubmodelstate = rospy.Publisher( '/robot_model_teleop_{}/YOLO'.format(args.number), ModelStates, queue_size=10 )

def _on_imagebis(im) :
	'''
	# Set up the detector with default parameters.
	detector = cv2.SimpleBlobDetector()
	 
	# Detect blobs.
	keypoints = detector.detect(im)
	 
	# Draw detected blobs as red circles.
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
	im_with_keypoints = cv2.drawKeypoints(im, keypoints, 
						np.array([]), 
						(0,0,255), 
						cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	results = []
	for elem in keypoints :
		results.append( ('bottle',[elem.pt[0], elem.pt[1], elem.pt[0], elem.pt[1]] ) )

	return im_with_keypoints, results
	'''
	
	hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
	# define range of white color in HSV
	# change it according to your need !
	lower_white = np.array([20,1,0], dtype=np.uint8)
	upper_white = np.array([100,20,10], dtype=np.uint8)

	# Threshold the HSV image to get only white colors
	#mask = cv2.inRange(hsv, lower_white, upper_white)
	mask = cv2.inRange(im, lower_white, upper_white)
	# Bitwise-AND mask and original image
	res = cv2.bitwise_and(im,im, mask= mask)

	#cv2.imshow('im',im)
	#cv2.imshow('res',res)
	
	results = []
	'''meanx = 0.0
	meany =0.0
	n = 0
	for i in range(mask.shape[0]) :
		for j in range(mask.shape[1]) :
			if mask[i,j] >= 125 :
				n+= 1
				meanx += i
				meany += j
	meanx /= n
	meany /= n
	results.append( ('bottle',[meanx,meany,meanx,meany] ) )
	'''
	# Set up the detector with default parameters.
	detector = cv2.SimpleBlobDetector()
	 
	# Detect blobs.
	keypoints = detector.detect(mask)
	 
	# Draw detected blobs as red circles.
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
	im_with_keypoints = cv2.drawKeypoints(res, keypoints, 
						np.array([]), 
						(0,0,255), 
						cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	results = []
	for elem in keypoints :
		results.append( ('bottle',[elem.pt[0], elem.pt[1], elem.pt[0], elem.pt[1]] ) )
	return res, results
 
def test_on_rpicam() :
	# initialize the camera and grab a reference to the raw camera capture
	camera = PiCamera()
	camera.vflip = True
	camera.hflip = True
	camera.resolution = (640, 480)
	camera.framerate = 30
	rawCapture = PiRGBArray(camera, size=(640, 480))
	 
	# allow the camera to warmup
	time.sleep(1)
	 
	# capture frames from the camera
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		start = time.time()
		# grab the raw NumPy array representing the image, then initialize the timestamp
		# and occupied/unoccupied text
		image = frame.array
	 	
		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)
	 
		# if the `q` key was pressed, break from the loop
		key = cv2.waitKey(1) & 0xFF
	 	if key == ord("q"):
			break

		# Our operations on the frame come here
		img, results = _on_imagebis(image)
		#img = image.copy()
		#results = []

		end = time.time()
		freq = 1.0/(end-start)
		
		size = 2
		thickness = 2
		color = (255,0,0) 
		cv2.putText(image, '{} Hz'.format(freq ), (10,50),cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness )
		cv2.putText(img, '{} Hz'.format(freq ), (10,50),cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness )
		# Display the resulting frame
		# show the frame
		outimage = cv2.resize(image, (320,240) )
		outimg = cv2.resize(img, (320,240) )
		#cv2.imshow("Frame", outimage)
		cv2.imshow("Frame Keypoints", outimg)


		#HANDLE THE OUTPUT RESULTS HERE :
		if True :
			publishModelStates(results,publisher=pubmodelstate)

test_on_rpicam()
