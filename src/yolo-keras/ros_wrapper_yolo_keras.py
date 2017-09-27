from keras.models import Sequential
from keras.layers import Reshape, Activation, Conv2D, Input, MaxPooling2D, BatchNormalization, Flatten, Dense
from keras.layers.advanced_activations import LeakyReLU
from keras.callbacks import EarlyStopping, ModelCheckpoint, TensorBoard
from keras.optimizers import SGD
import matplotlib.pyplot as plt
import numpy as np
import os
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose



# In[2]:

wt_path = 'tiny-yolo-voc.weights'                       
ann_dir = '/home/kevin/Development/git/basic-yolo-keras/VOCdevkit/VOC2012/Annotations/'
img_dir = '/home/kevin/Development/git/basic-yolo-keras/VOCdevkit/VOC2012/JPEGImages/'


# In[3]:

#execfile('utils.py')
exec( open("utils.py").read() )

LABELS = ['aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor']
COLORS = [(43,206,72),(255,204,153),(128,128,128),(148,255,181),(143,124,0),(157,204,0),(194,0,136),(0,51,128),(255,164,5),(255,168,187),(66,102,0),(255,0,16),(94,241,242),(0,153,143),(224,255,102),(116,10,255),(153,0,0),(255,255,128),(255,255,0),(255,80,5)]

NORM_H, NORM_W = 208, 208#416, 416
GRID_H, GRID_W = 6, 6#13 , 13
BATCH_SIZE = 8
BOX = 5
CLASS = 20
THRESHOLD = 0.01
ANCHORS = '1.08,1.19,  3.42,4.41,  6.63,11.38,  9.42,5.11,  16.62,10.52'
ANCHORS = [float(ANCHORS.strip()) for ANCHORS in ANCHORS.split(',')]
SCALE_NOOB, SCALE_CONF, SCALE_COOR, SCALE_PROB = 0.5, 5.0, 5.0, 1.0
weight_reader = WeightReader(wt_path)


# ## Construct the network

# In[4]:

model = Sequential()

# Layer 1
model.add(Conv2D(16, (3,3), strides=(1,1), padding='same', use_bias=False, input_shape=(NORM_H,NORM_W,3)))#(416,416,3)))
model.add(BatchNormalization())
model.add(LeakyReLU(alpha=0.1))
model.add(MaxPooling2D(pool_size=(2, 2)))

# Layer 2 - 5
for i in range(0,4):
    model.add(Conv2D(32*(2**i), (3,3), strides=(1,1), padding='same', use_bias=False))
    model.add(BatchNormalization())
    model.add(LeakyReLU(alpha=0.1))
    model.add(MaxPooling2D(pool_size=(2, 2)))

# Layer 6
model.add(Conv2D(512, (3,3), strides=(1,1), padding='same', use_bias=False))
model.add(BatchNormalization())
model.add(LeakyReLU(alpha=0.1))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(1,1), padding='same'))

# Layer 7 - 8
for _ in range(0,2):
    model.add(Conv2D(1024, (3,3), strides=(1,1), padding='same', use_bias=False))
    model.add(BatchNormalization())
    model.add(LeakyReLU(alpha=0.1))

# Layer 9
model.add(Conv2D(BOX * (4 + 1 + CLASS), (1, 1), strides=(1, 1), kernel_initializer='he_normal'))
model.add(Activation('linear'))
model.add(Reshape((GRID_H, GRID_W, BOX, 4 + 1 + CLASS)))


# In[5]:

#model.summary()


# ## Load pretrained weights

# In[6]:

weight_reader.reset()

for i in range(len(model.layers)):
    if 'conv' in model.layers[i].name:
        if 'batch' in model.layers[i+1].name:
            norm_layer = model.layers[i+1]
            size = np.prod(norm_layer.get_weights()[0].shape)
            
            beta  = weight_reader.read_bytes(size)
            gamma = weight_reader.read_bytes(size)
            mean  = weight_reader.read_bytes(size)
            var   = weight_reader.read_bytes(size)
            
            weights = norm_layer.set_weights([gamma, beta, mean, var])
            
        conv_layer = model.layers[i]
        if len(conv_layer.get_weights()) > 1:
            bias   = weight_reader.read_bytes(np.prod(conv_layer.get_weights()[1].shape))
            kernel = weight_reader.read_bytes(np.prod(conv_layer.get_weights()[0].shape))
            kernel = kernel.reshape(list(reversed(conv_layer.get_weights()[0].shape)))
            kernel = kernel.transpose([2,3,1,0])
            conv_layer.set_weights([kernel, bias])
        else:
            kernel = weight_reader.read_bytes(np.prod(conv_layer.get_weights()[0].shape))
            kernel = kernel.reshape(list(reversed(conv_layer.get_weights()[0].shape)))
            kernel = kernel.transpose([2,3,1,0])
            conv_layer.set_weights([kernel])


# **Randomize weights of the last layer**

# In[9]:

layer = model.layers[-3] # the last convolutional layer
weights = layer.get_weights()

new_kernel = np.random.normal(size=weights[0].shape)/(GRID_H*GRID_W)
new_bias = np.random.normal(size=weights[1].shape)/(GRID_H*GRID_W)

layer.set_weights([new_kernel, new_bias])




def _on_imagebis(image) :
	input_image = cv2.resize(image, (NORM_H, NORM_W))
	input_image = input_image / 255.
	input_image = input_image[:,:,::-1]
	input_image = np.expand_dims(input_image, 0)
	netout = model.predict(input_image)

	img,results = interpret_netoutbis(image, netout[0])
	img = img[:,:,::-1]
	return img, results

def test_on_rpicam() :
	model.load_weights("weights.hdf5")
	# initialize the camera and grab a reference to the raw camera capture
	camera = PiCamera()
	camera.resolution = (1088, 960)
	camera.framerate = 30
	rawCapture = PiRGBArray(camera, size=(1088, 960))
	 
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
		
		end = time.time()
		freq = 1.0/(end-start)
		size = 1
		thickness = 2
		color = (255,0,0) 
		cv2.putText(image, '{} Hz'.format(freq ), (10,25),cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness )
		# Display the resulting frame
		# show the frame
		outimage = cv2.resize(image, (640,480) )
		cv2.imshow("Frame", outimage)

		#HANDLE THE OUTPUT RESULTS HERE :
		publishModelStates(results)
	

def publishModelStates(results,publisher) :
	modelstate = ModelStates()
	dummytwist = Twist()
	state = None
	while state is None :
		if len(buffstate) :
			state = buffstate[-1]
			buffstate = []
	# for synchro purpose : add the current state of the robot in the modelstate :
	robotstateindex = [ index for index in range(len(state)) if 'self' in state.name[index] ]
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
			meany = np.mean( [ pos[1], pose[3] ] )

			r, theta = uv2rtheta(minx,meany)
			x = r*np.cos(theta)
			y = r*np.sin(theta)

			p = Pose()
			p.position.x = x
			p.position.y = y
			modelstate.name.append( 'target' )
			modelstate.pose.append( p)
			modelstate.twist.append(dummytwist)

	publisher.publish(modelstate,publisher=pubmodelstate)






def uv2rtheta(u,v) :
	# u \in [rangeu, 2*rangeu]
	rangeu = 156/2
	u = -(u-2*rangeu)
	# u \in [0, rangeu] u--> 0 == object near
	r = 1.0/(u-rangeu)

	rangev = 156
	rangetheta = 170
	offsettheta = 75
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


test_on_rpicam()
