import RPi.GPIO as GPIO # import GPIO librery
from time import sleep
import thread


def vw2rr(v,w,l=0.10,R=0.05) :
	r1 = (2.0*v+w*l)/2.0*R)
	r2 = (2.0*v-w*l)/2.0*R)
	return [r1,r2]
	
class MotorController :
	def __init__(self,mA,mB,mE,initThrust=0) :
		GPIO.setmode(GPIO.BCM)
		#NAMING :
		self.Motor1A = mA
		self.Motor1B = mB
		self.Motor1E = mE
		
		#SETTING :
		GPIO.setup(self.Motor1A,GPIO.OUT)
		GPIO.setup(self.Motor1B,GPIO.OUT)
		GPIO.setup(self.Motor1E,GPIO.OUT)
		
		# SAFE SETTING :
		GPIO.output(self.Motor1E,GPIO.LOW)
	
		#PWM SETTING :
		self.thrust = initThrust
		self.pwm=GPIO.PWM(self.Motor1E,100) # configuring Enable pin for PWM
		self.pwm.start(self.thrust) # starting it with (initThrust)% dutycycle
		
		self.thread = thread.start_new_thread ( MotorController.callbackTHRUST, (self) )
		
		
	def callbackTHRUST(self) :
		if self.thrust >= 0 and self.thrust <= 100 :
			GPIO.output(self.Motor1A,GPIO.HIGH)
			GPIO.output(self.Motor1B,GPIO.LOW)
			GPIO.output(self.Motor1E,GPIO.HIGH)
			
			self.pwm.ChangeDutyCycle(self.thrust)
		else if self.thrust < 0 and self.thrust >= -100 :
			GPIO.output(self.Motor1A,GPIO.LOW)
			GPIO.output(self.Motor1B,GPIO.HIGH)
			GPIO.output(self.Motor1E,GPIO.HIGH)
			
			self.pwm.ChangeDutyCycle(-self.thrust)
		else :
			print('ERROR : thrust is out-of-range...')

	def shutdown(self) :
		#self.thread.exit()
		GPIO.output(self.Motor1E,GPIO.LOW)
		self.pwm.stop()		
	
	def setThrust(self,value=0)
		self.thrust = value

def main() :
	GPIO.setmode(GPIO.BCM)

	#NAMING :
	Motor1A = 02 # set GPIO-02 as Input 1 of the controller IC
	Motor1B = 03 # set GPIO-03 as Input 2 of the controller IC
	Motor1E = 04 # set GPIO-04 as Enable pin 1 of the controller IC

	#SETTING :
	GPIO.setup(Motor1A,GPIO.OUT)
	GPIO.setup(Motor1B,GPIO.OUT)
	GPIO.setup(Motor1E,GPIO.OUT)

	# SAFE SETTING :
	GPIO.output(Motor1E,GPIO.LOW)

	#PWM SETTING :
	thrust = 50
	maxthrust = 100
	pwm=GPIO.PWM(Motor1E,maxthrust) # configuring Enable pin means GPIO-04 for PWM
	pwm.start(thrust) # starting it with 50% dutycycle

	# OPERATING : FORWARD :
	print "GO forward"
	GPIO.output(Motor1A,GPIO.HIGH)
	GPIO.output(Motor1B,GPIO.LOW)
	GPIO.output(Motor1E,GPIO.HIGH)

	sleep(2)
	# this will run your motor in forward direction for 2 seconds with 50% speed.

	thrust = 80
	pwm.ChangeDutyCycle(thrust) # increasing dutycycle to 80

	print "GO backward"
	GPIO.output(Motor1A,GPIO.LOW)
	GPIO.output(Motor1B,GPIO.HIGH)
	GPIO.output(Motor1E,GPIO.HIGH)

	sleep(2)

	# this will run your motor in reverse direction for 2 seconds with 80% speed by supplying 80% of the battery voltage

	print "Now stop"
	GPIO.output(Motor1E,GPIO.LOW)
	pwm.stop() # stop PWM from GPIO output it is necessary


	GPIO.cleanup()
	
	mc = MotorController(Motor1A,Motor1B,Motor1E)
	print "GO forward"
	mc.setThrust(50)
	sleep(2)
	
	print "GO backward"
	mc.setThrust(-50)
	sleep(2)
	
	print "Now stop"
	mc.shutdown()
	
	GPIO.cleanup()
	

if __name__ == '__main__' :
	main()
	

