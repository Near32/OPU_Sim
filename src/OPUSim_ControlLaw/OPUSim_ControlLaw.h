#ifndef OPUSIM_CONTROLLAW_H
#define OPUSIM_CONTROLLAW_H

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>

using namespace std;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/Twist.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"

#include "../PIDController/PIDControllerM.h"

namespace enc = sensor_msgs::image_encodings;

//#define debug_v0		
#define PI 3.1415926f


	
float normePoint( const cv::Point& p)
{
	return sqrt( p.x*p.x+p.y*p.y );
}

cv::Mat rotation(const float& radianangle)
{
	cv::Mat r = cv::Mat::zeros( cv::Size(2,2), CV_32F);
	r.at<float>(0,0) = cos(radianangle);
	r.at<float>(1,1) = cos(radianangle);
	r.at<float>(0,1) = sin(radianangle);
	r.at<float>(1,0) = -sin(radianangle);
	
	return r;
}

cv::Mat transformation( const float& radianangle, const cv::Mat& t)
{
	cv::Mat trans;
	cv::hconcat( rotation(radianangle), t, trans);
	cv::Mat dummy = cv::Mat::zeros( 1,3, CV_32F);
	dummy.at<float>(0,2) = 1;
	cv::vconcat( trans, dummy, trans);
	return trans;
}

cv::Mat polar2euclidian( const cv::Mat& polars)
{
	cv::Mat eucl = cv::Mat::ones( polars.rows, polars.cols, CV_32F);
	
	for(int i=0;i<eucl.cols;i++)
	{
		float r=polars.at<float>(0,i);
		float radianangle=polars.at<float>(1,i);
		float x = r*cos(radianangle);
		float y = r*sin(radianangle);
		eucl.at<float>(0,i) = x;
		eucl.at<float>(1,i) = y;
	}
	
	std::cout << eucl << std::endl;
	
	return eucl;
}



float atan21( const float& y, const float& x)
{
    float r = y;

    if( x!= 0)
    {
        if(x < 0)
        {
            if(y<0)
                r = PI + atan(y/x);
            else
                r = PI/2.0 - atan(y/x);
        }
        else
            r = atan(y/x);
    }
    else
        r = ( y <= 0 ? -PI/2.0 : PI/2.0);

    return r;

}

cv::Mat euclidian2polar( const cv::Mat& eucl)
{
	cv::Mat polars = cv::Mat::zeros( eucl.size(), CV_32F);
	
	for(int i=0;i<eucl.cols;i++)
	{
		float x= eucl.at<float>(0,i);
		float y= eucl.at<float>(1,i);
		
		float radianangle = atan21( y, x);		
		float radius = sqrt( x*x+y*y);
		
		polars.at<float>(0,i) = radius;
		polars.at<float>(1,i) = radianangle;
	}
	
	return polars;
}


cv::Mat polar2euclidianTarget( const cv::Mat& polars, const cv::Mat& polarTargetInRobot)
{
	//float radianangle = polarTargetInRobot.at<float>(1,0) * PI/180.0f+PI/2.0f;
	/*
	float radianangle = polarTargetInRobot.at<float>(1,0) * PI/180.0f;
	if(radianangle < 0.0f)
	{
		radianangle = -PI/2.0f-radianangle;
	}
	else
	{
		radianangle += PI/2.0f;
	}
	*/
	float radianangle = polarTargetInRobot.at<float>(1,0) * PI/180.0f+PI;
	
	//std::cout << " radian rotation angle : " << radianangle << std::endl;
	
	cv::Mat tT2RinT = cv::Mat::zeros(2,1, CV_32F);
	tT2RinT.at<float>(0,0) = polarTargetInRobot.at<float>(0,0);	
	cv::Mat TR2T = transformation( radianangle, tT2RinT );
	
	//std::cout << "robot position in target frame : eucl : " << tT2RinT << std::endl;
	cv::Mat eucl = polar2euclidian( polars);
	
	//std::cout << " eucl positions in robot frame : " << eucl << std::endl;
	
	return TR2T*eucl;	
}

class OPUSim_ControlLaw
{
	protected :
	
	std::mutex mutexRES;
	bool continuer;
	
	std::vector<cv::Mat> frames;
	std::thread* t;
	
	float R;
	float a;
	float epsilon;
	float kv;
	float kw;
	float Omega;
	
	float gain;
	float THETA;
	float r;
	
	float tau;
	
	bool emergencyBreak;
	bool verbose;
	
	PIDControllerM<float> pid;
	
	public :
	
	ros::NodeHandle nh;
	
	
	image_transport::ImageTransport* it;
	ros::Publisher twistpub;
	image_transport::Subscriber img_sub;
	
	int robot_number;
	geometry_msgs::Twist twistmsg;
	
	OPUSim_ControlLaw(const int& robot_number_, const bool& emergencyBreak_ = false, const bool& verbose_ = false, const float& gain_=4.0f, const float& R_=3.0f, const float& a_=1.0f, const float& epsilon_=10.0f, const float& kv_=0.1f, const float& kw_=0.2f, const float& Omega_=1.0f) : continuer(true), robot_number(robot_number_), R(R_), a(a_), epsilon(epsilon_), kv(kv_), kw(kw_), Omega(Omega_), gain(gain_), THETA(0.0f), r(0.0f), emergencyBreak(emergencyBreak_), verbose(verbose_),tau(10.0f)
	{			
		it = new image_transport::ImageTransport(nh);
		
		std::string path( "/robot_model_teleop_"+std::to_string(robot_number)+"/");
		//std::string path( "/robot_model_teleop/");
		std::string pathSUB(path+"RSO");
		std::string pathPUB(path+"cmd_vel");
		
		img_sub = it->subscribe( pathSUB.c_str(), 1, &OPUSim_ControlLaw::callback,this);
		twistpub = nh.advertise<geometry_msgs::Twist>( pathPUB.c_str(), 10);
		
		
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		this->pid.setConsigne(Mat<float>(0.0f,1,1));
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		
		
		t = new std::thread(&OPUSim_ControlLaw::loop, this);
		
		ROS_INFO( std::string("OPUSim_ControlLaw::"+std::to_string(robot_number)+"::Initialization : OK.").c_str() );
	}
	
	~OPUSim_ControlLaw()
	{
		
		this->setContinuer(false);
		
		if(t->joinable())
		{
			t->join();
		}
		
		delete t;
		delete it;
		
		ROS_INFO("OPUSim_ControlLaw::Exiting.");
	}
	
	void callback(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
	
		try
		{
			//cv_ptr = cv_bridge::toCvCopy(original_image,enc::BGR8);
			cv_ptr = cv_bridge::toCvCopy(original_image,enc::TYPE_32FC1);
		}
		catch( cv_bridge::Exception e)
		{
			ROS_ERROR("OPUSim_ControlLaw::::cv_bridge exception : %s", e.what());
			return ;
		}
		//------------------------------------------------
		//		Stack of Frames
		//-----------------------------------------------
		cv::Mat frameDownSample;
		cv_ptr->image.copyTo( frameDownSample);
    
		//------------------------------
		//------------------------------
		
		
		mutexRES.lock();
		frames.insert(frames.begin(), frameDownSample);	
		mutexRES.unlock();
	}
	
	
	
	
	void loop()
	{
		clock_t timer = clock();
		int count_info = 0;
		
		cv::Mat currentmsg;
    int nbrRobotVisible = 0;
    
		mutexRES.lock();
		while(continuer)
		{
			mutexRES.unlock();
			
			clock_t timerloop = clock();
			
			
			bool goOn = true;
			
			if(frames.size() >= 1)
			{
				goOn = true;
			}
			else
			{
				goOn = false;
			}
			
			if(goOn)
			{
				
				mutexRES.lock();
				frames[0].copyTo(currentmsg);
				frames.clear();
				mutexRES.unlock();
				
				if(this->verbose)
				{
					std::cout << " CURRENT MSG : " << currentmsg << std::endl;
				}
				
				//----------------------------------------------------
				//----------------------------------------------------
				//IDENTIFY COUPLING PAIR
				//----------------------------------------------------
				//----------------------------------------------------
				//-- Angular position rearrangin in [0,2*PI] :
				nbrRobotVisible = currentmsg.at<float>(0,0);
				THETA = currentmsg.at<float>(1,0);
				//it is the current THETA of the robot in the target reference-focused frame...
				
				if( THETA < 0.0f)
				{
					THETA = -(PI+THETA);
				}
				else
				{
					THETA = PI-THETA;
				}
				
				float sumphi = 0.0f;
				float mintheta = 2*PI;
				
				if(nbrRobotVisible>1)
				{
					//let us take the minimal values of thetas :
					
					for(int i=1;i<=nbrRobotVisible;i++)
					{
						float currenttheta = currentmsg.at<float>(1,i);
						
						while(currenttheta > 2*PI)
						{
							currenttheta -= 2*PI;
						}
						while(currenttheta < 0.0f)
						{
							currenttheta += 2*PI;
						}
						
						currentmsg.at<float>(1,i) = currenttheta;
						
						if( currenttheta > 0.0f && currenttheta < mintheta)
						{
							mintheta = currenttheta;
						}
						
					}
					
					//the coupling theta is the mintheta :
					float phi = 0.0f; //it is the current phi of the robot in the target reference-focused frame...
					sumphi += sin(mintheta);
				}

				// FILTER r :
				float rinput = currentmsg.at<float>(0,nbrRobotVisible+1);
				r = (tau-1.0f)*r/tau+rinput/tau;
				
				
				// PID-controller :
				//float P = 1.0f;
				/*
				float desiredPhi = PI/nbrRobotVisible;
				float I = 0.1f;
				this->pid.setKp(P);
				this->pid.setKi(I);
				Mat<float> currval( abs(desiredPhi-mintheta)/PI, 1,1);
				float Kgain = this->pid.update( currval, 0.001).get(1,1);
				*/
				//float Kgain = P*abs(desiredPhi-mintheta)/PI;
				
				float f = this->a*r*(1.0f-(r*r)/(this->R*this->R));
				float g = this->Omega + this->epsilon*sumphi;
				//float g = this->Omega + this->epsilon*(1.0f+Kgain)*sumphi;
				
				
        
				//----------------------------------------------------
				//----------------------------------------------------
				//----------------------------------------------------
				
				
				//float v = Kgain*this->gain*this->kv*(f*cos(THETA) + r*g*sin(THETA));
				float v = this->gain*this->kv*(f*cos(THETA) + r*g*sin(THETA));
				//float omega = Kgain*this->gain*this->kw*(r*g*cos(THETA) - f*sin(THETA));
				float omega = this->gain*this->kw*(r*g*cos(THETA) - f*sin(THETA));
				
				
				//----------------------------------------------------
				//----------------------------------------------------
				//COMPUTE QUANTITIES :
				//----------------------------------------------------
				//----------------------------------------------------
				
				if(this->emergencyBreak)
				{
					v= 0.0f;
					omega=0.0f;
				}
				
				//----------------------------------------------------
				//----------------------------------------------------
				//----------------------------------------------------
				
				//----------------------------------------------------
				//----------------------------------------------------
				// MESSAGE :
				//----------------------------------------------------
				//----------------------------------------------------
				
				
				twistmsg.linear.x = -v;
				twistmsg.linear.y = 0.0f;
				twistmsg.linear.z = 0.0f;
				
				twistmsg.angular.x = 0.0f;
				twistmsg.angular.y = 0.0f;
				twistmsg.angular.z = -omega;
				
				if(this->verbose)
				{
					std::cout << " V x W : " << v << " x " << omega << " // r filtered x input : " << r << " x " << rinput << " // phi_i_i+1 : " << mintheta << std::endl;
				}
				
				//----------------------------------------------------
				//----------------------------------------------------
				//----------------------------------------------------
				
				//----------------------------------------------------
				//----------------------------------------------------
				//----------------------------------------------------
				
				//PUSHING :
				
				
			}
			
			// 10 Hz control loop :
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			//----------------------------
			//		Publisher
			//----------------------------
		
			//--------------------------------------------
			//
			//	Convert CvImage to ROS image message and publish it to camera/image_processed topic
			//
			//--------------------------------------------
			twistpub.publish(twistmsg);
			
			
#ifdef debug_v0		
			count_info++;
	
			if(count_info>10)
			{
				ROS_INFO("OPUSim_ControlLaw::FPS : %f.", CLOCKS_PER_SEC/((float)(clock()-timer)) );
				count_info = 0;
			}
			timer = clock();
#endif

			mutexRES.lock();
		}
		mutexRES.unlock();

	}
	
	inline void setContinuer(bool c)
	{
		mutexRES.lock();
		continuer = c;
		mutexRES.unlock();	
	}
	

};



class ControlBarrierCertificate
{
	private :
	
	float minDist;
	float vmax;
	float amax;
	float gamma;
	cv::Mat A;
	cv::Mat b;
	
	clock_t lastClock;
	float elapsedTime;
	
	
	int nbrObj;
	
	cv::Mat state;
	cv::Mat predState;
	cv::Mat inputState;
	cv::Mat newState;
	
	/*
	Stacked variables :
	- x
	- y
	- isTarget
	- isObstacle
	- vx (except for inputState)
	- vy (except for inputState)
	//TODO : decide about the need of those variables :
	- ax (except for inputState)
	- ay (except for inputState)
	*/
	
	cv::Mat pairs;
	cv::Mat tailoredControlInput;
	
	
	
	
	
	public :
	
	ControlBarrierCertificate(const float& minDist_ = 0.2f, const float& gamma_ = 0.1f,const float& vmax_ = 0.02f,const float& amax_ = 0.002f) : minDist(minDist_), gamma(gamma_), vmax(vmax_), amax(amax_), nbrObj(0), lastClock( clock() )
	{
	
	}
	
	~ControlBarrierCertificate()
	{
	
	}
	
	void observation( const cv::Mat& inputMsg)
	{
		/*
		Deals with the parsing of the message in order to create the inputState :
		Argument(s) :
		- inputMsg : message delivered by OPUSim_RSO... : 
		Architecture of the message :
		first column : nbrRobot, thetaOmnidirectional sensor to target...
		next columns : (r,theta) of the visible neighbours robots.
		last column : radius to target ...
		
		(- inputObs : message delivered by the Obstacle-related sensor...)
		*/
		
		//Let us set up the elapsed time :
		this->elapsedTime = float( clock() - this->lastClock) / CLOCKS_PER_SEC ;
		this->lastClock = clock();
		
		
		bool needToInit = false;
		
		if(this->nbrObj == 0)
		{
			/*
			Then we will need to initialize it all :
			- velocities are initialized to zero...
			*/
			needToInit = true;
		}
	
		//TODO : change that line when we add the message from the obstacle sensor...
		this->nbrObj = inputMsg.cols-1;
	

		/*
		let us go on : we just have to parse the message and create the inputState :
		*/
		
	
		this->inputState = cv::Mat::zeros( cv::Size(4,this->nbrObj), CV_32F);
	
		for(int i=1;i<=this->inputState.cols;i++)
		{
			float r = inputMsg.at<float>(0,i);
			float theta = inputMsg.at<float>(1,i);
		
			float x = r*cos(theta);
			float y = r*sin(theta);
		
			float isTarget = 0.0f;
			float isObstacle = 0.0f;
		
			this->inputState.at<float>(0,i-1) = x;
			this->inputState.at<float>(1,i-1) = y;
			this->inputState.at<float>(2,i-1) = isTarget;
			this->inputState.at<float>(3,i-1) = isObstacle;
		
		}
		
		
		if(needToInit)
		{
			//Using only velocities... :
			//cv::concatv( this->inputState, cv::Mat::zeros( cv::Size(2,this->nbrObj), CV_32F), this->state);
			//Using velocities and accelerations ... :
			cv::vconcat( this->inputState, cv::Mat::zeros( cv::Size(4,this->nbrObj), CV_32F), this->state);
		}
		
	}
	
	void prediction()
	{
		/*
		Deals with the prediction scheme that approximate the current position of the objects observed at the last step, depending on the velocities observed at the last step.
		The state of the art would use an Extended Kalman filter but let us try to make an easy Euler approximation first...
		*/
		
		this->predState = this->state;
		
		for(int i=0;i<this->predState.cols;i++)
		{
			float vx = this->predState.at<float>(4,i);
			float vy = this->predState.at<float>(5,i);
			
			this->predState.at<float>(0,i) += vx*this->elapsedTime;
			this->predState.at<float>(1,i) += vy*this->elapsedTime;
		}
		
	}
	
	void updateState()
	{
		/*
		Deals with the update of the state according to the pairing with the observed objects :
		1) update positions and flags, from inputState...
		2) update velocities, from difference computation...
		//TODO : see if necessary ... :
		3) update accelerations, from difference computation...
		*/
		
		/* 
		Once again, a kalman-like filtering could be use, but let us keep things simple for the moment...
		*/
		
		this->nbrObj = this->pairs.cols;
		
		//Using velocities only :
		//this->newState = cv::Mat::zeros( cv::Size(6,this->nbrObj), CV_32F);
		//Using velocities and accelerations ... :
		this->newState = cv::Mat::zeros( cv::Size(8,this->nbrObj), CV_32F);
		
		for(int i=0;i<this->nbrObj;i++)
		{
			int idxInput = int( this->pairs.at<float>(0,i) );
			int idxOld = int( this->pairs.at<float>(1,i) );
			//predState and (old) state indexes are the same...
			
			//Update positions :
			this->newState.at<float>( 0, i) = this->inputState.at<float>(0,idxInput);
			this->newState.at<float>( 1, i) = this->inputState.at<float>(1,idxInput);
			//Update flags :
			this->newState.at<float>( 2, i) = this->inputState.at<float>(2,idxInput);
			this->newState.at<float>( 3, i) = this->inputState.at<float>(3,idxInput);
			
			//Update velocities :
			this->newState.at<float>( 4, i) = ( this->newState.at<float>(0,i) - this->state.at<float>(0,idxOld) )/this->elapsedTime;
			this->newState.at<float>( 5, i) = ( this->newState.at<float>(1,i) - this->state.at<float>(1,idxOld) )/this->elapsedTime;
			
			//Update accelerations :
			this->newState.at<float>( 6, i) = ( this->newState.at<float>(4,i) - this->state.at<float>(4,idxOld) )/this->elapsedTime;
			this->newState.at<float>( 7, i) = ( this->newState.at<float>(5,i) - this->state.at<float>(5,idxOld) )/this->elapsedTime;
			
		}
				
	}
	
	void optimize( const cv::Mat& desiredControlInput)
	{
		/*
		Run the optimization process of the Quadratic Problem...
		Argument(s) :
		- desiredVelocity : vector (v,omega) of the desired control law we want the current robot to follow.
		Output(s) :
		- tailoredVelocity : vector (v,w) optimized to fulfill our objectives. 
		*/
		
		this->tailoredControlInput = desiredControlInput;
		
		
	}
	
	cv::Mat run( const cv::Mat& desiredControlInput )
	{
		/*
		Deals with the computation of a safe velocity vector in order to fulfill the following of the desiredVelocity and the obstacle avoidance task.
		Argument(s) :
		- desiredVelocity : vector (v,omega) of the desired control law we want the current robot to follow.
		Output(s) :
		- tailoredVelocity : vector (v,w) optimized to fulfill our objectives. 
		*/
		
		//Let us compute the predicted state :
		this->prediction();
		
		//Let us pair those observations with the predicted state :
		this->pairing();
		
		//Let us update the newState from the pairs and the inputState:
		this->updateState();
		
		//Let us run the optimization problem :
		this->optimize( desiredControlInput);
		
		return this->tailoredControlInput;
	}
	
	
	
	/*-------------------------------------------------*/
	//		HELPER FUNCTIONS :
	/*-------------------------------------------------*/
	
	
	void pairing()
	{
		/*
		Deals with the pairing of the newly observed objects and those observed at the previous states. 
		It then proceed by computing the velocities when possible (that is to say when there was a pairing).
		*/
		
		//TODO : ensure that we have neighbours....!
		
		//Let us compute the distance matrix between the predicted state and the input state :
		cv::Mat distM = cv::Mat::zeros( cv::Size(this->inputState.cols,this->predState.cols), CV_32F);
		
		for(int i=0;i<distM.rows;i++)
		{
			for(int j=0;j<distM.cols;j++)
			{
				//distM.at<float>(i,j) = this->computeMahalanobisDistance( slice(this->inputState, std::vector<float>(1,-1.0f), std::vector<float>(1, i) ), slice(this->predState, std::vector<float>(1,-1.0f), std::vector<float>(1, j) ) ) 
				cv::Range r1[2] = {cv::Range::all(), cv::Range(i,i+1)};
				cv::Range r2[2] = {cv::Range::all(), cv::Range(j,j+1)};
				distM.at<float>(i,j) = this->computeMahalanobisDistance( cv::Mat(this->inputState, r1), cv::Mat(this->predState, r2) );
			}
		}
		
		//Let us pair those fellows by searching for the minimal distance until we reach the minimum between the number of newly observed objected and the number of previously observed object, as long as there are some :
		int minimalNbrObj = this->nbrObj;
		if( minimalNbrObj < this->predState.cols)
		{
			minimalNbrObj = this->predState.cols;
		}
		
		
		this->pairs = cv::Mat::zeros( cv::Size(2, minimalNbrObj), CV_32F); 	//from predState objects to inputState objects
		int size = 0;
		
		while( size < minimalNbrObj )
		{
			double dummy;
			double maxElimination;
			cv::Point min_loc, dummy_loc;
			cv::minMaxLoc(distM, &dummy, &maxElimination, &min_loc, &dummy_loc);
			
			//let us eliminate the pair :
			for(int i=0;i<this->inputState.cols;i++)
			{
				distM.at<float>(i, min_loc.y ) = maxElimination;
			}
			for(int j=0;j<this->predState.cols;j++)
			{
				distM.at<float>( min_loc.x, j ) = maxElimination;
			}
			
			//let us record the pair :
			this->pairs.at<float>(0,size) = min_loc.x;
			this->pairs.at<float>(1,size) = min_loc.y;
			size++;
		}
		
	}
	
	
	
	float computeMahalanobisDistance( cv::Mat x, cv::Mat z )
	{
		float ret = 0.0f;
		
		for(int i=0;i<x.rows;i++)
		{
				ret += pow( x.at<float>(i,0) - z.at<float>(i,0), 2.0f);
		}
		
		return sqrt(ret);
	}
	
	/*
	cv::Mat slice( const cv::Mat& x, const std::vector<float>& sr, const std::vector<float>& sc)
	{
		cv::Mat ret;
		
		if( sr.size() == 1)
		{
			if( sr[0] == -1)
			{
				
			}
			else
			{
			
			}
		}
		else if(sc.size() == 1)
		{
		
		}
		else
		{
			//TODO : not needed for the moment though...
		}
		
		return ret;
	}
	*/

};

#endif
