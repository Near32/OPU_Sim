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
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

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
    /*
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
    */
    return std::atan2(y,x);

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


float quaternion2Theta(const geometry_msgs::Quaternion& q)
{
	return std::atan2( 2.0f*(q.w*q.z + q.x*q.y), 1.0f-2.0f*(q.y*q.y+q.z*q.z) ); 
}

cv::Mat removeColumn( const cv::Mat& m, int idxC)
{
	int nbrCols = m.cols;
	cv::Mat ret = cv::Mat::zeros(1,1,CV_32F);
	
	if( nbrCols > idxC)
	{
		if(idxC != 0)
		{
			cv::Range r1[2] = {cv::Range::all(), cv::Range(0,idxC)};
			ret = cv::Mat( m, r1);
			
			if( idxC+1 < nbrCols)
			{
				cv::Range r2[2] = {cv::Range::all(), cv::Range(idxC+1,nbrCols)};
				cv::hconcat( ret, cv::Mat(m, r2), ret);
			}
		}
		else
		{
			if( idxC+1 < nbrCols)
			{
				cv::Range r2[2] = {cv::Range::all(), cv::Range(idxC+1,nbrCols)};
				ret = cv::Mat(m, r2);
			}
		}
		
	}
	
	return ret;
}

cv::Mat extractColumn( const cv::Mat& m, int idxC)
{
	int nbrCols = m.cols;
	cv::Mat ret;
	
	if( nbrCols > idxC)
	{
		cv::Range r1[2] = {cv::Range::all(), cv::Range(idxC,idxC+1)};
		ret = cv::Mat( m, r1);		
	}
	
	return ret;
}





class RepulsionMapMetaControlLaw
{
	private :
	
	float tresholdDist;
	float tresholdFarEnough;
	
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
	- z
	- isTarget
	- isObstacle
	- vx 
	- vy 
	- vz 
	//TODO : decide about the need of those variables :
	- ax 
	- ay 
	- az
	*/
	
	cv::Mat pairs;
	cv::Mat tailoredControlInput;
	
	bool needToOptimize;
	bool pairingToDo;
	
	int nbrAngularIntervals;
	cv::Mat angularIntervals;
	cv::Mat Interval2Bias;
	
	//Odom :
	cv::Mat currentOdometry;
	//debug :
	ros::NodeHandle* pnh;
	ros::Publisher cloud_pub;
	
	public :
	
	RepulsionMapMetaControlLaw(const float& tresholdDist_ = 1.0f, const float& tresholdFarEnough_ = 5.0f) : nbrObj(0), lastClock( clock() ), needToOptimize(false), tresholdDist(tresholdDist_),currentOdometry(cv::Mat::zeros(2,1,CV_32F)), pnh(NULL), pairingToDo(false), tresholdFarEnough(tresholdFarEnough_)
	{
		this->nbrAngularIntervals = 8;
		this->angularIntervals = cv::Mat::zeros( this->nbrAngularIntervals, 2, CV_32F);
		//upper bounds :
		this->angularIntervals.at<float>(0,0) = PI/2;
		this->angularIntervals.at<float>(0,1) = 3*PI/12;
		this->angularIntervals.at<float>(0,2) = 2*PI/12;
		this->angularIntervals.at<float>(0,3) = 1*PI/12;
		this->angularIntervals.at<float>(0,4) = 0.0f;
		this->angularIntervals.at<float>(0,5) = -1*PI/12;
		this->angularIntervals.at<float>(0,6) = -2*PI/12;
		this->angularIntervals.at<float>(0,7) = -3*PI/12;
		//lower bounds :
		this->angularIntervals.at<float>(1,7) = -PI/2;
		this->angularIntervals.at<float>(1,6) = -3*PI/12;
		this->angularIntervals.at<float>(1,5) = -2*PI/12;
		this->angularIntervals.at<float>(1,4) = -1*PI/12;
		this->angularIntervals.at<float>(1,3) = 0.0f;
		this->angularIntervals.at<float>(1,2) = 1*PI/12;
		this->angularIntervals.at<float>(1,1) = 2*PI/12;
		this->angularIntervals.at<float>(1,0) = 3*PI/12;
		
		this->Interval2Bias = cv::Mat::zeros( this->nbrAngularIntervals, 1, CV_32F);
		float testBias = 2*PI/24;
		this->Interval2Bias.at<float>(0,0) = -PI/12-testBias;
		this->Interval2Bias.at<float>(0,1) = -PI/12-testBias;
		this->Interval2Bias.at<float>(0,2) = -2*PI/12-testBias;
		this->Interval2Bias.at<float>(0,3) = -3*PI/12-testBias;
		this->Interval2Bias.at<float>(0,4) = 3*PI/12+testBias;
		this->Interval2Bias.at<float>(0,5) = 2*PI/12+testBias;
		this->Interval2Bias.at<float>(0,6) = PI/12+testBias;
		this->Interval2Bias.at<float>(0,7) = PI/12+testBias;
	}
	
	~RepulsionMapMetaControlLaw()
	{
	
	}
	
	void setPNH( ros::NodeHandle* pnh)
	{
		this->pnh = pnh;
		
		this->cloud_pub = this->pnh->advertise<sensor_msgs::PointCloud2>("/DEBUG/MAP", 1);
	}
	
	void setTresholdDistAccount( const float& tresholdDistAccount)
	{
		this->tresholdDist = tresholdDistAccount;
	}
	
	void setTresholdDistFarEnough( const float& tresholdDistFarEnough)
	{
		this->tresholdFarEnough = tresholdDistFarEnough;
	}
	
	void observation( const cv::Mat& inputMsg)
	{
		/*
		Deals with the parsing of the message in order to create the inputState :
		Argument(s) :
		- inputMsg : message delivered by RP_RO... : 
		Architecture of the message :
		first column : nbrObject, 0
		next columns : (r,theta) of the visible objects : in the current robot frame.
		//TODO : make the modification in RP_RelativeOdometry...
		last column : (radius, thetaOmnidirectional) sensor to target, if any...
		
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
		this->nbrObj = inputMsg.at<float>(0,0);
	
		if( this->nbrObj == 0)
		{
			this->needToOptimize = false;
			return ;
		}
		else
		{
			this->needToOptimize = true;
		}
		
		
		/*
		let us go on : we just have to parse the message and create the inputState :
		*/
		
	
		this->inputState = cv::Mat::zeros( 4, this->nbrObj, CV_32F);
	
		for(int i=1;i<=this->inputState.cols;i++)
		{
			float r = inputMsg.at<float>(0,i);
			float theta = inputMsg.at<float>(1,i);
		
			float x = r*cos(theta);
			float y = r*sin(theta);
		
			float isTarget = (inputMsg.at<float>(2,i) == 1.0f ? 1.0f : 0.0f);
			float isObstacle = 1.0f;
		
			this->inputState.at<float>(0,i-1) = x;
			this->inputState.at<float>(1,i-1) = y;
			this->inputState.at<float>(2,i-1) = isTarget;
			this->inputState.at<float>(3,i-1) = isObstacle;
		
		}
		
		//std::cout << "INPUT STATE : " << this->inputState << std::endl;
		
		
		if(needToInit)
		{
			//Using only velocities... :
			cv::vconcat( this->inputState, cv::Mat::zeros( cv::Size(this->nbrObj,3), CV_32F), this->state);
			//Using velocities and accelerations ... :
			//cv::vconcat( this->inputState, cv::Mat::zeros( cv::Size(this->nbrObj,6), CV_32F), this->state);
		}
		
	}
	
	
	
	void observationDepthObs( const cv::Mat& inputMsg)
	{
		/*
		Deals with the parsing of the message in order to create the inputState with respect to obstacles seen by the Depth sensor :
		Argument(s) :
		- inputMsg : message delivered by RP_RO... : 
		Architecture of the message :
		first column : nbrObject, 0
		next columns : (x,y,z) of the visible objects : in the current robot frame.
		*/
		
		//Let us set up the elapsed time :
		this->elapsedTime = float( clock() - this->lastClock) / CLOCKS_PER_SEC ;
		this->lastClock = clock();
		
		bool needToInit = false;
		if(this->nbrObj == 0)
		{
			needToInit = true;
		}
	
		//TODO : change that line when we add the message from the obstacle sensor...
		int nbrObjNearEnough = 0;
		std::vector<bool> nearEnough(inputMsg.cols,false);
		//beware : the first column is not a point...
		for(int i=1;i<inputMsg.cols;i++)
		{
			float x = inputMsg.at<float>(0,i);
			float y = inputMsg.at<float>(1,i);
			float z = inputMsg.at<float>(2,i);
			
			float dist = sqrt(x*x*+z*z);
			
			if( dist < this->tresholdFarEnough)
			{
				nearEnough[i] = true;
				nbrObjNearEnough++;
			}
		}
		
		this->nbrObj = nbrObjNearEnough;
	
		if( this->nbrObj == 0)
		{
			this->needToOptimize = false;
			return ;
		}
		else
		{
			this->needToOptimize = true;
		}
		
		
		/*
		let us go on : we just have to parse the message and create the inputState :
		*/
		this->inputState = cv::Mat::zeros( 5, this->nbrObj, CV_32F);
		int icountState = 0;
		
		for(int i=1;i<=inputMsg.cols;i++)
		{
			if( nearEnough[i] )
			{
				float x = inputMsg.at<float>(0,i);
				float y = inputMsg.at<float>(1,i);
				float z = inputMsg.at<float>(2,i);
	
				float isTarget = 0.0f;
				float isObstacle = 1.0f;
	
				this->inputState.at<float>(0,icountState) = x;
				this->inputState.at<float>(1,icountState) = y;
				this->inputState.at<float>(2,icountState) = z;
				this->inputState.at<float>(3,icountState) = isTarget;
				this->inputState.at<float>(4,icountState) = isObstacle;
				
				icountState++;
			}
		
		}
		
		//std::cout << "INPUT STATE : " << this->inputState << std::endl;
		
		//Using only velocities... :
		cv::vconcat( this->inputState, cv::Mat::zeros( cv::Size(this->nbrObj,3), CV_32F), this->inputState);
		//Using velocities and accelerations ... :
		//cv::vconcat( this->inputState, cv::Mat::zeros( cv::Size(this->nbrObj,6), CV_32F), this->inputState);
		
		if(needToInit)
		{
			this->state = this->inputState;
			this->predState = this->inputState;
			this->newState = this->inputState;
		}
		
		this->pairingToDo = true;
		
	}
	
	void observeOdometry( const cv::Mat& odo)
	{
		odo.copyTo(this->currentOdometry);
	}
	
	void predictionSimple()
	{
		/*
		Deals with the prediction scheme that approximate the current position of the objects observed at the last step, depending on the velocities observed at the last step.
		The state of the art would use an Extended Kalman filter but let us try to make an easy Euler approximation first...
		*/
		
		if(this->needToOptimize)
		{
			//NO VELOCITY for the moment...
			/*
			this->state = this->inputState;
			this->predState = this->state;
			*/
		
			this->predState = this->state;
			// WITH VELOCITIES :
			for(int i=0;i<this->predState.cols;i++)
			{
				float vx = this->predState.at<float>(5,i);
				float vy = this->predState.at<float>(6,i);
				float vz = this->predState.at<float>(7,i);
			
				this->predState.at<float>(0,i) += vx*this->elapsedTime;
				this->predState.at<float>(1,i) += vy*this->elapsedTime;
				this->predState.at<float>(2,i) += vz*this->elapsedTime;
			}
			
		}
		
		this->transformSimple();
		
	}
	
	void transformSimple()
	{
		/*
		Deals with the transformation that the point of cloud should undergo because of the movements of the robot :
		- use the odometry observations currentOdometry
		*/
		//Let us apply the rotation around the y axis (that is the z-axis in the frame of the robot)...
		
		float deltaTheta = currentOdometry.at<float>(1,0)*this->elapsedTime;
		float deltaR = currentOdometry.at<float>(0,0)*this->elapsedTime;
		
		for(int i=0;i<this->predState.cols;i++)
		{
			float x = this->predState.at<float>(0,i);
			float y = this->predState.at<float>(2,i);
			
			float r = sqrt( x*x + y*y);
			float theta = std::atan2(y,x);
			
			r += deltaR;
			theta += deltaTheta;
			
			x = cos(theta)*r;
			y = sin(theta)*r;
			
			this->predState.at<float>(0,i) = x;
			this->predState.at<float>(2,i) = y;
			
		}
		
	}
	
	
	void updateState()
	{
		/*
		Deals with the update of the state according to the pairing with the observed objects :
		1) update positions and flags, from inputState...
		2) update velocities, from difference computation...
		//TODO : see if necessary later... :
		3) update accelerations, from difference computation...
		*/
		
		/* 
		Once again, a kalman-like filtering could be use, but let us keep things simple for the moment...
		*/
		
		this->nbrObj = this->pairs.cols;
		
		//Using velocities only :
		this->newState = this->predState;
		float mvx = 0.0f;
		float mvy = 0.0f;
		float mvz = 0.0f;
		
		std::cout << "NBR PAIR : " << this->nbrObj << std::endl;
		
		//let us update old positions :
		for(int i=0;i<this->nbrObj;i++)
		{
			int idxInput = int( this->pairs.at<float>(0,i) );
			int idxOld = int( this->pairs.at<float>(1,i) );
			//predState and (old) state indexes are the same...
			
			//Update positions :
			this->newState.at<float>( 0, idxOld) = this->inputState.at<float>(0,idxInput);
			this->newState.at<float>( 1, idxOld) = this->inputState.at<float>(1,idxInput);
			this->newState.at<float>( 2, idxOld) = this->inputState.at<float>(2,idxInput);
			//Update flags :
			this->newState.at<float>( 3, idxOld) = this->inputState.at<float>(3,idxInput);
			this->newState.at<float>( 4, idxOld) = this->inputState.at<float>(4,idxInput);
			
			//Update velocities :
			/*
			this->newState.at<float>( 5, idxOld) = -this->currentVelocity.at<float>(0,0) + ( this->newState.at<float>(0,idxOld) - this->state.at<float>(0,idxOld) )/this->elapsedTime;
			this->newState.at<float>( 6, idxOld) = -this->currentVelocity.at<float>(1,0) + ( this->newState.at<float>(1,idxOld) - this->state.at<float>(1,idxOld) )/this->elapsedTime;
			this->newState.at<float>( 7, idxOld) = -this->currentVelocity.at<float>(2,0) + ( this->newState.at<float>(2,idxOld) - this->state.at<float>(2,idxOld) )/this->elapsedTime;
			*/
			this->newState.at<float>( 5, idxOld) = ( this->newState.at<float>(0,idxOld) - this->predState.at<float>(0,idxOld) )/this->elapsedTime;
			this->newState.at<float>( 6, idxOld) = ( this->newState.at<float>(1,idxOld) - this->predState.at<float>(1,idxOld) )/this->elapsedTime;
			this->newState.at<float>( 7, idxOld) = ( this->newState.at<float>(2,idxOld) - this->predState.at<float>(2,idxOld) )/this->elapsedTime;
			
			mvx += this->newState.at<float>(5,idxOld);
			mvy += this->newState.at<float>(6,idxOld);
			mvz += this->newState.at<float>(7,idxOld);
			
			//Update accelerations :
			/*
			this->newState.at<float>( 6, i) = ( this->newState.at<float>(4,i) - this->state.at<float>(4,idxOld) )/this->elapsedTime;
			this->newState.at<float>( 7, i) = ( this->newState.at<float>(5,i) - this->state.at<float>(5,idxOld) )/this->elapsedTime;
			*/
		}
		
		std::cout << " MEAN VX : " << mvx << std::endl;
		std::cout << " MEAN VY : " << mvy << std::endl;
		std::cout << " MEAN VZ : " << mvz << std::endl;
		
		
		//let us remove the positions that are far enough of the robot :
		for(int i=0;i<this->newState.cols;i++)
		{
			float x = this->newState.at<float>(0,i);
			float y = this->newState.at<float>(1,i);
			float z = this->newState.at<float>(2,i);
			
			float dist = sqrt(x*x+z*z);
			
			if( dist > this->tresholdFarEnough)
			{
				if(this->newState.cols-1)
				{					
					this->newState = removeColumn(this->newState, i);
					i--;
				}
				else
				{
					//we want to delete the last remaining column, it will bring an exception...
					this->newState = cv::Mat::zeros(this->newState.rows, 1, CV_32F);
					i=1;
					break;
				}
			}
		}
		
		//let us add new positions, previously unseen / unmatched positions out of the inputState.
		// what are those positions :
		std::vector<int> unmatchedIdx(this->inputState.cols, 0);
		int nbrUnmatched = 0;
		for(int i=0;i<this->inputState.cols;i++)
		{
			bool isNotWithinMatched = true;
			
			for(int j=0;j<this->pairs.cols;j++)
			{
				if( this->pairs.at<float>(0,j) == i)
				{
					isNotWithinMatched = false;
					break;
				}
			}
			
			if(isNotWithinMatched)
			{
				unmatchedIdx[nbrUnmatched] = i;
				nbrUnmatched++;
			}
		}
		
		//std::cout	<< " INPUT STATE : " <<	this->inputState << std::endl;
		//std::cout	<< " NEW STATE : " <<	this->newState << std::endl;
		
		
		//let us add those to newState :
		//cv::Mat addAll = cv::Mat::zeros( this->inputState.rows, unmatchedIdx.size(), CV_32F);
		
		for(int i=0;i<nbrUnmatched;i++)
		{
			//cv::Range r[2] = {cv::Range::all(), cv::Range(i,i+1)};
			//addAll(r) = extractColumn( this->inputState, unmatchedIdx[i]);
			
			cv::Mat add;
			extractColumn( this->inputState, unmatchedIdx[i]).copyTo(add);
			//std::cout << " unmatched idx : " << unmatchedIdx[i] << std::endl;
			//std::cout	<< " added STATE : " <<	add << std::endl;
			
			cv::hconcat( this->newState, add, this->newState);
		}
		
		//cv::hconcat(this->newState, addAll, this->newState);
				
		
		//let us actually update the state :
		this->nbrObj = this->newState.cols;
		this->state = this->newState;
		this->predState = this->newState;
		
		
		
				
	}
	
	
	void optimizeSimple( const cv::Mat& desiredControlInput)
	{
		/*
		Create the repulsion/attraction map from the positions of the obstacles retrieved by the RGBD sensor.
		Argument(s) :
		- desiredControlInput = desiredVelocity : vector (v,omega) of the desired control law we want the current robot to follow.
		Output(s) :
		- tailoredControlInput = tailoredVelocity : vector (v, omega) optimized to fulfill our objectives. 
		*/
		
		if(this->needToOptimize == false)
		{
			this->tailoredControlInput = desiredControlInput;
		}
		else
		{
			bool isThereRelevantObstacles = false;
			float angularObstacleInduicedBias = 0.0f;
			//this variable will be incremented or decremented with respect to the angular positions of obstacles.
			
			cv::Mat angularSparsePositions = cv::Mat::zeros( this->nbrAngularIntervals, 1, CV_32F);
			//this table will be filled up whenever there is an obstacle in the corresponding angular interval.
			
			
			
			//std::cout << "STATE : " << this->predState << std::endl;
			
			for(int i=0;i<this->nbrObj;i++)
			{
				float dist = sqrt(pow( this->predState.at<float>(0,i), 2) + pow( this->predState.at<float>(2,i), 2))+1e-3f;
				
				//std::cout << " DIST : " << dist << std::endl;
				
				if( dist < tresholdDist && dist > 0.001f)
				{
					isThereRelevantObstacles = true;
					
					float angleObstacle = std::atan2( this->predState.at<float>(2,i), this->predState.at<float>(0,i) ) - PI/2;
				
					//std::cout << " ANGLE : " << angleObstacle * 180.0f/PI << std::endl;
					
					for(int j=this->nbrAngularIntervals;j--;)
					{
						if( angleObstacle < angularIntervals.at<float>(0,j) && angleObstacle > angularIntervals.at<float>(1,j) )
						{
							angularSparsePositions.at<float>(0,j) += 1.0f;
							break;
						}
					}
					
				}
			}
			
			//let us increment and decrement the bias :
			for(int i=this->nbrAngularIntervals;i--;)
			{
				if( angularSparsePositions.at<float>(0,i) > 0.0f)
				{
					angularObstacleInduicedBias += this->Interval2Bias.at<float>(0,i);
				}
			}
			
			//let us verify that the current situation is regular :
			if( isThereRelevantObstacles && angularObstacleInduicedBias == 0.0f )
			{
				//then it is irregular, obstacles have cancelled each other...
				//let us regularize the solution by always going on the left :
				angularObstacleInduicedBias = +PI/2;
			}
			
			
			this->tailoredControlInput = cv::Mat::zeros( 2, 1, CV_32F);
			this->tailoredControlInput.at<float>(0,0) = desiredControlInput.at<float>(0,0);//v;
			this->tailoredControlInput.at<float>(1,0) = angularObstacleInduicedBias + desiredControlInput.at<float>(1,0);//omega;
			
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
	
	void publishMAP()
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		sensor_msgs::PointCloud2 cloudmsg;
		
		cloud.points.push_back( pcl::PointXYZ(0.0f,0.0f,0.0) );
		//x
		cloud.points.push_back( pcl::PointXYZ(1.0f,0.0f,0.0) );
		//y
		cloud.points.push_back( pcl::PointXYZ(0.0f,1.0f,0.0) );
		//z
		cloud.points.push_back( pcl::PointXYZ(0.0f,0.0f,1.0) );
		
		for(int i=0;i<this->state.cols;i++)
		{
			float x = this->state.at<float>(0,i);
			float y = this->state.at<float>(1,i);
			float z = this->state.at<float>(2,i);
			
			pcl::PointXYZ p(x,y,z);
			
			cloud.points.push_back(p);
			
		}
		
		pcl::toROSMsg(cloud, cloudmsg);
		
		//header :
		cloudmsg.header.frame_id = "PointCloudFrame";
		
		cloud_pub.publish(cloudmsg);
	}
	
	cv::Mat run( const cv::Mat& desiredControlInput, const bool& optimize=true )
	{
		/*
		Deals with the computation of a safe velocity vector in order to fulfill the following of the desiredVelocity and the obstacle avoidance task.
		Argument(s) :
		- desiredControlInput = 
			simple :: desiredVelocity : vector (v,omega) of the desired control law we want the current robot to follow.
			complex :: desiredAcceleration maybe... //TODO
		Output(s) :
		- tailoredControlInput =
			simple :: tailoredVelocity : vector (v,w) optimized to fulfill our objectives. 
			complex :: tailoredAcceleration maybe... //TODO
		*/
		
		if(this->needToOptimize)
		{
			this->needToOptimize = optimize;
		}
		
		//Let us compute the predicted state :
		this->predictionSimple();
		
		if(this->pairingToDo)
		{
			//Let us pair those observations with the predicted state :
			this->pairing();
			this->pairingToDo = false;
			
			//Let us update the newState from the pairs and the inputState and predState:
			this->updateState();
		}
		else
		{
			this->newState = this->predState;
			this->state = this->newState;
		}
		
		
		
		//debug : publish map :
		this->publishMAP();
		
		//Let us run the optimization problem :
		this->optimizeSimple( desiredControlInput);
		//this->optimize( desiredControlInput);
		
		return this->tailoredControlInput;
	}
	
	
	
	/*-------------------------------------------------*/
	//		HELPER FUNCTIONS :
	/*-------------------------------------------------*/
	
	
	void pairing()
	{
		/*
		Deals with the pairing of the newly observed objects inputState and those observed at the previous states after prediction of the new positions : predState. 
		It then proceed by computing the velocities when possible (that is to say when there was a pairing).
		*/
		
		//TODO : ensure that we have neighbours....!
		
		//Let us compute the distance matrix between the predicted state and the input state :
		cv::Mat distM = cv::Mat::zeros( this->inputState.cols,this->predState.cols, CV_32F);
		
		for(int i=0;i<distM.rows;i++)
		{
			for(int j=0;j<distM.cols;j++)
			{
				//distM.at<float>(i,j) = this->computeMahalanobisDistance( slice(this->inputState, std::vector<float>(1,-1.0f), std::vector<float>(1, i) ), slice(this->predState, std::vector<float>(1,-1.0f), std::vector<float>(1, j) ) ) 
				cv::Range r1[2] = {cv::Range(0,3), cv::Range(i,i+1)};
				cv::Range r2[2] = {cv::Range(0,3), cv::Range(j,j+1)};
				distM.at<float>(i,j) = this->computeMahalanobisDistance( cv::Mat(this->inputState, r1), cv::Mat(this->predState, r2) );
			}
		}
		
		//Let us pair those fellows by searching for the minimal distance until we reach the minimum between the number of newly observed objected and the number of previously observed object, as long as there are some :
		int minimalNbrObj = this->nbrObj;
		if( minimalNbrObj > this->predState.cols)
		{
			minimalNbrObj = this->predState.cols;
		}
		
		
		this->pairs = cv::Mat::zeros( 2, minimalNbrObj, CV_32F); 	//from predState objects to inputState objects
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
				distM.at<float>(i, min_loc.x ) = maxElimination;
			}
			for(int j=0;j<this->predState.cols;j++)
			{
				distM.at<float>( min_loc.y, j ) = maxElimination;
			}
			
			//let us record the pair :
			this->pairs.at<float>(0,size) = min_loc.y;
			this->pairs.at<float>(1,size) = min_loc.x;
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
