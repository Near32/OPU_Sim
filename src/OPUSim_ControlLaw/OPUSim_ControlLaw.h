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
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
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
#include "../RunningStats/RunningStats.h"

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





class MetaControlLaw
{
	private :
	
	float tresholdDist;
	float tresholdFarEnough;
	
	float kuramotoDistance;
	float kuramotoTau;
	bool kuramotoCurrentlyAvoiding;
	float minObstacleDistance;
	float obstacleAngle;
	
	clock_t lastClock;
	float elapsedTime;
	
	
	int nbrObj;
	
	cv::Mat state;
	cv::Mat predState;
	
	cv::Mat inputObsState;
	cv::Mat inputRobotState;
	
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
	float tresholdDistPair;
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
	
	MetaControlLaw(const float& tresholdDist_ = 0.5f, const float& tresholdFarEnough_ = 2.0f, const float& tresholdDistPair_ = 1.0f) : nbrObj(0), lastClock( clock() ), needToOptimize(false), tresholdDist(tresholdDist_),currentOdometry(cv::Mat::zeros(2,1,CV_32F)), pnh(NULL), pairingToDo(false), tresholdFarEnough(tresholdFarEnough_), tresholdDistPair(tresholdDistPair_)
	{
	
		this->kuramotoDistance = 0.0f;
		this->kuramotoTau = 10.0f;
		this->kuramotoCurrentlyAvoiding = false;
		this->obstacleAngle = 0.0f;
		this->minObstacleDistance = this->tresholdDist;
		
		this->nbrAngularIntervals = 10;
		this->angularIntervals = cv::Mat::zeros( this->nbrAngularIntervals, 2, CV_32F);
		//upper bounds :
		this->angularIntervals.at<float>(0,0) = PI;
		this->angularIntervals.at<float>(0,1) = PI/2;
		this->angularIntervals.at<float>(0,2) = 3*PI/12;
		this->angularIntervals.at<float>(0,3) = 2*PI/12;
		this->angularIntervals.at<float>(0,4) = 1*PI/12;
		this->angularIntervals.at<float>(0,5) = 0.0f;
		this->angularIntervals.at<float>(0,6) = -1*PI/12;
		this->angularIntervals.at<float>(0,7) = -2*PI/12;
		this->angularIntervals.at<float>(0,8) = -3*PI/12;
		this->angularIntervals.at<float>(0,9) = -PI/2;
		//lower bounds :
		this->angularIntervals.at<float>(1,9) = -PI;
		this->angularIntervals.at<float>(1,8) = -PI/2;
		this->angularIntervals.at<float>(1,7) = -3*PI/12;
		this->angularIntervals.at<float>(1,6) = -2*PI/12;
		this->angularIntervals.at<float>(1,5) = -1*PI/12;
		this->angularIntervals.at<float>(1,4) = 0.0f;
		this->angularIntervals.at<float>(1,3) = 1*PI/12;
		this->angularIntervals.at<float>(1,2) = 2*PI/12;
		this->angularIntervals.at<float>(1,1) = 3*PI/12;
		this->angularIntervals.at<float>(1,0) = PI/2;
		
		this->Interval2Bias = cv::Mat::zeros( this->nbrAngularIntervals, 1, CV_32F);
		float testBias = 0.0f;
		//float testBias = PI/12;
		this->Interval2Bias.at<float>(0,0) = 0.0f-testBias;
		this->Interval2Bias.at<float>(0,1) = -PI/12-testBias;
		this->Interval2Bias.at<float>(0,2) = -PI/12-testBias;
		this->Interval2Bias.at<float>(0,3) = -2*PI/12-testBias;
		this->Interval2Bias.at<float>(0,4) = -3*PI/12-testBias;
		this->Interval2Bias.at<float>(0,5) = 3*PI/12+testBias;
		this->Interval2Bias.at<float>(0,6) = 2*PI/12+testBias;
		this->Interval2Bias.at<float>(0,7) = PI/12+testBias;
		this->Interval2Bias.at<float>(0,8) = PI/12+testBias;
		this->Interval2Bias.at<float>(0,9) = 0.0f+testBias;
	}
	
	~MetaControlLaw()
	{
	
	}
	
	void setPNH( ros::NodeHandle* pnh, const int& robot_number=0)
	{
		this->pnh = pnh;
		
		this->cloud_pub = this->pnh->advertise<sensor_msgs::PointCloud2>(std::string("/DEBUG/MAP"+std::to_string(robot_number)).c_str(), 1);
	}
	
	void setTresholdDistAccount( const float& tresholdDistAccount)
	{
		this->tresholdDist = tresholdDistAccount;
	}
	
	void setTresholdDistFarEnough( const float& tresholdDistFarEnough)
	{
		this->tresholdFarEnough = tresholdDistFarEnough;
	}
	
	void setTresholdDistPair( const float& tresholdDistPair)
	{
		this->tresholdDistPair = tresholdDistPair;
	}
	
	void observation( const cv::Mat& inputMsg)
	{
		/*
		Deals with the parsing of the message in order to create the inputState with respect to obstacles seen by the Depth sensor :
		Argument(s) :
		- inputMsg : message delivered by OPUSim_RO... : 
		ARCHITECTURE OF RESULT :
		column1 : nbrRobotVisible thetatarget
		next columns : radius theta of visible robot in target frame.
		last column : radius 0  of reference robot in target frame....
		
		next : the same for the previous pictures..
		
		//Architecture of the message :
		//first column : nbrObject, 0
		//next columns : (x,y,z) of the visible objects : in the current robot frame.
		*/
		
		/*
		//Let us set up the elapsed time :
		if( !this->pairingToDo)
		{
			this->elapsedTime = float( clock() - this->lastClock) / CLOCKS_PER_SEC ;
			this->lastClock = clock();
		}
		*/
		
		bool needToInit = false;
		if(this->nbrObj == 0)
		{
			needToInit = true;
		}
	
		//TODO : change that line when we add the message from the obstacle sensor...
		int nbrObjNearEnough = 0;
		std::vector<bool> nearEnough(inputMsg.cols/2,false);
		//beware : the first column is not a point...
		for(int i=1;i<inputMsg.cols/2;i++)
		{
			float dist = inputMsg.at<float>(0,i);
			
			if( dist < this->tresholdFarEnough)
			{
				nearEnough[i] = true;
				nbrObjNearEnough++;
			}
		}
		
		this->nbrObj = nbrObjNearEnough;
	
		if( this->nbrObj == 0)
		{
			return ;
		}
		else
		{
			this->needToOptimize = true;
		}
		
		
		/*
		let us go on : we just have to parse the message and create the inputState :
		*/
		this->inputRobotState = cv::Mat::zeros( 5, this->nbrObj, CV_32F);
		int icountState = 0;
		
		for(int i=1;i<=inputMsg.cols/2;i++)
		{
			if( nearEnough[i] )
			{
				float r = inputMsg.at<float>(0,i);
				float theta = inputMsg.at<float>(1,i);

				//frame is x<-x,y<-z,z<--y
				float x = r*cos(theta);
				float y = 0;
				float z = r*sin(theta);
	
				float isTarget = 0.0f;
				float isObstacle = 1.0f;
	
				this->inputRobotState.at<float>(0,icountState) = x;
				this->inputRobotState.at<float>(1,icountState) = y;
				this->inputRobotState.at<float>(2,icountState) = z;
				this->inputRobotState.at<float>(3,icountState) = isTarget;
				this->inputRobotState.at<float>(4,icountState) = isObstacle;
				
				icountState++;
			}
		
		}
		
		//Using only velocities... :
		cv::vconcat( this->inputRobotState, cv::Mat::zeros( cv::Size(this->nbrObj,3), CV_32F), this->inputRobotState);
		//Using velocities and accelerations ... :
		//cv::vconcat( this->inputState, cv::Mat::zeros( cv::Size(this->nbrObj,6), CV_32F), this->inputState);
		
		if( this->pairingToDo)
		{
			cv::hconcat(this->inputState, this->inputRobotState, this->inputState);
		}
		else
		{
			this->inputRobotState.copyTo( this->inputState);
		}
		
		if(needToInit)
		{
			this->state = this->inputState;
			this->predState = this->inputState;
			this->newState = this->inputState;
		}
		
		this->pairingToDo = true;
		
	}
	
	
	void observationObs( const cv::Mat& inputMsg)
	{
		/*
		Deals with the parsing of the message in order to create the inputState with respect to obstacles seen by the sensor :
		Argument(s) :
		- inputMsg : message delivered by OPUSim_Obstacles... : 
		ARCHITECTURE OF RESULT :
		column1 : nbrRobotVisible 0
		next columns : radius theta of visible robot in target frame.
		next : the same for the previous pictures..
		*/
		
		if( inputMsg.at<float>(0,0) == 0)
		{
			return;
		}
		
		/*
		//Let us set up the elapsed time :
		if( !this->pairingToDo)
		{
			this->elapsedTime = float( clock() - this->lastClock) / CLOCKS_PER_SEC ;
			this->lastClock = clock();
		}
		*/
		
		bool needToInit = false;
		if(this->nbrObj == 0)
		{
			needToInit = true;
		}
	
		//TODO : change that line when we add the message from the obstacle sensor...
		int nbrObjNearEnough = 0;
		std::vector<bool> nearEnough(inputMsg.cols/2,false);
		//beware : the first column is not a point...
		for(int i=1;i<inputMsg.cols/2;i++)
		{
			float dist = inputMsg.at<float>(0,i);
			
			if( dist < this->tresholdFarEnough)
			{
				nearEnough[i] = true;
				nbrObjNearEnough++;
			}
		}
		
		this->nbrObj = nbrObjNearEnough;
	
		if( this->nbrObj == 0)
		{
			return ;
		}
		else
		{
			this->needToOptimize = true;
		}
		
		
		/*
		let us go on : we just have to parse the message and create the inputState :
		*/
		this->inputObsState = cv::Mat::zeros( 5, this->nbrObj, CV_32F);
		int icountState = 0;
		
		for(int i=1;i<=inputMsg.cols/2;i++)
		{
			if( nearEnough[i] )
			{
				float r = inputMsg.at<float>(0,i);
				float theta = inputMsg.at<float>(1,i);

				//frame is x<-x,y<-z,z<--y
				float x = r*cos(theta);
				float y = 0;
				float z = r*sin(theta);
	
				float isTarget = 0.0f;
				float isObstacle = 1.0f;
	
				this->inputObsState.at<float>(0,icountState) = x;
				this->inputObsState.at<float>(1,icountState) = y;
				this->inputObsState.at<float>(2,icountState) = z;
				this->inputObsState.at<float>(3,icountState) = isTarget;
				this->inputObsState.at<float>(4,icountState) = isObstacle;
				
				icountState++;
			}
		
		}
		
		//Using only velocities... :
		cv::vconcat( this->inputObsState, cv::Mat::zeros( cv::Size(this->nbrObj,3), CV_32F), this->inputObsState);
		//Using velocities and accelerations ... :
		//cv::vconcat( this->inputState, cv::Mat::zeros( cv::Size(this->nbrObj,6), CV_32F), this->inputState);
		
		if( this->pairingToDo)
		{
			cv::hconcat( this->inputState, this->inputObsState, this->inputState);			
		}
		else
		{
			this->inputObsState.copyTo(this->inputState);
		}
		
		
		if(needToInit)
		{
			this->state = this->inputState;
			this->predState = this->inputState;
			this->newState = this->inputState;
		}
		
		this->pairingToDo = true;
		
	}
	
	
	void observationDepthObs( const cv::Mat& inputMsg)
	{
		/*
		Deals with the parsing of the message in order to create the inputState with respect to obstacles seen by the Depth sensor :
		Argument(s) :
		- inputMsg : message delivered by OPUSim_RangeSensor... : 
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
		
		this->elapsedTime = float( clock() - this->lastClock) / CLOCKS_PER_SEC ;
		this->lastClock = clock();
		
		//std::cout << " FREQUENCY = " << 1.0f/this->elapsedTime << " Hz." <<  std::endl;
		
		this->predictionSimple();
		
	}
	
	void predictionSimple()
	{
		/*
		Deals with the prediction scheme that approximate the current position of the objects observed at the last step, depending on the velocities observed at the last step.
		The state of the art would use an Extended Kalman filter but let us try to make an easy Euler approximation first...
		*/
		
		//this->predState = this->state;
		// WITH VELOCITIES :
		for(int i=0;i<this->predState.cols;i++)
		{
			//float vx = this->predState.at<float>(5,i);
			//float vy = this->predState.at<float>(6,i);
			//float vz = this->predState.at<float>(7,i);
			float vx = this->state.at<float>(5,i);
			float vy = this->state.at<float>(6,i);
			float vz = this->state.at<float>(7,i);
		
			//this->predState.at<float>(0,i) += vx*this->elapsedTime;
			//this->predState.at<float>(1,i) += vy*this->elapsedTime;
			//this->predState.at<float>(2,i) += vz*this->elapsedTime;
			this->state.at<float>(0,i) += vx*this->elapsedTime;
			this->state.at<float>(1,i) += vy*this->elapsedTime;
			this->state.at<float>(2,i) += vz*this->elapsedTime;
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
		
		float deltaTheta = -currentOdometry.at<float>(1,0)*this->elapsedTime;
		float deltaR = -currentOdometry.at<float>(0,0)*this->elapsedTime;
		
		for(int i=0;i<this->predState.cols;i++)
		{
			//float x = this->predState.at<float>(0,i);
			//float y = this->predState.at<float>(2,i);
			float x = this->state.at<float>(0,i);
			float y = this->state.at<float>(2,i);
			
			float r = sqrt( x*x + y*y);
			float theta = std::atan2(y,x);
			
			r += deltaR;
			theta += deltaTheta;
			
			x = cos(theta)*r;
			y = sin(theta)*r;
			
			//this->predState.at<float>(0,i) = x;
			//this->predState.at<float>(2,i) = y;
			this->state.at<float>(0,i) = x;
			this->state.at<float>(2,i) = y;
			
		}
		
		this->predState = this->state;
		
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
		
		this->newState = this->predState;
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
			this->newState.at<float>( 5, idxOld) = ( this->newState.at<float>(0,idxOld) - this->predState.at<float>(0,idxOld) )/this->elapsedTime;
			this->newState.at<float>( 6, idxOld) = ( this->newState.at<float>(1,idxOld) - this->predState.at<float>(1,idxOld) )/this->elapsedTime;
			this->newState.at<float>( 7, idxOld) = ( this->newState.at<float>(2,idxOld) - this->predState.at<float>(2,idxOld) )/this->elapsedTime;
			*/
			this->newState.at<float>( 5, idxOld) = ( this->inputState.at<float>(0,idxInput) - this->predState.at<float>(0,idxOld) )/this->elapsedTime;
			this->newState.at<float>( 6, idxOld) = ( this->inputState.at<float>(1,idxInput) - this->predState.at<float>(1,idxOld) )/this->elapsedTime;
			this->newState.at<float>( 7, idxOld) = ( this->inputState.at<float>(2,idxInput) - this->predState.at<float>(2,idxOld) )/this->elapsedTime;
			
			//Update accelerations :
			/*
			this->newState.at<float>( 6, i) = ( this->newState.at<float>(4,i) - this->state.at<float>(4,idxOld) )/this->elapsedTime;
			this->newState.at<float>( 7, i) = ( this->newState.at<float>(5,i) - this->state.at<float>(5,idxOld) )/this->elapsedTime;
			*/
		}
		
		
		
		//let us remove the positions that are far enough of the robot :
		int nbrRemoved = 0;
		for(int i=0;i<this->newState.cols;i++)
		{
			float x = this->newState.at<float>(0,i);
			float y = this->newState.at<float>(1,i);
			float z = this->newState.at<float>(2,i);
			
			float dist = sqrt(x*x+z*z);
			
			if( dist > this->tresholdFarEnough)
			{
				nbrRemoved++;
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
		
		std::cout << " NBR OF REMOVED OBJS : " << nbrRemoved << std::endl;
		
		
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
		
		std::cout << " NBR OF UNMATCHED OBJS : " << nbrUnmatched << std::endl;
		
		//let us add those to newState :
		cv::Mat add = cv::Mat::zeros( this->inputState.rows, 1, CV_32F);
		for(int i=0;i<nbrUnmatched;i++)
		{
			extractColumn( this->inputState, unmatchedIdx[i]).copyTo(add);			
			cv::hconcat( this->newState, add, this->newState);
		}
				
		
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
		float angularTresholdSafeToGoForward = 2.0f*PI/12.0f;
		float pourcentUnsafeGoForward = 0.1f;
		
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
				//float dist = sqrt(pow( this->predState.at<float>(0,i), 2) + pow( this->predState.at<float>(2,i), 2))+1e-3f;
				float dist = sqrt(pow( this->state.at<float>(0,i), 2) + pow( this->state.at<float>(2,i), 2))+1e-3f;
				
				//std::cout << " DIST : " << dist << std::endl;
				
				if( dist < tresholdDist && dist > 0.001f)
				{
					isThereRelevantObstacles = true;
					
					//float angleObstacle = std::atan2( this->predState.at<float>(2,i), this->predState.at<float>(0,i) ) - PI/2;
					float angleObstacle = std::atan2( this->state.at<float>(2,i), this->state.at<float>(0,i) ) + PI;
					while( angleObstacle > PI)
					{
						angleObstacle -= 2*PI;
					}
					while( angleObstacle < -PI)
					{
						angleObstacle += 2*PI;
					}
					
					std::cout << " ANGLE : " << angleObstacle * 180.0f/PI << std::endl;
					
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
			if( abs(angularObstacleInduicedBias) > angularTresholdSafeToGoForward )
			{
				this->tailoredControlInput.at<float>(0,0) = desiredControlInput.at<float>(0,0)*pourcentUnsafeGoForward;//v;
			}
			else
			{
				this->tailoredControlInput.at<float>(0,0) = desiredControlInput.at<float>(0,0);//v;
			}
			this->tailoredControlInput.at<float>(1,0) = angularObstacleInduicedBias + desiredControlInput.at<float>(1,0);//omega;
			
			std::cout << " DESIRED OMEGA : " << desiredControlInput.at<float>(1,0) << std::endl;
			std::cout << " ADDITION OMEGA : " << angularObstacleInduicedBias << std::endl;
			
		}
		
		
	}
	
	
	void optimizeKuramoto( const cv::Mat& desiredControlInput)
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
			const float& gain_=1.0f;
			const float& offset = 0.2f*tresholdDist;
			// offset has to be large enough otherwise we end up with true negative avoidance...
			float R_= tresholdDist-offset;
			float a_=-1.0f;
			float kv_=-0.1f;
			float kw_=0.5f;
			float Omega_=4.0f;
			//float Omega_=0.5f;
			
			bool isThereRelevantObstacles = false;
			float tresholdAngle = 3*PI/8;
			float tresholdAngular = 3*PI/8;
			float tresholdDesiredOmegaKeepAvoiding = abs(a_*kw_*1e-1f*R_);
			obstacleAngle = PI/2;
			minObstacleDistance = tresholdDist+offset;
			
			for(int i=0;i<this->nbrObj;i++)
			{
				float dist = sqrt(pow( this->state.at<float>(0,i), 2) + pow( this->state.at<float>(2,i), 2))+1e-3f;
				
				if( dist < tresholdDist)
				{
					isThereRelevantObstacles = true;
					
					//float angleObstacle = std::atan2( this->predState.at<float>(2,i), this->predState.at<float>(0,i) ) - PI/2;
					float angleObstacle = std::atan2( this->state.at<float>(2,i), this->state.at<float>(0,i) ) + PI;
					while( angleObstacle > PI)
					{
						angleObstacle -= 2*PI;
					}
					while( angleObstacle < -PI)
					{
						angleObstacle += 2*PI;
					}
					
					if( minObstacleDistance > dist)
					{
						minObstacleDistance = dist;
						obstacleAngle = angleObstacle;
					}
					
				}
			}			
			
			if(!isThereRelevantObstacles )
			{
				//TODO
				//maybe it is a false detection...
				
				this->tailoredControlInput = desiredControlInput;
				this->kuramotoCurrentlyAvoiding = false;
				
				std::cout << " NO RELEVANT OBSTACLES TO AVOID. " << std::endl;
				
				return;
			}
			
			std::cout << " MIN DIST OBS :: ANGLE : " << obstacleAngle * 180.0f/PI << std::endl;
			std::cout << " MIN DIST OBS :: DIST : " << minObstacleDistance << std::endl;
			
			if( this->kuramotoCurrentlyAvoiding)
			{
				//let us check about the criterion :
				if( abs(obstacleAngle) > tresholdAngle )
				{
					//if we have give or take the orientation required to avoid the obstacle :
					float desiredOmega = desiredControlInput.at<float>(1,0);
					std::cout << " CRITERION :: abs desired omega x treshold : " << abs(desiredOmega) << " x " << tresholdDesiredOmegaKeepAvoiding << std::endl;
					if( abs(desiredOmega) < tresholdDesiredOmegaKeepAvoiding )
					{
						std::cout << " VALIDATED !!!!!! " << std::endl;
						//if the desired rotation is the correct one with respect to the real control law,
						//then we no longer need to avoid that obstacle :
						this->kuramotoCurrentlyAvoiding = false;
					}
					else
					{
						//else, let us go on.
						std::cout << " NOT VALIDATED !!!!!! " << std::endl;
						this->kuramotoCurrentlyAvoiding = true;
					}
				}
				else
				{
					//else, we have not got the right orientation yet, let us keep avoiding :
					this->kuramotoCurrentlyAvoiding = true;
				}
			}
						
			if( !this->kuramotoCurrentlyAvoiding )
			{
				//if we are not currently avoiding, or if we just stopped :
				// let us check about the current obstacle :
				//if( ( obstacleAngle > 0.0f && obstacleAngle > tresholdAngle && obstacleAngle < PI-tresholdAngle ) || ( obstacleAngle < 0.0f && obstacleAngle < -tresholdAngle && obstacleAngle > -PI+tresholdAngle ) )
				if( abs(obstacleAngle) > tresholdAngle )
				{
					//if the obstacle really on the side, then it is okay, we can go on with the real control law :
					this->tailoredControlInput = desiredControlInput;
					std::cout << " NO OBSTACLES TO AVOID :: obstacle on the side. " << std::endl;
					this->needToOptimize = false;
				}
				else
				{
					//else, we need to avoid it : 				
					std::cout << " OBSTACLE IS NOT ON THE SIDE, WE HAVE TO AVOID IT. ..... " << std::endl;	
					this->kuramotoCurrentlyAvoiding = true;
					this->needToOptimize = true;
				}
			}
			
			if(!this->needToOptimize)
			{
				return;
			}
			
			
			//FILTERING :
			this->kuramotoDistance = (this->kuramotoDistance*this->kuramotoTau+minObstacleDistance)/(this->kuramotoTau+1);
			minObstacleDistance = this->kuramotoDistance;
			
			//let us compute the control law with that obstacle as center :
			//depending on the side of the obstacle, we avoid differently :
			if( obstacleAngle > 0.0f)
			{
				Omega_ = -abs(Omega_);
				a_  = abs(a_);
				kv_ = abs(kv_);
				kw_ = -abs(kw_);
			}
			else
			{
				Omega_ = abs(Omega_);
				a_  = -abs(a_);
				kv_ = -abs(kv_);
				kw_ = abs(kw_);
			}
			
			float THETA = obstacleAngle;
			if( THETA < 0.0f)
			{
				THETA = -PI-THETA;
			}
			else
			{
				THETA = PI-THETA;
			}
			
			float f = a_*minObstacleDistance*(1.0f-(minObstacleDistance*minObstacleDistance)/(R_*R_));
			float g = Omega_;
  
			//----------------------------------------------------
			
			float v = gain_*kv_*(f*cos(THETA) + minObstacleDistance*g*sin(THETA));
			float omega = gain_*kw_*(minObstacleDistance*g*cos(THETA) - f*sin(THETA));
			
			//-----------------------------------------------------
			//----------------------------------------------------
			//----------------------------------------------------
			float coefficient = 0.0f*abs(minObstacleDistance-tresholdDist); 
			this->tailoredControlInput = cv::Mat::zeros( 2, 1, CV_32F);
			this->tailoredControlInput.at<float>(0,0) = v+coefficient*desiredControlInput.at<float>(0,0);
			this->tailoredControlInput.at<float>(1,0) = omega+coefficient*desiredControlInput.at<float>(1,0);;
			
		}
		
	}
	
	
	
	
	void optimizeKuramotoRadius( const cv::Mat& desiredControlInput)
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
			const float& gain_=1.0f;
			const float& offset = 0.2f*this->tresholdDist;
			// offset has to be large enough otherwise we end up with true negative avoidance...
			float R_= this->tresholdDist-offset;
			float a_=-1.0f;
			float kv_=-0.1f;
			float kw_=0.5f;
			float Omega_=4.0f;
			//float Omega_=0.5f;
			
			bool isThereRelevantObstacles = false;
			float tresholdAngle = 3*PI/8;
			float tresholdAngular = 3*PI/8;
			float tresholdDesiredOmegaKeepAvoiding = abs(a_*kw_*1e-1f*R_);
			
			obstacleAngle = PI/2;
			minObstacleDistance = tresholdDist+offset;
			
			for(int i=0;i<this->nbrObj;i++)
			{
				float dist = sqrt(pow( this->state.at<float>(0,i), 2) + pow( this->state.at<float>(2,i), 2))+1e-3f;
				
				if( dist < this->tresholdDist)
				{
					isThereRelevantObstacles = true;
					
					float angleObstacle = std::atan2( this->state.at<float>(2,i), this->state.at<float>(0,i) ) + PI;
					
					while( angleObstacle > PI)
					{
						angleObstacle -= 2*PI;
					}
					while( angleObstacle < -PI)
					{
						angleObstacle += 2*PI;
					}
					
					if( minObstacleDistance > dist)
					{
						minObstacleDistance = dist;
						obstacleAngle = angleObstacle;
					}
					
				}
			}			
			
			if(!isThereRelevantObstacles )
			{
				//TODO
				//maybe it is a false detection...
				
				this->tailoredControlInput = desiredControlInput;
				this->kuramotoCurrentlyAvoiding = false;
				
				std::cout << " NO RELEVANT OBSTACLES TO AVOID. " << std::endl;
				
				return;
			}
			
			// So far, we know of an obstacle in the vigilence area :
			// let us return the obstacle angle : 
			this->tailoredControlInput = cv::Mat::zeros( 2, 1, CV_32F);
			this->tailoredControlInput.at<float>(0,0) = minObstacleDistance;
			this->tailoredControlInput.at<float>(1,0) = obstacleAngle;
			
		}
		
	}
	
	
	void reset()
	{
		this->nbrObj = 0;
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
	
	cv::Mat run( const cv::Mat& desiredControlInput, const bool& optimize=true, const bool& kuramoto = true, const bool& kuramotoRadius = false )
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
		
		//Let us run the optimization problem :
		if( kuramoto)
		{
			this->optimizeKuramoto( desiredControlInput );
		}
		else if( kuramotoRadius)
		{
			this->optimizeKuramotoRadius( desiredControlInput );
		}
		else
		{
			this->optimizeSimple( desiredControlInput);
		}
		
		return this->tailoredControlInput;
	}
	
	void update()
	{
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
		
		//Let us pair those fellows by searching for the minimal distance until we reach the minimum between the number of newly observed objected and the number of previously observed object, as long as there are some, or as long as those are close enough :
		int minimalNbrObj = this->nbrObj;
		if( minimalNbrObj > this->predState.cols)
		{
			minimalNbrObj = this->predState.cols;
		}
		
		
		this->pairs = cv::Mat::zeros( 2, minimalNbrObj, CV_32F); 	//from predState objects to inputState objects
		int size = 0;
		
		while( size < minimalNbrObj )
		{
			double valuemin;
			double maxElimination;
			cv::Point min_loc, dummy_loc;
			cv::minMaxLoc(distM, &valuemin, &maxElimination, &min_loc, &dummy_loc);
			
			if( (float)valuemin < this->tresholdDistPair )
			{
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
			else
			{
				//the minimal distance has been reached, we no longer pair things together...
				break;
			}
		}
		
		//let us remove those extra spots :
		if(size == 0)
		{
			size = 1;
		}
			
		cv::Range r[2] = {cv::Range::all(), cv::Range(0,size)};
		this->pairs = cv::Mat( this->pairs, r);
		
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

};





class OPUSim_ControlLaw
{
	protected :
	
	std::mutex mutexRES;
	bool continuer;
	
	std::vector<cv::Mat> frames;
	std::vector<cv::Mat> obstacles;
	std::thread* t;
	
	float R;
	float desiredR;
	float Rdot;
	float thetaNearestObstacle;
	float distNearestObstacle;
	float kR;
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
	
	PIDControllerM<float> pidang;
	float Pang;
	float Iang;
	PIDControllerM<float> pidlin;
	float Plin;
	float Ilin;
	
	MetaControlLaw metacl;
	float tresholdDistAccount;
	
	
	image_transport::ImageTransport* it;
	ros::Publisher twistpub;
	ros::Publisher pathpub;
	image_transport::Subscriber img_sub;
	image_transport::Subscriber obs_sub;
	ros::Subscriber odometry_sub;
	
	int robot_number;
	geometry_msgs::Twist twistmsg;
	geometry_msgs::Pose2D goToGoalPose;
	geometry_msgs::Pose2D currentPose;
	geometry_msgs::Twist currentVel;
	nav_msgs::Path pathmsg;
	
	
	public :
	
	ros::NodeHandle nh;
	RunningStats<float>* rs;
	
	//------------------------------
	//------------------------------
	
	
	OPUSim_ControlLaw(const int& robot_number_, const bool& emergencyBreak_ = false, const bool& verbose_ = false, const float& gain_=1.0f, const float& R_=2.0f, const float& a_=1.0f, const float& epsilon_=10.0f, const float& kv_=0.1f, const float& kw_=0.2f, const float& Omega_=1.0f, const float& tresholdDistAccount_ = 0.4f, const float& tresholdDistFarEnough_ = 1.0f, const float& tresholdDistPair_ = 0.2f,  const float& Pang_=10e-1f, const float Iang_ = 0e-1f, const float& Plin_=10e-1f, const float Ilin_ = 0e-1f) : continuer(true), robot_number(robot_number_), R(R_), a(a_), epsilon(epsilon_), kv(kv_), kw(kw_), Omega(Omega_), gain(gain_), THETA(0.0f), r(0.0f), emergencyBreak(emergencyBreak_), verbose(verbose_),tau(10.0f),  Pang(Pang_), Iang(Iang_), Plin(Plin_), Ilin(Ilin_), tresholdDistAccount(tresholdDistAccount_)
	{		
	
		std::string pathvar = "OPUSim_ControlLaw_"+std::to_string(this->robot_number)+"/robot_number";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			this->nh.getParam(pathvar.c_str(),this->robot_number);
		}
		
		std::cout << "robot number : " << this->robot_number << std::endl;
		
		pathvar = "OPUSim_ControlLaw_"+std::to_string(this->robot_number)+"/emergencyBreak";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			int eb;
			this->nh.getParam(pathvar.c_str(),eb);
			this->emergencyBreak = (eb==1?true:false);
		}
		else
		{
			std::cout << pathvar << " : not found..." << std::endl;
		}
		
		std::cout << "emergency break : " << this->emergencyBreak << std::endl;
		
		pathvar = "OPUSim_ControlLaw_"+std::to_string(this->robot_number)+"/tresholdDistAccount";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			this->nh.getParam(pathvar.c_str(),this->tresholdDistAccount);
		}
		
		std::cout << "tresholdDistAccount : " << this->tresholdDistAccount << std::endl;
		
		pathvar = "OPUSim_ControlLaw_"+std::to_string(this->robot_number)+"/a";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			this->nh.getParam(pathvar.c_str(),this->a);
		}
		
		std::cout << "a : " << this->a << std::endl;
		
		pathvar = "OPUSim_ControlLaw_"+std::to_string(this->robot_number)+"/kv";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			this->nh.getParam(pathvar.c_str(),this->kv);
		}
		
		std::cout << "kv : " << this->kv << std::endl;
		
		pathvar = "OPUSim_ControlLaw_"+std::to_string(this->robot_number)+"/epsilon";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			this->nh.getParam(pathvar.c_str(),this->epsilon);
		}
		
		std::cout << "epsilon : " << this->epsilon << std::endl;
		
		
		pathvar = "OPUSim_ControlLaw_"+std::to_string(this->robot_number)+"/Omega";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			this->nh.getParam(pathvar.c_str(),this->Omega);
		}
		
		std::cout << "Omega : " << this->Omega << std::endl;
		
		pathvar = "OPUSim_ControlLaw_"+std::to_string(this->robot_number)+"/kw";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			this->nh.getParam(pathvar.c_str(),this->kw);
		}
		
		this->kw = abs(this->kw)*(this->Omega/abs(this->Omega));
		
		std::cout << "kw : " << this->kw << std::endl;
		
		pathvar = "OPUSim_ControlLaw_"+std::to_string(this->robot_number)+"/R";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			this->nh.getParam(pathvar.c_str(),this->R);
		}
		
		std::cout << "R : " << this->R << std::endl;
		this->r = 0.0f;
		this->desiredR = this->R;
		this->Rdot = 0.0f;
		this->thetaNearestObstacle = 0.0f;
		this->distNearestObstacle = 0.0f;
		
		pathvar = "OPUSim_ControlLaw_"+std::to_string(this->robot_number)+"/kR";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			this->nh.getParam(pathvar.c_str(),this->kR);
		}
		
		std::cout << "kR : " << this->kR << std::endl;
		
		
		pathvar = "OPUSim_ControlLaw_"+std::to_string(this->robot_number)+"/Plin";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			this->nh.getParam(pathvar.c_str(),this->Plin);
		}
		
		pathvar = "OPUSim_ControlLaw_"+std::to_string(this->robot_number)+"/Ilin";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			this->nh.getParam(pathvar.c_str(),this->Ilin);
		}
		
		pathvar = "OPUSim_ControlLaw_"+std::to_string(this->robot_number)+"/Pang";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			this->nh.getParam(pathvar.c_str(),this->Pang);
		}
		
		pathvar = "OPUSim_ControlLaw_"+std::to_string(this->robot_number)+"/Iang";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			this->nh.getParam(pathvar.c_str(),this->Iang);
		}
		
		it = new image_transport::ImageTransport(nh);
		
		this->metacl.setPNH( &(this->nh), this->robot_number );
		this->metacl.setTresholdDistAccount( this->tresholdDistAccount);
		this->metacl.setTresholdDistFarEnough( tresholdDistFarEnough_);
		this->metacl.setTresholdDistPair( tresholdDistPair_);
		
		std::string path( "/robot_model_teleop_"+std::to_string(this->robot_number)+"/");
		//std::string path( "/robot_model_teleop/");
		std::string pathSUB(path+"RSO");
		std::string pathSUB_OBS(path+"OBSTACLES");
		std::string pathPUB(path+"cmd_vel");
		std::string pathpathPUB(path+"PATH");
		std::string pathOdometrySUB( path+"odom_diffdrive");
		
		img_sub = it->subscribe( pathSUB.c_str(), 1, &OPUSim_ControlLaw::callback,this);
		obs_sub = it->subscribe( pathSUB_OBS.c_str(), 1, &OPUSim_ControlLaw::callbackOBS,this);
		odometry_sub = nh.subscribe( pathOdometrySUB.c_str(), 1, &OPUSim_ControlLaw::callbackOdometry, this);
		twistpub = nh.advertise<geometry_msgs::Twist>( pathPUB.c_str(), 10);
		pathpub = nh.advertise<nav_msgs::Path>( pathpathPUB.c_str(), 10);
		pathmsg.header.frame_id="map";
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		this->pidang.setConsigne(Mat<float>(0.0f,1,1));
		this->pidlin.setConsigne(Mat<float>(0.0f,1,1));
		this->pidang.setKp(this->Pang);
		this->pidang.setKi(this->Iang);
		this->pidlin.setKp(this->Plin);
		this->pidlin.setKi(this->Ilin);		
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		this->goToGoalPose.x = 0.0f;
		this->goToGoalPose.y = 0.0f;
		this->goToGoalPose.theta = 0.0f;
		
		this->currentPose = this->goToGoalPose;
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		
		std::string pathlog = "/home/kevin/rosbuild_ws/sandbox/logs/ControlLaw_"+std::to_string(this->robot_number)+"_datas";
		this->rs = new RunningStats<float>(pathlog, 100 );
		this->t = new std::thread(&OPUSim_ControlLaw::loop, this);
		
		
		ROS_INFO( std::string("OPUSim_ControlLaw::"+std::to_string(this->robot_number)+"::Initialization : OK.").c_str() );
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
		delete rs;
		
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
	
	void callbackOBS(const sensor_msgs::ImageConstPtr& original_image)
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
		cv::Mat obs;
		cv_ptr->image.copyTo( obs);
    
		//------------------------------
		//------------------------------
		
		
		mutexRES.lock();
		obstacles.insert(obstacles.begin(), obs);	
		mutexRES.unlock();
	}
	
	
	void callbackOdometry(const nav_msgs::OdometryPtr& original_msg)
	{
		
		geometry_msgs::Pose currentpose = original_msg->pose.pose;
		geometry_msgs::Pose2D currentpose2D;
		currentpose2D.x = currentpose.position.x;
		currentpose2D.y = currentpose.position.y;
		float desiredTheta = quaternion2Theta(currentpose.orientation);
		//regularization around -Pi;+Pi
		while(desiredTheta > PI)	desiredTheta -= 2*PI;
		while(desiredTheta < -PI)	desiredTheta += 2*PI;
		currentpose2D.theta = desiredTheta;
		//------------------------------
		//------------------------------
		this->rs->tadd( std::string("robotX"), currentpose2D.x );
		this->rs->tadd( std::string("robotY"), currentpose2D.y );
		this->rs->tadd( std::string("robotTHETA"), currentpose2D.theta );
		//------------------------------
		//------------------------------
		
		geometry_msgs::PoseStamped posest;
		posest.pose = currentpose;
		this->pathmsg.poses.push_back(posest);
		
		this->pathpub.publish(this->pathmsg);
		
		
		mutexRES.lock();
		this->currentPose = currentpose2D;
		this->currentVel = original_msg->twist.twist;
		mutexRES.unlock();
		
	}
	
	
	
	
	void loop()
	{
		clock_t timer = clock();
		clock_t timerdt = clock();
		int count_info = 0;
		clock_t clocktime = clock();
		float freqlogs = 1e4f;
		
		cv::Mat currentmsg;
		cv::Mat currentObsmsg;
    int nbrRobotVisible = 0;
    float v = 0.0f, metav=0.0f, outputv=0.0f;
    float omega = 0.0f, metaomega=0.0f, outputomega=0.0f;
    float mintheta;
    float g = 0.0f;
    float f = 0.0f;
    bool isDirectionSuitable = false;
    float tresholdDirectionSuitable = 3e-1f;
    float thetatarget = 0.0f;
    float dt = 0.1f;
    
		mutexRES.lock();
		while(continuer)
		{
			mutexRES.unlock();
			
			
			clock_t timerloop = clock();
			
			
			bool goOn = true;
			bool goOnObs = true;
			
			if(frames.size() >= 1 && frames[0].rows != 0 && frames[0].cols != 0)
			{
				goOn = true;
			}
			else
			{
				goOn = false;
			}
			
			if(obstacles.size() >= 1 && obstacles[0].rows != 0 && obstacles[0].cols != 0)
			{
				goOnObs = true;
			}
			else
			{
				goOnObs = false;
			}
			
			if(goOn || goOnObs)
			{
				
				mutexRES.lock();
				if(goOn)
				{
					frames[0].copyTo(currentmsg);
					frames.clear();
				}
				
				if(goOnObs)
				{
					obstacles[0].copyTo(currentObsmsg);
					obstacles.clear();
				}
				
				mutexRES.unlock();
				
				if(true)//this->verbose)
				{
					if(goOn)
					{
						std::cout << " CURRENT MSG FROM RSO : " << currentmsg << std::endl;
					}
					
					if(goOnObs)
					{
						std::cout << " CURRENT MSG FROM OBS : " << currentObsmsg << std::endl;
					}
				}
				
				/*
				if(goOn)
				{
					this->metacl.observation(currentmsg);
				}
				*/
				
				if(goOnObs)
				{
					metacl.reset();
					metacl.observationObs(currentObsmsg);
				}
								
				
				if( goOn)
				{
					//----------------------------------------------------
					//----------------------------------------------------
					//IDENTIFY COUPLING PAIR
					//----------------------------------------------------
					//----------------------------------------------------
					//-- Angular position rearrangin in [0,2*PI] :
				
					//TODO : handle the state of robots and targets after filtering :
				
					nbrRobotVisible = currentmsg.at<float>(0,0);
					
					this->parametersUpdate(nbrRobotVisible);
					
					THETA = currentmsg.at<float>(1,0);
					thetatarget = THETA;
					//it is the current THETA of the robot in the target reference-focused frame...
					while(THETA > PI)	THETA -= 2*PI;
					while(THETA < -PI)	THETA += 2*PI;
					
					//THETA *= (this->Omega/abs(this->Omega));
					//THETA = (PI-THETA);
					
					if( THETA < 0.0f)
					{
						THETA = -PI-THETA;
					}
					else
					{
						THETA = PI-THETA;
					}
					
					
					
					
					
				
					float sumphi = 0.0f;
					mintheta = 2*PI;
				
					if(nbrRobotVisible>=1)
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
				
					//float f = this->a*r*(1.0f-(r*r)/(this->R*this->R));
					f = this->a*(this->R-r);
					g = this->Omega + this->epsilon*sumphi;
					//float g = this->Omega + this->epsilon*(1.0f+Kgain)*sumphi;
				
				
		      
					//----------------------------------------------------
					//----------------------------------------------------
					//----------------------------------------------------
				
				
					//float v = Kgain*this->gain*this->kv*(f*cos(THETA) + r*g*sin(THETA));
					v = this->gain*this->kv*(f*cos(THETA) + r*g*sin(THETA));
					//float omega = Kgain*this->gain*this->kw*(r*g*cos(THETA) - f*sin(THETA));
					omega = this->gain*this->kw*(r*g*cos(THETA) - f*sin(THETA));
				
				}
				
				//----------------------------------------------------
				//COMPUTE META Control Law :
				//----------------------------------------------------
				
				cv::Mat desiredControlInput = cv::Mat::zeros( 2,1, CV_32F);
				desiredControlInput.at<float>(0,0) = v;
				desiredControlInput.at<float>(1,0) = omega;
				
				
				//filtering that prevent obstacles to become hurdles to the correct orientation of the robot...
				bool optimize = true;
				bool kuramotoUse = false;
				bool kuramotoRadiusUse = true;
				
				dt = float(clock()-timerdt)/CLOCKS_PER_SEC; //in seconds
				timerdt = clock();
			
				cv::Mat tailoredControlInput( this->metacl.run( desiredControlInput, optimize, kuramotoUse,kuramotoRadiusUse) );
				if(kuramotoUse)
				{
					metav = tailoredControlInput.at<float>(0,0);
					metaomega = tailoredControlInput.at<float>(1,0);
				}
				else if(kuramotoRadiusUse)
				{
					metav = v;
					metaomega = omega;
					
					if(tailoredControlInput.at<float>(0,0) != v)
					{
						this->thetaNearestObstacle = tailoredControlInput.at<float>(1,0);
						this->distNearestObstacle = tailoredControlInput.at<float>(0,0); 
					}
					else
					{
						this->distNearestObstacle = tailoredControlInput.at<float>(0,0);
						this->thetaNearestObstacle = PI/2.0;
					}
					
					this->updateRadius(this->thetaNearestObstacle,this->distNearestObstacle,dt);
				}
				else
				{
					metav = v;
					metaomega = omega;
				}
				//----------------------------------------------------
				//----------------------------------------------------
				
				if(this->emergencyBreak)
				{
					//v= 0.0f;
					//omega=0.0f;
					metav=0.0f;
					metaomega=0.0f;
				}	
				
			}
			
			
			/*-------------------------------------------------------*/
			/*-------------------------------------------------------*/
			//	PI(D)-Controller : 
			/*-------------------------------------------------------*/
			
			
			/*-------------------------------------------------------*/
			//observe velocity :
			float currentv = -this->currentVel.linear.x;
			float currentomega = this->currentVel.angular.z;
			float errorv = metav-currentv;
			float erroromega = metaomega-currentomega;
			
			//TODO : find how to use it ...
			/*
			if( abs(erroromega)/metaomega > tresholdDirectionSuitable)
			{
				isDirectionSuitable = false;
				std::cout << " DIRECTION NOT SUITABLE :: NO LINEAR VELOCITY !!!! " << std::endl;
			}
			else
			{
				isDirectionSuitable = true;
			}
			*/
			isDirectionSuitable = true;
			
			cv::Mat odo = cv::Mat::zeros(2,1,CV_32F);
			odo.at<float>(0,0) = currentv;
			odo.at<float>(1,0) = currentomega;
			
			this->metacl.observeOdometry( odo );
			this->metacl.update();
			
			/*-------------------------------------------------------*/
			
			if( !isDirectionSuitable)
			{
				//we do not move until the direction is the correct one.
				metav = 0.0f;
			}
			
			this->pidlin.setConsigne(Mat<float>(metav,1,1) );
			this->pidang.setConsigne(Mat<float>(metaomega,1,1) );
			
			//TODO : decide the need of PID...
			
			outputv = this->pidlin.update( Mat<float>(currentv,1,1) ).get(1,1);				
			outputomega = this->pidang.update( Mat<float>(currentomega,1,1) ).get(1,1);
			/*
			outputv = metav;
			outputomega = metaomega;
			*/
			//std::cout << " PID outputs :: VxW : " << outputv << " x " << outputomega << std::endl;
			
			//clipping :
			float treshV = 10.0f;
			float treshOmega = 10.0f;
			if( abs(outputv) > treshV)
			{
				outputv = (outputv*treshV)/abs(outputv);
			}
			
			if( abs(outputomega) > treshOmega)
			{
				outputomega = (outputomega*treshOmega)/abs(outputomega);
			}
			
			if(this->emergencyBreak)
			{
				outputv= 0.0f;
				outputomega=0.0f;
			}
			
			if(std::isnan(outputv))	outputv = 0.0f;
			if(std::isnan(outputomega))	outputomega = 0.0f;
			
			//----------------------------
			//		Publisher
			//----------------------------
			//----------------------------------------------------
			
			//----------------------------------------------------
			// MESSAGE :
			//----------------------------------------------------
			//----------------------------------------------------
			
			
			twistmsg.linear.x = -outputv;
			twistmsg.linear.y = 0.0f;
			twistmsg.linear.z = 0.0f;
			
			twistmsg.angular.x = 0.0f;
			twistmsg.angular.y = 0.0f;
			twistmsg.angular.z = -outputomega;
			
			if(this->verbose && (goOn || goOnObs) )
			{ 
				std::cout << " V x W : " << v << " x " << omega << std::endl;
			}
			
			
			twistpub.publish(twistmsg);
			
			
			
			//----------------------------------------------------
			//----------------------------------------------------
			//	LOGS :
			//----------------------------------------------------
			//----------------------------------------------------
			float elapsedTime = (clock()-clocktime)/CLOCKS_PER_SEC;
			if( true )//elapsedTime > 1.0f/freqlogs )
			{
				//LOGS :
				this->rs->tadd( std::string("v nl"), v );
				this->rs->tadd( std::string("omega nl"), omega );
				
				this->rs->tadd( std::string("v error"), errorv );
				this->rs->tadd( std::string("omega error"), erroromega );
			
				//this->rs->tadd( std::string("v consigne"), v );
				//this->rs->tadd( std::string("omega consigne"), omega );
				this->rs->tadd( std::string("v consigne"), metav );
				this->rs->tadd( std::string("omega consigne"), metaomega );
			
				this->rs->tadd( std::string("v pid"), outputv );
				this->rs->tadd( std::string("omega pid"), outputomega );
			
				this->rs->tadd( std::string("v odometry"), odo.at<float>(0,0) );
				this->rs->tadd( std::string("omega odometry"), odo.at<float>(1,0) );
			
				this->rs->tadd( std::string("nbrRobotVisible"), float(nbrRobotVisible) );
				this->rs->tadd( std::string("Omega"), float(this->Omega) );
				this->rs->tadd( std::string("kw"), float(this->kw) );
				this->rs->tadd( std::string("kv"), float(this->kv) );
				this->rs->tadd( std::string("kR"), float(this->kR) );
				this->rs->tadd( std::string("epsilon"), float(this->epsilon) );
				this->rs->tadd( std::string("a"), float(this->a) );
				this->rs->tadd( std::string("THETA"), float(THETA) );
				this->rs->tadd( std::string("THETAtarget"), float(thetatarget) );
				this->rs->tadd( std::string("g"), float(g) );
				this->rs->tadd( std::string("f"), float(f) );
				this->rs->tadd( std::string("PSI"), float(mintheta) );
				this->rs->tadd( std::string("r"), float(this->r) );
				this->rs->tadd( std::string("R"), float(this->R) );
				this->rs->tadd( std::string("Rdot"), float(this->Rdot) );
				this->rs->tadd( std::string("thetaNearestObstacle"), float(this->thetaNearestObstacle) );
				this->rs->tadd( std::string("distNearestObstacle"), float(this->distNearestObstacle) );
				this->rs->tadd( std::string("dt"), float(dt) );
				
				clocktime = clock();
			}
			
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
	
	void updateRadius(float thetaObs, float distObs, float dt)
	{
		//compute the velocity :
		if( abs(thetaObs) > PI/2.0 )	thetaObs = PI/2.0f;
		
		float lambda = cos(thetaObs);
		this->Rdot = (this->desiredR-this->R)*(1.0f/this->desiredR)*(1.0-lambda) + (-this->kR/(abs(distObs-0.7*this->tresholdDistAccount)+1e-4f))*(thetaObs/(abs(thetaObs)+1e-4f))*lambda; 
		
		//update :
		this->R += dt*this->Rdot;
		
		if(this->R < this->desiredR/4.0)	this->R = this->desiredR/4.0;
		if(this->R > this->desiredR*4.0) this->R = this->desiredR*4.0;
		
	}
	
	void parametersUpdate(const int& nbrRobotVisible)
	{
		//nbr robot visible without counting itself..., nor the target.
		// N = nbrRobotVisible+1
		switch(nbrRobotVisible)
		{
			case 0 :
			{
				this->kw =  abs(this->kw) * ( this->Omega/abs(this->Omega) );
				/* :
				this->epsilon = - abs(this->epsilon) * ( this->Omega/abs(this->Omega) );
				this->kv = - abs(this->kv) * ( this->Omega/abs(this->Omega) );
				this->a = - abs(this->a) * ( this->Omega/abs(this->Omega) );
				*/
				
				/*
				//counter-clockwise iff Omega > 0 :
				this->epsilon = abs(this->epsilon) * ( this->Omega/abs(this->Omega) );
				this->kv = abs(this->kv) * ( this->Omega/abs(this->Omega) );
				this->a = abs(this->a) * ( this->Omega/abs(this->Omega) );
				*/
				
				//clockwise iff Omega > 0 :
				this->epsilon = - abs(this->epsilon) * ( this->Omega/abs(this->Omega) );
				this->kv = - abs(this->kv) * ( this->Omega/abs(this->Omega) );
				this->a = - abs(this->a) * ( this->Omega/abs(this->Omega) );
			}
			break;
			
			case 1:
			{
				//this->Omega = abs(this->Omega);
				this->kw = abs(this->kw) * ( this->Omega/abs(this->Omega) );//0.2f;
				/*
				// DEFAULT :
				this->epsilon = - abs(this->epsilon) * ( this->Omega/abs(this->Omega) );//-1.0f;
				this->kv = abs(this->kv) * ( this->Omega/abs(this->Omega) );//0.1f;
				this->a = abs(this->a) * ( this->Omega/abs(this->Omega) );//1.0f;
				*/
				
				// clockwise iff Omega > 0:
				this->epsilon = abs(this->epsilon) * ( this->Omega/abs(this->Omega) );//-1.0f;
				this->kv = - abs(this->kv) * ( this->Omega/abs(this->Omega) );//0.1f;
				this->a = - abs(this->a) * ( this->Omega/abs(this->Omega) );//1.0f;
			}
			break;
			
			case 2:
			{
				this->kw = abs(this->kw) * ( this->Omega/abs(this->Omega) );//0.2f;
				/*
				//DEFAULT
				this->epsilon = abs(this->epsilon) * ( this->Omega/abs(this->Omega) );//0.5f;
				this->kv = - abs(this->kv) * ( this->Omega/abs(this->Omega) );//-0.1f;
				this->a = - abs(this->a) * ( this->Omega/abs(this->Omega) );//-1.0f;
				*/
				
				// clockwise iff Omega > 0 :
				this->epsilon = abs(this->epsilon) * ( this->Omega/abs(this->Omega) );//0.5f;
				this->kv = - abs(this->kv) * ( this->Omega/abs(this->Omega) );//-0.1f;
				this->a = - abs(this->a) * ( this->Omega/abs(this->Omega) );//-1.0f;
			}
			break;
			
			case 3:
			{
				this->kw = abs(this->kw) * ( this->Omega/abs(this->Omega) );
				
				// clockwise iff Omega > 0 :
				this->epsilon = - abs(this->epsilon) * ( this->Omega/abs(this->Omega) );
				this->kv = - abs(this->kv) * ( this->Omega/abs(this->Omega) );
				this->a = - abs(this->a) * ( this->Omega/abs(this->Omega) );
			}
			break;
			
			case 4:
			{
				this->kw = abs(this->kw) * ( this->Omega/abs(this->Omega) );
				
				// clockwise iff Omega > 0 :
				this->epsilon = - abs(this->epsilon) * ( this->Omega/abs(this->Omega) );
				this->kv = - abs(this->kv) * ( this->Omega/abs(this->Omega) );
				this->a = - abs(this->a) * ( this->Omega/abs(this->Omega) );
			}
			break;
			
			default :
			{
				this->kw = abs(this->kw) * ( this->Omega/abs(this->Omega) );
				
				// clockwise iff Omega > 0 :
				this->epsilon = - abs(this->epsilon) * ( this->Omega/abs(this->Omega) );
				this->kv = - abs(this->kv) * ( this->Omega/abs(this->Omega) );
				this->a = - abs(this->a) * ( this->Omega/abs(this->Omega) );
			}
			break;
		}
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
