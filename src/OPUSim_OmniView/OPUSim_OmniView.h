#ifndef OPUSIM_OMNIVIEW_H
#define OPUSIM_OMNIVIEW_H

#include <iostream>
#include <string>
#include <thread>
#include <mutex>

using namespace std;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"

namespace enc = sensor_msgs::image_encodings;




class OPUSim_OmniView
{
	protected :
	
	std::mutex mutexRES;
	bool continuer;
	
	//std::vector<cv::Mat> frameProcessed;
	std::vector<std::vector<cv::Mat> > frames;
	
	std::thread* t;
	
	public :
	
	ros::NodeHandle nh;
	
	
	image_transport::ImageTransport* it;
	image_transport::Publisher img_pub;
	
	
	image_transport::Subscriber img_sub1;
	image_transport::Subscriber img_sub2;
	image_transport::Subscriber img_sub3;
	image_transport::Subscriber img_sub4;
	
	int robot_number;
	int method;
	int scaler;
	
	
	OPUSim_OmniView(const int& robot_number_, const int& method_ = 0 ) : continuer(true), robot_number(robot_number_), method(method_), scaler(1)
	{			
		if( this->nh.hasParam("OPUSim_OmniView/robot_number") )
		{
			this->nh.getParam("OPUSim_OmniView/robot_number",this->robot_number);
		}
				
		for(int i=4;i--;)	
		{
			frames.push_back( std::vector<cv::Mat>() );
			//frameProcessed.push_back( cv::Mat() );
		}
		
		
		it = new image_transport::ImageTransport(nh);
		
		std::string path( "/robot_model_teleop_"+std::to_string(robot_number)+"/");
		//std::string path( "/robot_model_teleop/");
		std::string path1( path+"camera1/image_raw");
		std::string path2( path+"camera2/image_raw");
		std::string path3( path+"camera3/image_raw");
		std::string path4( path+"camera4/image_raw");
		
		
		img_sub1 = it->subscribe( path1.c_str(), 1, &OPUSim_OmniView::callback1,this);
		img_sub2 = it->subscribe( path2.c_str(), 1, &OPUSim_OmniView::callback2,this);
		img_sub3 = it->subscribe( path3.c_str(), 1, &OPUSim_OmniView::callback3,this);
		img_sub4 = it->subscribe( path4.c_str(), 1, &OPUSim_OmniView::callback4,this);
		
		img_pub = it->advertise( std::string(path+"/OMNIVIEW").c_str(), 1);
		
		
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		
		
		t = new std::thread(&OPUSim_OmniView::loop, this);
		
		ROS_INFO( std::string("OPUSim_OmniView::"+std::to_string(robot_number)+"::Initialization : OK.").c_str() );
	}
	
	~OPUSim_OmniView()
	{
		
		this->setContinuer(false);
		
		if(t->joinable())
		{
			t->join();
		}
		
		delete t;
		delete it;
		
		ROS_INFO("OPUSim_OmniView::Exiting.");
	}
	
	void callback1(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
	
		try
		{
			cv_ptr = cv_bridge::toCvCopy(original_image,enc::BGR8);
		}
		catch( cv_bridge::Exception e)
		{
			ROS_ERROR("OPUSim_OmniView::::cv_bridge exception : %s", e.what());
			return ;
		}
		//------------------------------------------------
		//		Stack of Frames
		//-----------------------------------------------
		cv::Mat frameDownSample;
		int h = cv_ptr->image.rows/scaler;
		//h*=2;
		//int h = 480;
		int w = cv_ptr->image.cols/scaler;
		//w*=2;
		//int w = 640;
		cv::resize(cv_ptr->image,frameDownSample,cv::Size(w,h));
		
		//------------------------------
		//------------------------------
		//	enhancement :
		//------------------------------
		//------------------------------
		//float sigma_s = 10.àf;
		//float sigma_r = 0.15f;
		//cv::detailEnhance(frameDownSample, frameDownSample, sigma_s, sigma_r);
		
		/// Do the operation new_image(i,j) = alpha*image(i,j) + beta
		float alpha = 1.0f;
		float beta = 0.0f;
		frameDownSample.convertTo( frameDownSample, -1, alpha, beta);
    
		//------------------------------
		//------------------------------
		
		
		mutexRES.lock();
		frames[0].insert(frames[0].begin(), frameDownSample);	
		mutexRES.unlock();
		
	}
	
	void callback2(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
	
		try
		{
			cv_ptr = cv_bridge::toCvCopy(original_image,enc::BGR8);
		}
		catch( cv_bridge::Exception e)
		{
			ROS_ERROR("OPUSim_OmniView::::cv_bridge exception : %s", e.what());
			return ;
		}
		//------------------------------------------------
		//		Stack of Frames
		//-----------------------------------------------
		cv::Mat frameDownSample;
		int h = cv_ptr->image.rows/scaler;
		//h*=2;
		//int h = 480;
		int w = cv_ptr->image.cols/scaler;
		//w*=2;
		//int w = 640;
		cv::resize(cv_ptr->image,frameDownSample,cv::Size(w,h));
		
		//------------------------------
		//------------------------------
		//	enhancement :
		//------------------------------
		//------------------------------
		//float sigma_s = 10.àf;
		//float sigma_r = 0.15f;
		//cv::detailEnhance(frameDownSample, frameDownSample, sigma_s, sigma_r);
		
		/// Do the operation new_image(i,j) = alpha*image(i,j) + beta
		float alpha = 1.0f;
		float beta = 0.0f;
		frameDownSample.convertTo( frameDownSample, -1, alpha, beta);
    
		//------------------------------
		//------------------------------
		
		
		mutexRES.lock();
		frames[1].insert(frames[1].begin(), frameDownSample);	
		mutexRES.unlock();
		
	}
	
	void callback3(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
	
		try
		{
			cv_ptr = cv_bridge::toCvCopy(original_image,enc::BGR8);
		}
		catch( cv_bridge::Exception e)
		{
			ROS_ERROR("OPUSim_OmniView::::cv_bridge exception : %s", e.what());
			return ;
		}
		//------------------------------------------------
		//		Stack of Frames
		//-----------------------------------------------
		cv::Mat frameDownSample;
		int h = cv_ptr->image.rows/scaler;
		//h*=2;
		//int h = 480;
		int w = cv_ptr->image.cols/scaler;
		//w*=2;
		//int w = 640;
		cv::resize(cv_ptr->image,frameDownSample,cv::Size(w,h));
		
		//------------------------------
		//------------------------------
		//	enhancement :
		//------------------------------
		//------------------------------
		//float sigma_s = 10.àf;
		//float sigma_r = 0.15f;
		//cv::detailEnhance(frameDownSample, frameDownSample, sigma_s, sigma_r);
		
		/// Do the operation new_image(i,j) = alpha*image(i,j) + beta
		float alpha = 1.0f;
		float beta = 0.0f;
		frameDownSample.convertTo( frameDownSample, -1, alpha, beta);
    
		//------------------------------
		//------------------------------
		
		
		mutexRES.lock();
		frames[2].insert(frames[2].begin(), frameDownSample);	
		mutexRES.unlock();
		
	}
		
		
		
	void callback4(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
	
		try
		{
			cv_ptr = cv_bridge::toCvCopy(original_image,enc::BGR8);
		}
		catch( cv_bridge::Exception e)
		{
			ROS_ERROR("OPUSim_OmniView::::cv_bridge exception : %s", e.what());
			return ;
		}
		//------------------------------------------------
		//		Stack of Frames
		//-----------------------------------------------
		cv::Mat frameDownSample;
		int h = cv_ptr->image.rows/scaler;
		//h*=2;
		//int h = 480;
		int w = cv_ptr->image.cols/scaler;
		//w*=2;
		//int w = 640;
		cv::resize(cv_ptr->image,frameDownSample,cv::Size(w,h));
		
		//------------------------------
		//------------------------------
		//	enhancement :
		//------------------------------
		//------------------------------
		//float sigma_s = 10.àf;
		//float sigma_r = 0.15f;
		//cv::detailEnhance(frameDownSample, frameDownSample, sigma_s, sigma_r);
		
		/// Do the operation new_image(i,j) = alpha*image(i,j) + beta
		float alpha = 1.0f;
		float beta = 0.0f;
		frameDownSample.convertTo( frameDownSample, -1, alpha, beta);
    
		//------------------------------
		//------------------------------
		
		
		mutexRES.lock();
		frames[3].insert(frames[3].begin(), frameDownSample);	
		mutexRES.unlock();
		
	}
	
	
	
	
	
	void loop()
	{
		clock_t timer = clock();
		int count_info = 0;
		
		cv::namedWindow("OMNIVIEW",CV_WINDOW_AUTOSIZE);
		cv::Mat frameProcessed;
		
		
		mutexRES.lock();
		while(continuer)
		{
			mutexRES.unlock();
			
			cv::Mat frameToProcess[4];
			bool goOn = true;
			
			for(int i=4;i--;)
			{
				if(frames[i].size())
				{
					goOn = true;
				}
				else
				{
					goOn = false;
					break;
				}
			}
			
			if(goOn)
			{
				
				mutexRES.lock();
				for(int i=4;i--;)
				{
					frames[i][frames[i].size()-1].copyTo(frameToProcess[i]);
				}
				mutexRES.unlock();
				
				//----------------------------------------------------
				//----------------------------------------------------
				//CLIPPING :
				//----------------------------------------------------
				//----------------------------------------------------
				
				cv::hconcat(frameToProcess[3],frameToProcess[2],frameProcessed);
				cv::hconcat(frameProcessed,frameToProcess[1],frameProcessed);
				cv::hconcat(frameProcessed,frameToProcess[0],frameProcessed);
				
				//----------------------------------------------------
				//----------------------------------------------------
				//----------------------------------------------------
				
				mutexRES.lock();
				for(int i=4;i--;)	frames[i].clear();
				mutexRES.unlock();
				
#ifdef debug_v0		
				count_info++;
		
				if(count_info>10)
				{
					ROS_INFO("OPUSim_OmniView::FPS : %f.", CLOCKS_PER_SEC/((float)(clock()-timer)) );
					count_info = 0;
				}
				timer = clock();
#endif 

				//----------------------------
				//		Publisher
				//----------------------------
		
				//--------------------------------------------
				//
				//	Convert CvImage to ROS image message and publish it to camera/image_processed topic
				//
				//--------------------------------------------
					
				sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameProcessed).toImageMsg();
				img_pub.publish(msg);
				cv::imshow( "OMNIVIEW", frameProcessed);
			
	
			}
			
			if(char(cv::waitKey(1))=='q')			
			{
				this->setContinuer(false);
			}
			
			
			
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

#endif
