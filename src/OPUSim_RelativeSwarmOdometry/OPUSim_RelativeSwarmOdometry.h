#ifndef OPUSIM_RELATIVESWARMODOMETRY_H
#define OPUSIM_RELATIVESWARMODOMETRY_H

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

#define debug_v0		
//#define debug_v1
#define PI 3.1415926f

int low_r=0, low_g=0, low_b=30;
int high_r=50, high_g=50, high_b=255;

int tlow_r=30, tlow_g=0, tlow_b=0;
int thigh_r=255, thigh_g=50, thigh_b=50;

std::string objdetectstr;
std::string trackstr;
cv::RNG rng(12345);

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
	
	//std::cout << eucl << std::endl;
	
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

class OPUSim_RelativeSwarmOdometry
{
	protected :
	
	std::mutex mutexRES;
	bool continuer;
	
	std::vector<cv::Mat> frames;
	
	std::thread* t;
	
	public :
	
	ros::NodeHandle nh;
	
	
	image_transport::ImageTransport* it;
	image_transport::Publisher img_pub;
	// it is not a really image, it is a matrix of data.
	/*
	DATA Representation :
	For each column, we have the information of a robot :
	[ isTarget? radius theta rdot thetadot ]^T
	*/
	
	
	image_transport::Subscriber img_sub;
	
	int robot_number;
	int method;
	int scaler;
	int h;
	int w;
	
	bool notarget;
	bool noneighbours;
	bool pushing;
	
	bool verbose;
	
	
	OPUSim_RelativeSwarmOdometry(const int& robot_number_, const int& method_ = 0, const bool& verbose_ = false ) : continuer(true), robot_number(robot_number_), method(method_),scaler(1),notarget(true),noneighbours(true), verbose(verbose_), pushing(false)
	{			
		std::string pathvar = "OPUSim_RelativeSwarmOdometry_"+std::to_string(this->robot_number)+"/robot_number";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			this->nh.getParam(pathvar.c_str(),this->robot_number);
		}
		
		pathvar = "OPUSim_RelativeSwarmOdometry_"+std::to_string(this->robot_number)+"/debug";
		if( this->nh.hasParam(pathvar.c_str()) )
		{
			int verbose;
			this->nh.getParam(pathvar.c_str(),verbose);
			this->verbose = (verbose==1?true:false);
		}
		
		objdetectstr = std::string("Object Detection "+std::to_string(robot_number));
		trackstr = std::string("TRACKING "+std::to_string(robot_number));
		
		it = new image_transport::ImageTransport(nh);
		
		std::string path( "/robot_model_teleop_"+std::to_string(robot_number)+"/");
		//std::string path( "/robot_model_teleop/");
		std::string pathSUB(path+"OMNIVIEW");
		std::string pathPUB(path+"RSO");
		
		img_sub = it->subscribe( pathSUB.c_str(), 1, &OPUSim_RelativeSwarmOdometry::callback,this);
		img_pub = it->advertise( pathPUB.c_str(), 1);
		
		
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		
		
		t = new std::thread(&OPUSim_RelativeSwarmOdometry::loop, this);
		
		ROS_INFO( std::string("OPUSim_RelativeSwarmOdometry::"+std::to_string(robot_number)+"::Initialization : OK.").c_str() );
	}
	
	~OPUSim_RelativeSwarmOdometry()
	{
		
		this->setContinuer(false);
		
		if(t->joinable())
		{
			t->join();
		}
		
		delete t;
		delete it;
		
		ROS_INFO("OPUSim_RelativeSwarmOdometry::Exiting.");
	}
	
	void callback(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
	
		try
		{
			cv_ptr = cv_bridge::toCvCopy(original_image,enc::BGR8);
		}
		catch( cv_bridge::Exception e)
		{
			ROS_ERROR("OPUSim_RelativeSwarmOdometry::::cv_bridge exception : %s", e.what());
			return ;
		}
		//------------------------------------------------
		//		Stack of Frames
		//-----------------------------------------------
		cv::Mat frameDownSample;
		h = cv_ptr->image.rows/scaler;
		//h*=2;
		//int h = 480;
		w = cv_ptr->image.cols/scaler;
		//w*=2;
		//int w = 640;
		cv::resize(cv_ptr->image,frameDownSample,cv::Size(w,h));
		
		//------------------------------
		//------------------------------
		//	enhancement :
		//------------------------------
		//------------------------------
		//float sigma_s = 10.0f;
		//float sigma_r = 0.15f;
		//cv::detailEnhance(frameDownSample, frameDownSample, sigma_s, sigma_r);
		
		/// Do the operation new_image(i,j) = alpha*image(i,j) + beta
		float alpha = 1.0f;
		float beta = 0.0f;
		frameDownSample.convertTo( frameDownSample, -1, alpha, beta);
    
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
		
		cv::Mat result;
		float thetatargetrad = 0.0f;
		
		if(this->verbose)
		{
			cv::namedWindow(trackstr.c_str(), CV_WINDOW_AUTOSIZE);
    	cv::namedWindow(objdetectstr.c_str(), CV_WINDOW_AUTOSIZE);
    
		  //-- Trackbars to set thresholds for RGB values
		  cv::createTrackbar("Low R",objdetectstr.c_str(), &low_r, 255, (cv::TrackbarCallback) &OPUSim_RelativeSwarmOdometry::on_low_r_thresh_trackbar,this);
		  cv::createTrackbar("High R",objdetectstr.c_str(), &high_r, 255, (cv::TrackbarCallback) &OPUSim_RelativeSwarmOdometry::on_high_r_thresh_trackbar,this);
		  cv::createTrackbar("Low G",objdetectstr.c_str(), &low_g, 255, (cv::TrackbarCallback) &OPUSim_RelativeSwarmOdometry::on_low_g_thresh_trackbar,this);
		  cv::createTrackbar("High G",objdetectstr.c_str(), &high_g, 255, (cv::TrackbarCallback) &OPUSim_RelativeSwarmOdometry::on_high_g_thresh_trackbar,this);
		  cv::createTrackbar("Low B",objdetectstr.c_str(), &low_b, 255, (cv::TrackbarCallback) &OPUSim_RelativeSwarmOdometry::on_low_b_thresh_trackbar,this);
			cv::createTrackbar("High B",objdetectstr.c_str(), &high_b, 255, (cv::TrackbarCallback) &OPUSim_RelativeSwarmOdometry::on_high_b_thresh_trackbar,this);
		}
		
		std::vector<cv::Mat> frameTresholded(2);
		std::vector<std::vector<cv::Point> > robots(2);
		std::vector<std::vector<std::vector<cv::Point> > > contours(2);
		std::vector<std::vector<std::vector<cv::Point> > > tcontour(2);
		std::vector<std::vector<cv::Vec4i> > thierarchy(2);
		std::vector<std::vector<cv::Vec4i> >hierarchy(2);
		std::vector<int> targetIDX(2);
		
		std::vector<std::vector<float> > thetas(2);
		std::vector<std::vector<float> > radius(2);
		std::vector<cv::Mat> polarsInT(2);
		polarsInT[1] = cv::Mat::zeros(2,1,CV_32F);
		
		mutexRES.lock();
		while(continuer)
		{
			mutexRES.unlock();
			
			//clock_t timerloop = clock();
			
			
			cv::Mat frameToProcess[1];
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
				for(int i=1;i--;)
				{
					frames[i].copyTo(frameToProcess[i]);
					
				}
				
				frames.clear();
				mutexRES.unlock();
				
				//----------------------------------------------------
				//----------------------------------------------------
				//EXTRACT ROBOTS AND TARGET:
				//----------------------------------------------------
				//----------------------------------------------------
				//-- Detect the object based on RGB Range Values
				for(int i=1;i--;)
				{
					cv::Mat blurred;
					//cv::GaussianBlur(frameToProcess[i], blurred, cv::Size(11, 11));
					frameToProcess[i].copyTo(blurred);
					
					clock_t timerinrange = clock();
					
					//cv::inRange(frameToProcess[i],cv::Scalar(low_b,low_g,low_r), cv::Scalar(high_b,high_g,high_r), frameTresholded[i]);
        	cv::Mat targetimg;
        	cv::inRange(blurred,cv::Scalar(low_b,low_g,low_r), cv::Scalar(high_b,high_g,high_r), frameTresholded[i]);
        	cv::inRange(blurred,cv::Scalar(tlow_b,tlow_g,tlow_r), cv::Scalar(thigh_b,thigh_g,thigh_r), targetimg);
        	
        	#ifdef debug_v1
        	ROS_INFO("OPUSim_RelativeSwarmOdometry::FPS :: inrange : %f.", CLOCKS_PER_SEC/((float)(clock()-timerinrange)) );
        	timerinrange=clock();
        	#endif
        	
					//cv::Mat hsv;
					//cv::cvtColor(blurred,hsv, cv::COLOR_BGR2HSV);
					
					cv::Mat erode;
					int morph_type = cv::MORPH_ELLIPSE;
					int morph_size = 1;
					cv::Mat element = cv::getStructuringElement( morph_type,
										                           cv::Size( 2*morph_size + 1, 2*morph_size+1 ),
										                           cv::Point( morph_size, morph_size ) );
					/// Apply the erosion operation
					//cv::erode( frameTresholded[i], erode, element );
					frameTresholded[i].copyTo(erode);
					cv::erode( targetimg, targetimg, element );
					
					morph_size = 4;
					element = cv::getStructuringElement( morph_type,
										                           cv::Size( 2*morph_size + 1, 2*morph_size+1 ),
										                           cv::Point( morph_size, morph_size ) );
					/// Apply the dilatation operation :
					cv::dilate( erode,frameTresholded[i], element );
					cv::dilate( targetimg,targetimg, element );
					
					#ifdef debug_v1
					ROS_INFO("OPUSim_RelativeSwarmOdometry::FPS :: erode dilate : %f.", CLOCKS_PER_SEC/((float)(clock()-timerinrange)) );
					timerinrange=clock();
					#endif
					
					/// Detect edges using canny
					cv::Mat canny_output;
					int tresh = 100;
					Canny( frameTresholded[i], canny_output, tresh, tresh*2, 3 );
					Canny( targetimg, targetimg, tresh, tresh*2, 3 );
					/// Find contours
					findContours( targetimg, tcontour[i], thierarchy[i], CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
					findContours( canny_output, contours[i], hierarchy[i], CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
					
					#ifdef debug_v1
					ROS_INFO("OPUSim_RelativeSwarmOdometry::FPS :: findcontour : %f.", CLOCKS_PER_SEC/((float)(clock()-timerinrange)) );
					timerinrange=clock();
					#endif
					
					/// Draw contours
					cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
						
					if( contours[i].size() > 0)
					{
						noneighbours = false;
						
						for( int j = 0; j< contours[i].size(); j++ )
						{
							cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
							cv::drawContours( drawing, contours[i], j, color, 2, 8, hierarchy[i], 0, cv::Point() );
						
							cv::Scalar meancenter( mean(contours[i][j] ) );

							/*
							float minycont = 0.0f;
							for(int k=0;k<=contours[i][j].size();k++)
							{
								if(minycont < contours[i][j][k].y)
								{
									minycont = contours[i][j][k].y;
								}
							}

							cv::Point temp(meancenter[0], minycont);
							*/
							
							cv::Point temp(meancenter[0], meancenter[1]);
							
							float tresholdDistance = 10.0f;
							bool duplicate = alreadyExists( temp, robots[i], tresholdDistance);
						
							if( !duplicate )
							{
								robots[i].push_back( temp  );
							
								if(i==0)
								{
									cv::circle( drawing, robots[i][robots[i].size()-1],5,cv::Scalar(200,0,50),5); 
								}
							}
						
						
						}
					}
					else
					{
						//TODO : there is no other robot in the swarm...
						noneighbours = true;
					}
					
					if( tcontour[i].size() > 0)
					{
						notarget = false;
						
						for( int j = 0; j< tcontour[i].size(); j++ )
						{
							cv::Scalar color = cv::Scalar( 255, 255, 255 );
							cv::drawContours( drawing, tcontour[i], j, color, 2, 8, thierarchy[i], 0, cv::Point() );
						
							cv::Scalar meancenter( mean(tcontour[i][j] ) );
							
							/*
							float minycont = 0.0f;
							for(int k=0;k<=tcontour[i][j].size();k++)
							{
								if(minycont < tcontour[i][j][k].y)
								{
									minycont = tcontour[i][j][k].y;
								}
							}
							
							cv::Point temp(meancenter[0], minycont);
							*/

							cv::Point temp(meancenter[0], meancenter[1]);
						
							float tresholdDistance = 2.0f;
							bool duplicate = alreadyExists( temp, robots[i], tresholdDistance);
						
							if( !duplicate )
							{
								robots[i].push_back( temp  );
							
								if(i==0)
								{
									cv::circle( drawing, robots[i][robots[i].size()-1],5,cv::Scalar(200,0,50),5); 
								}
							
								targetIDX[i] = robots[i].size()-1; 
							}
						}					
						
					}
					else
					{
						//TODO : there is no target visible...
						notarget = true;
					}
					
					#ifdef debug_v1
					ROS_INFO("OPUSim_RelativeSwarmOdometry::FPS :: contours drawing : %f.", CLOCKS_PER_SEC/((float)(clock()-timerinrange)) );
					#endif
					
					drawing.copyTo(frameTresholded[i]);
					

        }
        
        
        
        
        //-- Show the most recent frame
        if(this->verbose)
        {
        	std::cout << " FOUND : " << robots[0].size() << std::endl;
        	imshow(trackstr.c_str(),frameTresholded[0]);
        }
				//----------------------------------------------------
				//----------------------------------------------------
				//----------------------------------------------------
				
				//----------------------------------------------------
				//----------------------------------------------------
				//COMPUTE QUANTITIES :
				//----------------------------------------------------
				//----------------------------------------------------
				float cameraoffsetangle = 45.0f;
				float wTothetas = 360.0f/float(w);
				float offseth = float(h)/2;
				float offsetcamerah = 0.01f;
				float hToradius = float(h-offseth);
			
				for(int i=1;i--;)
				{
					for(int j=0;j<robots[i].size();j++)
					{
						// COMPUTATION OF THETAS :
						float thetaval =  180.0f - wTothetas*robots[i][j].x - cameraoffsetangle;
						
						if(thetaval < -180.0f)
						{
							thetaval += 360.0f;
						}
						if(thetaval > 180.0f)
						{
							thetaval -= 360.0f;
						}
						
						// COMPUTATION OF RADIUS :
						//float radiusval = 1.2f*50.0f/(robots[i][j].y-offseth) + offsetcamerah;
						float radiusval = 20.0f/(robots[i][j].y-offseth) + offsetcamerah;
						
						if( robots[i][j].y > offseth && radiusval > offsetcamerah)
						{
							radius[i].push_back( radiusval);
							thetas[i].push_back( thetaval);
						}
						else
						{
							//let us account for the target :
							if(j==robots[i].size()-1)
							{
								notarget = true;
							}
						}
					}
				}
				
				if( !noneighbours )
				{
				
					cv::Mat cvthetatarget = cv::Mat::zeros( cv::Size(1,1), CV_32F);
					cv::Mat cvradiustarget = cv::Mat::zeros( cv::Size(1,1), CV_32F);
					cv::Mat targetpolar;
					
					if( !notarget )
					{
						this->pushing = true;
						thetatargetrad = thetas[0][targetIDX[0]]*PI/180.0f;
						cvthetatarget.at<float>(0,0) = thetas[0][targetIDX[0]];
						cvradiustarget.at<float>(0,0) = radius[0][targetIDX[0]];
						
						//thetas[0].erase(thetas[0].begin()+targetIDX[0]);
						//radius[0].erase(radius[0].begin()+targetIDX[0]);
						// if there is a target, then, however the formulation, it is the last one :
						thetas[0].erase(thetas[0].begin()+thetas[0].size()-1);
						radius[0].erase(radius[0].begin()+radius[0].size()-1);
						
						
						cv::vconcat( cvradiustarget, cvthetatarget, targetpolar);
				
						//std::cout << " target polar : " << targetpolar << std::endl;
					}
					else
					{
						std::cout << " NO TARGET..." << std::endl;
						this->pushing = true;
						
						//THERE IS NO TARGET VISIBLE : Let the robot assume it is at the correct distance :
						targetpolar = cv::Mat::zeros( 2, 1, CV_32F);
						float desiredradius = 3.0f;
						targetpolar.at<float>(0,0) = desiredradius;
						targetpolar.at<float>(1,0) = 0.0f;
						
						//std::cout << " target polar : " << targetpolar << std::endl;
					}
				
					cv::Mat cvthetas = cv::Mat(thetas[0]);
					//std::cout << " angles in robot frame : " << cvthetas << std::endl;
					for(int i=cvthetas.rows;i--;)	cvthetas.at<float>(i,0) = cvthetas.at<float>(i,0) * PI/180.0f;			
					cv::Mat cvradius = cv::Mat(radius[0]);
					
					cv::Mat polars;
					cv::vconcat( cvradius.t(), cvthetas.t(), polars);
					cv::hconcat( polars, cv::Mat::zeros(2,1,CV_32F),polars);
					//let us add the position of this robot in order to send its position in the target frame :
					
					cv::vconcat( polars, cv::Mat::ones(1,polars.cols,CV_32F), polars);
					// homogeneous transformation..
					
					//std::cout << " polar pos in robot frame : " << polars << std::endl;
					
					// let us transform the reference frame en the coordinate system :
					cv::Mat euclInT = polar2euclidianTarget( polars, targetpolar);
				
					//std::cout << " eucl pos in target frame : " << euclInT << std::endl;
					
					//let us come back to the polar coordinate system and extract the non-homogeneous form :
					//std::cout << "euclinT : " << euclInT << std::endl;
					//std::cout << "eucl2polar : " << euclidian2polar( euclInT ) << std::endl;
					
					polarsInT[0] = (euclidian2polar( euclInT ))(cv::Range(0,2), cv::Range(0,euclInT.cols) );

					// REGULARIZATION WITH REGARDS TO THE NUMBER OF NEIGHBOURS :
					cv::Mat regularizeMessageNbrRobot = cv::Mat::zeros( 2,1, CV_32F);
					regularizeMessageNbrRobot.at<float>(0,0) = polarsInT[0].cols-1;
					// account for the fact that we do not want to count that reference robot.
					regularizeMessageNbrRobot.at<float>(1,0) = thetatargetrad;
					//let us give to the controller the value of thetatarget in order to compute THETA.
					
					cv::hconcat( regularizeMessageNbrRobot, polarsInT[0], polarsInT[0] );
					
				}
				else
				{
					this->pushing = true;
					//TODO : define if we push or not when there is no neighbours
					
					// REGULARIZATION WITH REGARDS TO THE NUMBER OF NEIGHBOARS :
					thetatargetrad = thetas[0][targetIDX[0]]*PI/180.0f;
					polarsInT[0] = cv::Mat::zeros( 2,1, CV_32F);
					polarsInT[0].at<float>(1,0) = thetatargetrad;
					//let us give to the controller the value of thetatarget in order to compute THETA.
					
					if( !notarget )
					{
						cv::Mat targetpolar = cv::Mat::zeros(2,1,CV_32F);
						targetpolar.at<float>(0,0) = radius[0][targetIDX[0]];
						cv::hconcat( polarsInT[0], targetpolar, polarsInT[0]);
				
						//std::cout << " target polar : " << targetpolar << std::endl;
					}
					else
					{
						std::cout << " NO TARGET." << std::endl;
						//THERE IS NO TARGET VISIBLE : Let the robot assume it is at the correct distance :
						cv::Mat targetpolar = cv::Mat::zeros( 2, 1, CV_32F);
						float desiredradius = 3.0f;
						targetpolar.at<float>(0,0) = desiredradius;
						
						cv::hconcat( polarsInT[0], targetpolar, polarsInT[0]);
						//std::cout << " target polar : " << targetpolar << std::endl;
					}
					
				}	
				
				//----------------------------------------------------
				//----------------------------------------------------
				//----------------------------------------------------
				
				//----------------------------------------------------
				//----------------------------------------------------
				// MESSAGE :
				//----------------------------------------------------
				//----------------------------------------------------
				
				cv::hconcat( polarsInT[0], polarsInT[1], result);
				
				if(this->verbose)
				{
					std::cout << "POLAR POSITION IN TARGET FRAME : " << result << std::endl;
				}
				
				//frameTresholded[0].copyTo(result);
				/* 
				ARCHITECTURE OF RESULT :
				column1 : nbrRobotVisible thetatarget
				next columns : radius theta of visible robot in target frame.
				last column : radius 0  of reference robot in target frame....
				
				next : the same for the previous pictures...
				*/
				
				//----------------------------------------------------
				//----------------------------------------------------
				//----------------------------------------------------
				
				//----------------------------------------------------
				//----------------------------------------------------
				//----------------------------------------------------
				
				//PUSHING :
				if(this->pushing)
				{
					this->pushing = false;
					
					frameTresholded[1] = frameTresholded[0];
					robots[1] = robots[0];
					robots[0].clear();
					contours[1] = contours[0];
					contours[0].clear();
					tcontour[1] = tcontour[0];
					tcontour[0].clear();
					thierarchy[1] = thierarchy[0];
					thierarchy[0].clear();				
					hierarchy[1] = hierarchy[0];
					hierarchy[0].clear();
					targetIDX[1] = targetIDX[0];
	
					thetas[1] = thetas[0];
					thetas[0].clear();
					radius[1] = radius[0];
					radius[0].clear();
				
					polarsInT[1] = polarsInT[0];
				}

			}
			else
			{
				//msleep(1000);
			}
			
			if( char(cv::waitKey(1))=='q' )			
			{
				this->setContinuer(false);
			}
			
			//----------------------------
			//		Publisher
			//----------------------------
		
			//--------------------------------------------
			//
			//	Convert CvImage to ROS image message and publish it to camera/image_processed topic
			//
			//--------------------------------------------
			//sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg();
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", result).toImageMsg();
			img_pub.publish(msg);
			
			
#ifdef debug_v0		
			count_info++;
	
			if(count_info>20000)
			{
				ROS_INFO("OPUSim_RelativeSwarmOdometry::FPS : %f.", CLOCKS_PER_SEC/((float)(clock()-timer)) );
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
	
	
	
	
	float normePoint( const cv::Point& p)
	{
		return sqrt( p.x*p.x+p.y*p.y );
	}
	
	bool alreadyExists( const cv::Point& p, const std::vector<cv::Point>& ps, const int& tresh)
	{
		bool ret = false;
		
		for(int i=ps.size();i--;)
		{
			if( normePoint( p-ps[i] ) < tresh )
			{
				ret = true;
				break;
			}
		}
		
		return ret;
	}
	
	
	
	
	
	void on_low_r_thresh_trackbar(int, void * object)
	{
		  low_r = cv::min(high_r-1, low_r);
		  cv::setTrackbarPos("Low R",objdetectstr.c_str(), low_r);
	}
	void on_high_r_thresh_trackbar(int, void * object)
	{
		  high_r = cv::max(high_r, low_r+1);
		  cv::setTrackbarPos("High R", objdetectstr.c_str(), high_r);
	}
	void on_low_g_thresh_trackbar(int, void * object)
	{
		  low_g = cv::min(high_g-1, low_g);
		  cv::setTrackbarPos("Low G",objdetectstr.c_str(), low_g);
	}
	void on_high_g_thresh_trackbar(int, void * object)
	{
		  high_g = cv::max(high_g, low_g+1);
		  cv::setTrackbarPos("High G", objdetectstr.c_str(), high_g);
	}
	void on_low_b_thresh_trackbar(int, void * object)
	{
		  low_b= cv::min(high_b-1, low_b);
		  cv::setTrackbarPos("Low B",objdetectstr.c_str(), low_b);
	}
	void on_high_b_thresh_trackbar(int, void * object)
	{
		  high_b = cv::max(high_b, low_b+1);
		  cv::setTrackbarPos("High B", objdetectstr.c_str(), high_b);
	}
	

};

#endif
