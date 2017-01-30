#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "./Quaternion.h"
#include "./EKF/EKF.h"

#include <iostream>
#include <string>
#include <mutex>
#include <thread>

using namespace std;


#define debug_lvl1
//#define debug_lvl2
//#define debug_lvl3

#define Gacc 9.81f
//#define COMPL1		//COMPLEMENTARY FILTER 1st ORDER
#define EKF_USE		// KALMAN FILTER
//#define EKF_ACCGYRO

void Euler2Quaternion(geometry_msgs::Quaternion* q, float roll, float pith, float yaw);


class IMUListener
{

	public :
	
	IMUListener(const int& rate_ = 100,const float& regul_ = 1.0f, const float& nobsaccx=1e-1f, const float& nobsaccy=1e-1f, const float& nobsaccz=1e-1f, const float& nobsgyx=1e-1f, const float& nobsgyy=1e-1f, const float& nobsgyz=1e-1f  ) : rate(rate_), regul(regul_), continuer(true)
	{
	
#ifdef debug_lvl1
		//Gestion ecriture dans un fichier :
		filepath = string("./logIMUOdometry.txt");
		log = fopen(filepath.c_str(), "w+");
		if(log == NULL)
		{
			cout << "ERROR : cannot open the file LOG." << endl;
			exit(1);
		}
		else
			cout << "File opened LOG." << endl;
		//------------------------------------------
		//Ecriture  :
   	stringstream s;
		s << "positionX" << " | " << "positionY" << " | " << "positionZ" << " | " << "angleX" << " | " << "angleY" << " | " << "angleZ" << " | " << "vx" << " | " << "vy" << " | " << "vz" << " | " << "acx"  << " | " << "acy" << " | " << "acz" << " | " << "gyx" << " | " << "gyy" << " | " << "gyz" ;
		s << endl;
		fputs( s.str().c_str(), log);
#endif

				
		imu_pub = nh.advertise<sensor_msgs::Imu>("IMUListener/imu",10);
		imu_sub = nh.subscribe("/imu", 10, &IMUListener::callbackIMU, this);
		
		path_pub = nh.advertise<nav_msgs::Path>("IMUListener/Path",10);
		pose_stamped_pub = nh.advertise<geometry_msgs::PoseStamped>("IMUListener/POSESTAMPED",10);
		
		imu_data = sensor_msgs::Imu();
		
		rangeAcc = 2.0f*Gacc;
		precisionAcc = pow(2.0f, 16-1);
		rangeGyro = 1.0f*250.0f;
		precisionGyro = pow(2.0f, 16-1);
		sgyro = rangeGyro/precisionGyro;
		sacc = rangeAcc/precisionAcc;
		
		
		//TODO : figure out how to use correctly those constants :
		sacc = 1.0f;
		sgyro = 1.0f;
		
		/*
		sacx = atoi("AcX");
		sacy = atoi("AcY");
		sacz = atoi("AcZ");
		sgyx = atoi("GyX");
		sgyy = atoi("GyY");
		sgyz = atoi("GyZ");
		*/
		
		acx = 0;
		acy = 0;
		acz = 0;
		gyx = 0;
		gyy = 0;
		gyz = 0;
		
		angleX = 0.0f;
		angleY = 0.0f;
		angleZ = 0.0f;
		dt = 1.0f/rate;
		
		pose = new Mat<float>(0.0f, 3,1);
		biasAcc = new Mat<float>(0.0f,3,1);
		initBias = false;
		initBiasAccX = false;
		initBiasAccY = false;
		initBiasAccZ = false;
		
		info_count = 1001;
		
		
		//--------------------------------------------------------
		//--------------------------------------------------------
		//EKF :
		#ifndef EKF_ACCGYRO
		nbrstate = 12;
		nbrcontrol = 0;
		nbrobs = 6;
		dtEKF = dt;
		
		stdnoise = 1e-1f;
		stdnoise_obs = 1e-1f;
		float stdnoise_obsaccx= nobsaccx;
		float stdnoise_obsaccy= nobsaccy;
		float stdnoise_obsaccz= nobsaccz;
		float stdnoise_obsgyx= nobsgyx;
		float stdnoise_obsgyy= nobsgyy;
		float stdnoise_obsgyz= nobsgyz;

		ext = false;
		filteron = true;
		noise = false;

		EKFPose = Mat<float>((float)0,nbrstate,1);  
		EKFPoseCovar = Mat<float>((float)0,nbrstate,nbrstate);  
		instanceEEKF = new EEKF<float>(nbrstate,nbrcontrol,nbrobs,dtEKF,stdnoise,EKFPose,ext,filteron,noise);

		Mat<float> A((float)0,nbrstate,nbrstate);
		//for(int i=1;i<=nbrstate/2;i++)	A.set((float)1,i,i);
		for(int i=1;i<=nbrstate;i++)	A.set((float)1,i,i);
		//unstable if the velocity is propagated...
		for(int i=1;i<=nbrstate/2;i++)	A.set((float)dtEKF,i,nbrstate/2+i);
		A.afficher();
		instanceEEKF->initA(A);
		
		Mat<float> C((float)0,nbrobs,nbrstate);
		for(int i=1;i<=nbrobs;i++)	C.set((float)1,i,nbrobs+i);
		C.afficher();
		instanceEEKF->initC(C);
		
		/*
		Mat<float> B((float)0,nbrstate,nbrcontrol);
		for(int i=1;i<=nbrcontrol;i++)	B.set((float)1,nbrcontrol+i,i);
		B.afficher();
		instanceEEKF->initB(B);
		*/
		
		Q = Mat<float>(0.0f,nbrstate,nbrstate);
		for(int i=1;i<=nbrstate;i++)	Q.set( stdnoise, i,i);
		Q.afficher();
		instanceEEKF->initQ(Q);
		
		R = Mat<float>(0.0f,nbrobs,nbrobs);
		//for(int i=1;i<=nbrobs;i++)	R.set( stdnoise_obs, i,i);
		R.set( stdnoise_obsaccx, 1,1);
		R.set( stdnoise_obsaccy, 2,2);
		R.set( stdnoise_obsaccz, 3,3);
		R.set( stdnoise_obsgyx, 4,4);
		R.set( stdnoise_obsgyy, 5,5);
		R.set( stdnoise_obsgyz, 6,6);
		R.afficher();
		instanceEEKF->initR(R);
		
		#else
		
		nbrstate = 6;
		nbrcontrol = 3;
		nbrobs = 2;
		dtEKF = dt;
		
		stdnoise = 1e-3f;
		stdnoise_obs = 1e-3f;

		ext = false;
		filteron = true;
		noise = false;

		EKFPose = Mat<float>((float)0,nbrstate,1);  
		EKFPoseCovar = Mat<float>((float)0,nbrstate,nbrstate);  
		instanceEEKF = new EEKF<float>(nbrstate,nbrcontrol,nbrobs,dtEKF,stdnoise,EKFPose,ext,filteron,noise);

		Mat<float> A((float)0,nbrstate,nbrstate);
		for(int i=1;i<=nbrstate;i++)	A.set((float)1,i,i);
		//unstable if the velocity is propagated...
		for(int i=1;i<=nbrstate/2;i++)	A.set((float)-dtEKF,i,nbrstate/2+i);
		A.afficher();
		instanceEEKF->initA(A);
		
		Mat<float> C((float)0,nbrobs,nbrstate);
		for(int i=1;i<=nbrobs;i++)	C.set((float)1,i,i);
		C.afficher();
		instanceEEKF->initC(C);
		
		
		Mat<float> B((float)0,nbrstate,nbrcontrol);
		for(int i=1;i<=nbrcontrol;i++)	B.set((float)1,i,i);
		B.afficher();
		instanceEEKF->initB(B);
		
		
		Q = Mat<float>(0.0f,nbrstate,nbrstate);
		for(int i=1;i<=nbrstate/2;i++)	Q.set( stdnoise, i,i);
		//bias :
		for(int i=nbrstate/2+1;i<=nbrstate;i++)	Q.set( stdnoise*1e-5, i,i);
		Q.afficher();
		instanceEEKF->initQ(Q);
		
		R = Mat<float>(0.0f,nbrobs,nbrobs);
		for(int i=1;i<=nbrobs;i++)	R.set( stdnoise_obs, i,i);
		R.afficher();
		instanceEEKF->initR(R);
		
		#endif
		
		//--------------------------------------------------------
		//--------------------------------------------------------
		
		//this->mainLoop();	
		this->t = new std::thread(&IMUListener::mainLoop, this);
	}
	
	~IMUListener()
	{
		this->continuer = false;
		
		if(t->joinable())
		{
			t->join();
		}
		
		delete t;
		
		delete pose;
		delete biasAcc;
		delete instanceEEKF;
		
#ifdef debug_lvl1
		//------------------------------------
		//Fermeture du fichier :
		if(fclose(log) == EOF)
		{
			cout << "ERROR : cannot close the file." << endl;
			exit(1);
		}
		else
			cout << "File closed." << endl;
			
		//--------------------------------
		//--------------------------------
#endif				
	}
	
	void callbackIMU( const sensor_msgs::Imu& imu_msg)
	{
		this->mutexRessources.lock();
		this->imu_datas.push_back( imu_msg);
		
		this->mutexRessources.unlock();
	}
	
	void mainLoop()
	{
		ros::Rate r(rate);
		
		while(this->continuer)
		{
			clock_t timer = clock();
			
			//if(instanceCOMPORT->listen() != 0)
			//std::cout << " NBR IMU MSGS RECEIVED :: " << this->imu_datas.size() << std::endl;
			
			if( this->imu_datas.size() > 0 )
			{
				this->mutexRessources.lock();
				this->current_imu_data = this->imu_datas[this->imu_datas.size()-1];
				this->imu_datas.clear();
				this->mutexRessources.unlock();
											
								
				//acx = this->current_imu_data.linear_acceleration.x;
				acx = this->current_imu_data.linear_acceleration.z;
				
				if(!initBiasAccX)
				{
					initBiasAccX = true;
					biasAcc->set(acx,1,1);
					
					if(initBiasAccY && initBiasAccZ)
						initBias = true;
				}
				
				acy = this->current_imu_data.linear_acceleration.y;
					
				if(!initBiasAccY)
				{
					initBiasAccY = true;
					biasAcc->set(acy,2,1);
					
					if(initBiasAccX && initBiasAccZ)
						initBias = true;
				}
				
				//acz = this->current_imu_data.linear_acceleration.z;
				acz = -this->current_imu_data.linear_acceleration.x;
					
				if(!initBiasAccZ)
				{
					initBiasAccZ = true;
					biasAcc->set(acz,3,1);
					
					if(initBiasAccY && initBiasAccX)
						initBias = true;
				}	
				
				//gyx = this->current_imu_data.angular_velocity.x;
				gyx = this->current_imu_data.angular_velocity.z;
				gyy = this->current_imu_data.angular_velocity.y;
				//gyz = this->current_imu_data.angular_velocity.z;
				gyz = -this->current_imu_data.angular_velocity.x;
			}
			
			angular_vel.x = (float)gyx/regul;
			angular_vel.y = (float)gyy/regul;
			angular_vel.z = (float)gyz/regul;
			
			linear_acc.x = (float)acx/regul;
			linear_acc.y = (float)acy/regul;
			linear_acc.z = (float)acz/regul;
			
#ifdef debug_lvl1
			//ROS_INFO(" READ : acc x : %f ; y : %f ; z : %f ;; vel x : %f ; y : %f ; z : %f", linear_acc.x, linear_acc.y, linear_acc.z, angular_vel.x, angular_vel.y, angular_vel.z);
			//ROS_INFO(" CONV : acc x : %f ; y : %f ; z : %f ;; vel x : %f ; y : %f ; z : %f", linear_acc.x*sacc, linear_acc.y*sacc, linear_acc.z*sacc, angular_vel.x*sgyro, angular_vel.y*sgyro, angular_vel.z*sgyro);
#endif		


			//--------------------------
			//		SLAM
			//-------------------------
			Mat<float> rotAG2Gl(transpose( Euler2Rot( angleX,angleY,angleZ) ) );
			Mat<float> acc(3,1);
			acc.set( acx*sacc,1,1);
			acc.set( acy*sacc,2,1);
			acc.set( acz*sacc,3,1);
			
			acc = rotAG2Gl * acc;
			//instantaneous felt accelerations in the world frame.
			if(initBias)
			{
				initBias = false;
				*biasAcc = rotAG2Gl * ((sacc)*(*biasAcc));
				// bias in the world frame, which takes into account the gravity :
				// this initialization assume that the IMU is resting its frame being equal to the worl frame.
				
			}
			
			acc -= (*biasAcc);
			//instantaneous accelerations in the world frame ( especially no longer any effect from gravity effect).
			
#ifdef EKF_USE	
	#ifndef EKF_ACCGYRO	
			//------------------------
			//		EKF
			//--------------------
			Mat<float> measure(6,1);
			measure.set( acc.get(1,1)*dt,1,1);
			measure.set( acc.get(2,1)*dt,2,1);
			measure.set( acc.get(3,1)*dt,3,1);
			measure.set( gyx*sgyro,4,1);
			measure.set( gyy*sgyro,5,1);
			measure.set( gyz*sgyro,6,1);
			
			instanceEEKF->measurement_Callback(measure);
			//instanceEEKF->setCommand(updatePose);
			instanceEEKF->state_Callback();
			EKFPose = instanceEEKF->getX();
			EKFPoseCovar = instanceEEKF->getSigma();
			
			//ROS_INFO("EKF GLOBAL POSE : ");
			//transpose(EKFPose).afficher();
			//EKFPoseCovar.afficher();
			
			//---------------------------	
			*pose = extract(EKFPose, 1,1, 3,1);
			
	#else
			//------------------------
			//		EKF_ACCGYRO
			//--------------------
			Mat<float> input(3,1);
			Mat<float> measure(2,1);
			
			input.set( gyx*sgyro,1,1);
			input.set( gyy*sgyro,2,1);
			input.set( gyz*sgyro,3,1);
			
			float accRoll = atan2( acy*sacc, sqrt(acx*acx*sacc*sacc+acz*acz*sacc*sacc));
			float accPitch = atan2( acy*sacc*cos(accRoll*PI/180.0f), acz*sacc);
			measure.set( accRoll, 1,1);
			measure.set( accPitch, 2,1);
			
			
			//ROS_INFO("MEASURE : ACC");
			//transpose(measure).afficher();
			//ROS_INFO("INPUT : GYRO");
			//transpose(input).afficher();
			instanceEEKF->setCommand(input);
			instanceEEKF->measurement_Callback(measure);
			//instanceEEKF->setCommand(updatePose);
			instanceEEKF->state_Callback();
			EKFPose = instanceEEKF->getX();
			EKFPoseCovar = instanceEEKF->getSigma();
			
			//ROS_INFO("EKF ANGLE BIAS : ");
			//transpose(EKFPose).afficher();
			//EKFPoseCovar.afficher();
			
			//---------------------------	
			*pose = extract(EKFPose, 1,1, 3,1);
	
	
	#endif
#endif
//#else
			
			*pose += acc;	
	#ifdef debug_lvl1			
			ROS_INFO("POSE : x = %f ; y = %f ; z = %f", pose->get(1,1),	pose->get(2,1), pose->get(3,1));
	#endif				
			
//#endif			
			//-------------------------
			//		FILTERING :
			//-------------------------
#ifdef COMPL1			
			//Complementary Filter :
			float lambda1 = 0.02;
			angleX = (1.0f-lambda1)*(angleX + gyx*sgyro*dt)+lambda1*acx*sacc;
			angleY = (1.0f-lambda1)*(angleY + gyy*sgyro*dt)+lambda1*acy*sacc;
			angleZ = (1.0f-lambda1)*(angleZ + gyz*sgyro*dt)+lambda1*acz*sacc;
#endif

#ifndef COMPL1
	#ifdef EKF_USE
		#ifndef EKF_ACCGYRO
			// Kalman Filter :
			angleX = EKFPose.get(4,1);			
			angleY = EKFPose.get(5,1);
			angleZ = EKFPose.get(6,1);
		#else
			// Kalman Filter ACCGYRO:
			angleX = EKFPose.get(1,1);			
			angleY = EKFPose.get(2,1);
			angleZ = EKFPose.get(3,1);
		#endif
	#endif
#endif

#ifndef COMPL1
	#ifndef EKF_USE
			//No Filter at all :
			angleX = angleX + gyx*sgyro*dt;
			angleY = angleY + gyy*sgyro*dt;
			angleZ = angleZ + gyz*sgyro*dt;
	#endif
#endif			

			Euler2Quaternion(&q, angleX,angleY,angleZ);
			//ROS_INFO("ANGLE : roll : %f ; pitch : %f ; yaw : %f", angleX*3.1415f/180.0f,angleY*3.1415f/180.0f,angleZ*3.1415f/180.0f);
			
			
			//--------------------------
			//		PUBLISHER
			//-------------------------
			
			imu_data.header.stamp = ros::Time(0);
			imu_data.header.frame_id = "map";
			
			imu_data.orientation = q;
			imu_data.angular_velocity = angular_vel;
			imu_data.linear_acceleration = linear_acc;
			
			imu_pub.publish(imu_data);
			
			// SLAM PUBLISHER :
			std_msgs::Header head_path;
			head_path.frame_id = "map";
			
			geometry_msgs::Pose path_pose;
			path_pose.position.x = EKFPose.get(1,1);
			path_pose.position.y = EKFPose.get(2,1);
			path_pose.position.z = EKFPose.get(3,1);
			
			Quat q = Euler2Qt( EKFPose.get(4,1), EKFPose.get(5,1), EKFPose.get(6,1) );
			path_pose.orientation.x = q.x;
			path_pose.orientation.y = q.y;
			path_pose.orientation.z = q.z;
			path_pose.orientation.w = q.w;
			
			
			geometry_msgs::PoseStamped path_poseStamped;
			path_poseStamped.header.stamp = ros::Time(0);
			path_poseStamped.header.frame_id = "map";
			path_poseStamped.pose = path_pose;
			
			path.poses.push_back(path_poseStamped);
			path.header.stamp = ros::Time(0);
			path.header.frame_id = "map";
			
			//--------------------------------
			//--------------------------------
			
			
			path_pub.publish(path);
			pose_stamped_pub.publish(path_poseStamped);
				
			//-------------------------
			//-------------------------
			
			
#ifdef debug_lvl1			
			if(info_count >= 1000)
			{
				info_count = 0;
				ROS_INFO("IMU_COMport_Listener :: EXECUTION : %f Hz.", (float)(CLOCKS_PER_SEC/(clock()-timer)) );
				//ROS_INFO(" ACC x : %f ; y : %f ; z : %f ;; VEL x : %f ; y : %f ; z : %f", linear_acc.x, linear_acc.y, linear_acc.z, angular_vel.x, angular_vel.y, angular_vel.z);				
			}
			else
				info_count++;
#endif



#ifdef debug_lvl1
		//------------------------------------------
		//------------------------------------------
		//Ecriture  :
    	stringstream s;
		s << path_pose.position.x << " | " << path_pose.position.y << " | " << path_pose.position.z << " | " << angleX << " | " << angleY << " | " << angleZ << " | " << EKFPose.get(7,1) << " | " << EKFPose.get(8,1) << " | " << EKFPose.get(9,1) << acx*dt  << " | " << acy*dt << " | " << acz*dt << " | " << gyx << " | " << gyy << " | " << gyz ;
		s << endl;
		fputs( s.str().c_str(), log);
#endif		
			r.sleep();
			
		}
		
	}		
	
	private :
	
	std::thread* t;
	bool continuer;
	
	string filepath;
	FILE* log;
	
	int rate;	
	float regul;
	float dt;
	
	std::mutex mutexRessources;
	ros::NodeHandle nh;
	ros::Publisher imu_pub;
	ros::Publisher path_pub;
	ros::Publisher pose_stamped_pub;
	
	nav_msgs::Path path;
	
	ros::Subscriber imu_sub;
	std::vector<sensor_msgs::Imu> imu_datas;
	sensor_msgs::Imu current_imu_data;

	float rangeAcc;
	float precisionAcc;
	float rangeGyro;
	float precisionGyro;
	
	string data;
	float sacc;
	float sgyro;
	
	/*
	int acx;
	int acy;
	int acz;
	int gyx;
	int gyy;
	int gyz;
	
	int sacx;
	int sacy;
	int sacz;
	int sgyx;
	int sgyy;
	int sgyz;
	*/
	float acx;
	float acy;
	float acz;
	float gyx;
	float gyy;
	float gyz;
	
	float angleX;
	float angleY;
	float angleZ;
	
	Mat<float>* pose;
	Mat<float>* biasAcc;
	bool initBias;
	bool initBiasAccX;
	bool initBiasAccY;
	bool initBiasAccZ;
	
	geometry_msgs::Quaternion q;
	geometry_msgs::Vector3 angular_vel;
	geometry_msgs::Vector3 linear_acc;
	sensor_msgs::Imu imu_data;
	
	int info_count;
	
	//------------------------------------------
	//-----------------------------------------
	//EEKF :
	int nbrstate;
	int nbrcontrol;
	int nbrobs;
	float dtEKF;
	float stdnoise;
	float stdnoise_obs;
	bool ext;
	bool filteron;
	bool noise;

	Mat<float> EKFPose;  
	Mat<float> EKFPoseCovar;  
	EEKF<float>* instanceEEKF;
	Mat<float> Q;
	Mat<float> R;
	
	//------------------------------------------
	//-----------------------------------------
};


void Euler2Quaternion(geometry_msgs::Quaternion* q, float roll, float pitch, float yaw)
{
	Quat qq = Euler2Qt(roll,pitch,yaw);
	q->x = qq.x;
	q->y = qq.y;
	q->z = qq.z;
	q->w = qq.w;
}

int main(int argc, char* argv[])
{

	ros::init(argc,argv,"OPUSim_IMUOdometry");	
		
	int rate = 10;
	if(argc>1)
		rate = atoi(argv[1]);
	
	float regul = 1.0f;
	
	float nobsaccx=2e-1f;
	if(argc>2)
		nobsaccx = atof(argv[2]);
	float nobsaccy=2e-1f;
	if(argc>3)
		nobsaccy = atof(argv[3]);
	float nobsaccz=2e-1f;
	if(argc>4)
		nobsaccz = atof(argv[4]);

	float nobsgyx=1e-1f;
	if(argc>5)
		nobsgyx = atof(argv[5]);
	float nobsgyy=1e-1f;
	if(argc>6)
		nobsgyy = atof(argv[6]);
	float nobsgyz=1e-1f;
	if(argc>7)
		nobsgyz = atof(argv[7]);
		
	IMUListener instanceIMUL(rate,regul, nobsaccx, nobsaccy, nobsaccz, nobsgyx, nobsgyy, nobsgyz);
	
	ros::spin();
	
	return 0;
}

