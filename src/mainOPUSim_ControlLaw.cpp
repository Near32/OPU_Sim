#include "./OPUSim_ControlLaw/OPUSim_ControlLaw.h"

int main(int argc, char* argv[])
{
	
	
	std::cout << " Usage : robot_number [int] // emergencyBreak [int=0] // verbose [int=1] //  gain_=4.0f // R_=3.0f // a_=-1.0f // epsilon_=10.0f // kv_=-0.1f // kw_=0.2f // Omega_=1.0f." << std::endl;
	
	int robot_number = 0;
	if(argc>1)
	{
		robot_number = atoi(argv[1]);
	}
	
	bool emergencyBreak = false;
	if(argc>2)
	{
		emergencyBreak = (atoi(argv[2])==1?true:false);
	}
	
	bool verbose = true;
	if(argc>3)
	{
		verbose = (atoi(argv[3])==1?true:false);
	}
	
	float gain_=4.0f;
	if(argc>4)
	{
		gain_ = atof(argv[4]);
	}
	
	float R_=3.0f;
	if(argc>5)
	{
		R_ = atof(argv[5]);
	}
	
	float a_=-1.0f;
	if(argc>6)
	{
		a_ = atof(argv[6]);
	}
	
	float epsilon_=10.0f;
	if(argc>7)
	{
		epsilon_ = atof(argv[7]);
	}
	
	float kv_=-0.1f;
	if(argc>8)
	{
		kv_ = atof(argv[8]);
	}
	
	float kw_=0.2f;
	if(argc>9)
	{
		kw_ = atof(argv[9]);
	}
	
	float Omega_=1.0f;
	if(argc>10)
	{
		Omega_ = atof(argv[10]);
	}
	
	
	ros::init(argc, argv,std::string("OPUSim_ControlLaw_"+std::to_string(robot_number)).c_str() );

	OPUSim_ControlLaw cl(robot_number, emergencyBreak, verbose, gain_, R_, a_, epsilon_, kv_, kw_, Omega_);
	
	ros::spin();
	
	return 0;
	
}

