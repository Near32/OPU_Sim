#include "./OPUSim_Obstacles/OPUSim_Obstacles.h"

int main(int argc, char* argv[])
{
	
	
	std::cout << " Usage : robot_number [int] // method [int=0] // verbose [int=0] // everycolours [int=1]." << std::endl;
	
	int robot_number = 0;
	if(argc>1)
	{
		robot_number = atoi(argv[1]);
	}
	
	int method = 0;	//ordered standard clipping
	if( argc>2)
	{
		method = atoi(argv[2]);
	}
	
	bool verbose = false;
	if( argc>3)
	{
		verbose = (atoi(argv[3])==1?true:false);
	}
	
	int low_r=0, low_g=30, low_b=0;
	int high_r=50, high_g=255, high_b=50;
	
	bool everycolours = true;
	if(argc>4)
	{
		everycolours = (atoi(argv[4]) == 1? true : false);
	}
	
	if(everycolours)
	{
		low_r = 5;
		low_g = 5;
		low_b = 5;
		high_r = 250;
		high_g = 250;
		high_b = 250;
	}
	
	ros::init(argc, argv,std::string("OPUSim_Obstacles_"+std::to_string(robot_number)).c_str() );

	OPUSim_Obstacles obs(robot_number,method,verbose, low_r, low_g, low_b, high_r, high_g, high_b);
	
	ros::spin();
	
	return 0;
	
}

