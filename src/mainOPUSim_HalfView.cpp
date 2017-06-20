#include "./OPUSim_HalfView/OPUSim_HalfView.h"

int main(int argc, char* argv[])
{
	
	
	std::cout << " Usage : robot_number [int] // method [int=0]." << std::endl;
	
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
	
	
	ros::init(argc, argv,std::string("OPUSim_HalfView_"+std::to_string(robot_number)).c_str() );

	OPUSim_HalfView ov(robot_number,method);
	
	ros::spin();
	
	return 0;
	
}

