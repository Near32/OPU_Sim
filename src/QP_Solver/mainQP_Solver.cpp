#include "./QP_Solver.h"

int main(int argc, char* argv[])
{
	cv::Mat Q = cv::Mat::eye(cv::Size(2,2), CV_32F);
	cv::Mat c = cv::Mat::zeros(cv::Size(1,2), CV_32F);
	cv::Mat A = cv::Mat::zeros(cv::Size(2,2), CV_32F);
	//cv::Mat A = cv::Mat::zeros(cv::Size(2,1), CV_32F);
	A.at<float>(0,0) = 1.0f;
	A.at<float>(1,0) = 1.0f;
	A.at<float>(1,1) = 1.0f;
	A.at<float>(0,1) = 0.0f;
	cv::Mat b = cv::Mat::zeros(cv::Size(1,2), CV_32F);
	//cv::Mat b = cv::Mat::zeros(cv::Size(1,1), CV_32F);
	b.at<float>(0,0) = 10.0f;
	b.at<float>(1,0) = 15.0f;
	
	QP_SolverIC qps( Q, c, A, b);
	
	cv::Mat x;
	x = qps.solve();
	
	std::cout << x << std::endl;
	
	return 0;
}
