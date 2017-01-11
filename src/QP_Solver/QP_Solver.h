#ifndef QP_SOLVER_H
#define QP_SOLVER_H

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;

class QP_Solver
{
	private :
	
	cv::Mat Q;
	cv::Mat c;
	
	// Equality Constraints :
	cv::Mat A;
	cv::Mat tA;
	cv::Mat b;
	
	// Inequality Constraints :
	bool inequalityConstraints;
	cv::Mat iA;
	cv::Mat	itA;
	cv::Mat ib;
	
	// Solution variables :
	cv::Mat x;
	cv::Mat lambda;
	cv::Mat sol;
	
	cv::Mat K;
	cv::Mat Kinv;
	
	bool verbose;
	
	public :
	
	QP_Solver(const cv::Mat& Q_, const cv::Mat& c_, const cv::Mat& A_, const cv::Mat& b_) : Q(Q_), c(c_), A(A_), b(b_)
	{
		this->verbose = true;
		this->inequalityConstraints = false;
	}
	
	~QP_Solver()
	{
	
	}
	
	cv::Mat solve()
	{
		cv::Mat zero = cv::Mat::zeros( cv::Size(this->A.rows,this->A.rows), CV_32F);
		cv::transpose(this->A, this->tA);
		
		if( this->verbose )
		{
			std::cout << " tA : " << this->tA << std::endl;
			std::cout << " Q : " << this->Q << std::endl;
		}
		
		cv::hconcat( this->Q, this->tA, this->K);
		cv::hconcat( this->A, zero, zero);
		cv::vconcat( this->K, zero, this->K);
		
		cv::vconcat( (-1.0f)*this->c, this->b, zero);

		this->Kinv = this->K.inv();
		
		if( this->verbose )
		{
			std::cout << " Kinv : " << this->Kinv << std::endl;
			std::cout << " bb : " << zero << std::endl;
		}
		
		
		
		
		cv::Mat solution(this->Kinv * zero);
		solution.copyTo(this->sol);
		
		
		
		cv::Range allj = cv::Range::all();
		cv::Range xi = cv::Range(0,A.cols);
		cv::Range lambdai = cv::Range(A.cols,K.cols);
		
		cv::Range rx[2] = {xi,allj};
		cv::Range rlambda[2] = {lambdai,allj};
		
		this->sol( rx).copyTo(this->x);
		this->sol( rlambda).copyTo(this->lambda);
		
		if( this->verbose )
		{
			std::cout << " sol X : " << this->x << std::endl;
			std::cout << " sol Lambda : " << this->lambda << std::endl;
		}
		
		return this->x;
	}
	
	
	void setInequalityConstraints( const cv::Mat& iA, const cv::Mat& ib)
	{
		this->inequalityConstraints = true;
		
		iA.copyTo( this->iA);
		ib.copyTo( this->ib);
	}
};



class QP_SolverIC
{
	private :
	
	cv::Mat Q;
	cv::Mat c;
	
	// Equality Constraints :
	cv::Mat A;
	cv::Mat tA;
	cv::Mat b;
	
	// Inequality Constraints :
	bool inequalityConstraints;
	cv::Mat iA;
	cv::Mat	itA;
	cv::Mat ib;
	
	// Solution variables :
	cv::Mat x;
	cv::Mat lambda;
	cv::Mat sol;
	
	cv::Mat K;
	cv::Mat Kinv;
	cv::Mat iK;
	cv::Mat iKinv;
	
	bool verbose;
	
	public :
	
	QP_SolverIC(const cv::Mat& Q_, const cv::Mat& c_, const cv::Mat& iA_, const cv::Mat& ib_) : Q(Q_), c(c_), iA(iA_), ib(ib_)
	{
		this->verbose = true;
		this->inequalityConstraints = false;
	}
	
	~QP_SolverIC()
	{
	
	}
	
	cv::Mat solve()
	{
		cv::Mat zero = cv::Mat::zeros( cv::Size(this->iA.rows,this->iA.rows), CV_32F);
		cv::transpose(this->iA, this->itA);
		
		if( this->verbose )
		{
			std::cout << " tA : " << this->itA << std::endl;
			std::cout << " Q : " << this->Q << std::endl;
		}
		
		cv::hconcat( this->Q, this->itA, this->K);
		cv::hconcat( this->iA, zero, zero);
		cv::vconcat( this->K, zero, this->K);
		
		cv::vconcat( (-1.0f)*this->c, this->ib, zero);

		cv::Mat izeros;
		cv::hconcat( cv::Mat::zeros( cv::Size(this->ib.rows,this->itA.rows), CV_32F ), cv::Mat::zeros( cv::Size(this->ib.rows,this->itA.rows), CV_32F), izeros);
		cv::Mat ieyes;
		cv::hconcat( cv::Mat::eye( cv::Size(this->ib.rows,this->iA.rows), CV_32F ), (-1.0f)*cv::Mat::eye( cv::Size(this->ib.rows,this->iA.rows), CV_32F), ieyes);
		cv::Mat idummy;
		cv::vconcat( izeros, ieyes, idummy);
		
		cv::hconcat( this->K, idummy, this->iK);
		
		
		if( this->verbose )
		{
			std::cout << " iK : " << this->iK << std::endl;
		}
		
		
		cv::Mat tiK;
		cv::transpose( this->iK, tiK);
		this->iK = tiK*this->iK;
		zero = tiK*zero;
		
		this->iKinv = this->iK.inv();
		
		if( this->verbose )
		{
			std::cout << " iK: " << this->iK << std::endl;
			std::cout << " iKinv : " << this->iKinv << std::endl;
			std::cout << " bb : " << zero << std::endl;
		}
		
		
		
		
		cv::Mat solution(this->iKinv * zero);
		solution.copyTo(this->sol);
		
		
		
		cv::Range allj = cv::Range::all();
		cv::Range xi = cv::Range(0,this->iA.cols);
		cv::Range lambdai = cv::Range(this->iA.cols,this->iK.cols);
		
		cv::Range rx[2] = {xi,allj};
		cv::Range rlambda[2] = {lambdai,allj};
		
		this->sol( rx).copyTo(this->x);
		this->sol( rlambda).copyTo(this->lambda);
		
		if( this->verbose )
		{
			std::cout << " solution : " << this->sol << std::endl;
			std::cout << " sol X : " << this->x << std::endl;
			std::cout << " sol Lambda : " << this->lambda << std::endl;
		}
		
		return this->x;
	}
	
	
	void setInequalityConstraints( const cv::Mat& iA, const cv::Mat& ib)
	{
		this->inequalityConstraints = true;
		
		iA.copyTo( this->iA);
		ib.copyTo( this->ib);
	}
};

#endif
