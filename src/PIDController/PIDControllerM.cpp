#include "PIDControllerM.h"

PIDcontrollerM::PIDcontrollerM() : err( Mat<T>((T)0,1,1)), err_old( Mat<T>((T)0,1,1)), err_sum( Mat<T>((T)0,1,1)), consigne( Mat<T>((T)0,1,1)), value( Mat<T>((T)0,1,1))
{
    Kp = (T)1;
    Ki = (T)1;
    Kd = (T)1;
    
}

PIDcontrollerM::PIDcontrollerM(T Kp, T Ki, T Kd) : err( Mat<T>((T)0,1,1)), err_old( Mat<T>((T)0,1,1)), err_sum( Mat<T>((T)0,1,1)), consigne( Mat<T>((T)0,1,1)), value( Mat<T>((T)0,1,1))
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

PIDcontrollerM::~PIDcontrollerM()
{
    
}
    
void PIDcontrollerM::reset()
{
    err_old = Mat<T>((T)0,1,1);
    err_sum = Mat<T>((T)0,1,1);
}
void PIDcontrollerM::setConsigne(const Mat<T>& consigne)
{
    this->consigne = consigne;
}

Mat<T> PIDcontrollerM::update(const Mat<T>& currentValue, T dt = (T)0.1)
{
    err = consigne - currentValue;
    err_sum = err_sum + err;
    Mat<T> err_diff = err-err_old;
    err_old = err;   
    
    value = Kp*(err + Ki*err_sum + (Kd/dt)*err_diff);
    
    return value;
}

void PIDcontrollerM::setKp(T Kp)
{
    this->Kp = Kp;
}

void PIDcontrollerM::setKi(T Ki)
{
    this->Ki = Ki;
}

void PIDcontrollerM::setKd(T Kd)
{
    this->Kd = Kd;
}

void PIDcontrollerM::set(T Kp, T Ki,T Kd)
{
    setKp(Kp);
    setKi(Ki);
    setKd(Kd);
}
