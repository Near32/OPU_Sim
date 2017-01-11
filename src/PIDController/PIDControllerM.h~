#ifndef PIDcontrollerM_H
#define PIDcontrollerM_H
#include "../Mat/Mat.h"

template<typename T>
class PIDControllerM
{
    private :
    T Kp;
    T Ki;
    T Kd;
    
    Mat<T> err;
    Mat<T> err_old;
    Mat<T> err_sum;
    Mat<T> consigne;
    Mat<T> value;
    
    
    public :
    
    PIDControllerM() : err( Mat<T>((T)0,1,1)), err_old( Mat<T>((T)0,1,1)), err_sum( Mat<T>((T)0,1,1)), consigne( Mat<T>((T)0,1,1)), value( Mat<T>((T)0,1,1))
    {
        Kp = (T)1;
        Ki = (T)1;
        Kd = (T)1;
        
    }
    
    PIDControllerM(T Kp, T Ki, T Kd) : err( Mat<T>((T)0,1,1)), err_old( Mat<T>((T)0,1,1)), err_sum( Mat<T>((T)0,1,1)), consigne( Mat<T>((T)0,1,1)), value( Mat<T>((T)0,1,1))
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
    }
    
    ~PIDControllerM()
    {
        
    }
        
    void reset()
    {
        err_old = Mat<T>((T)0,1,1);
        err_sum = Mat<T>((T)0,1,1);
    }
    void setConsigne(const Mat<T>& consigne)
    {
        this->consigne = consigne;
        err_sum = Mat<T>((T)0, consigne.getLine(), consigne.getColumn());
        err_old = Mat<T>((T)0, consigne.getLine(), consigne.getColumn());
    }
    
    Mat<T> update(const Mat<T>& currentValue, T dt = (T)0.1)
    {
        err = consigne - currentValue;
        err_sum = err_sum + err;
        Mat<T> err_diff(err-err_old);
        err_old = err;   
        
        value = Kp*(err + Ki*err_sum + (Kd/dt)*err_diff);
        
        return value;
    }
    
    void setKp(T Kp)
    {
        this->Kp = Kp;
    }
    
    void setKi(T Ki)
    {
        this->Ki = Ki;
    }
    
    void setKd(T Kd)
    {
        this->Kd = Kd;
    }
    
    void set(T Kp, T Ki,T Kd)
    {
        setKp(Kp);
        setKi(Ki);
        setKd(Kd);
    }
    
};

#endif
