#ifndef PIDcontroller_H
#define PIDcontroller_H

template<typename T>
class PIDcontroller
{
    private :
    T Kp;
    T Ki;
    T Kd;
    
    T err;
    T err_old;
    T err_sum;
    T consigne;
    T value;
    
    
    public :
    
    PIDcontroller()
    {
        Kp = (T)1;
        Ki = (T)1;
        Kd = (T)1;
        err = (T)0;
        err_old = (T)0;
        err_sum = (T)0;
        consigne = (T)0;    
        value =(T)0;
    }
    
    PIDcontroller(T Kp, T Ki, T Kd)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        err = (T)0;
        err_old = (T)0;
        err_sum = (T)0;
        consigne = (T)0;        
        value = (T)0;
    }
    
    ~PIDcontroller()
    {
        
    }
        
    void reset()
    {
        err_old = (T)0;
        err_sum = (T)0;
    }
    void setConsigne(T consigne)
    {
        this->consigne = consigne;
    }
    
    T update(T currentValue, T dt = (T)0.1)
    {
        err = consigne - currentValue;
        err_sum = err_sum + err;
        T err_diff = err-err_old;
        err_old = err;   
        
        value = Kp*(err + Ki*err_sum + Kd*err_diff/dt);
        
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
