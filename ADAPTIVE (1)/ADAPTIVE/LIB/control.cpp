#include "control.h"

// Вычисление коэффициентов цифровых ПИД регуляторов - первый вариант (без фильтрации D члена)
void DIGPIDCOEFF(float q[], const float kP, const float kI,const float kD, const float TAU);

{  
  float K=kP, tI=kP/kI, tD=kD/kP;
  q[0]=K*(1+tD/TAU);
  q[1]=-K*(1+2*tD/TAU-TAU/tI);
  q[2]=K*tD/TAU;
  q[3]=1;
  q[4]=0;
}

// расчет коэффициентов альфа для адаптивного ПИД
void SetAlpha(const float Ts, const float xi,const float tau,float alp[]);
{
    float p0,p1,p2;
    float a,wn;    
    wn=4/Ts/xi;
    p0=exp(-2*xi*wn*tau);
    p1=-2*exp(-xi*wn*tau)*cos(wn*tau*sqrt(1-xi*xi));
    a=40;
    p2=exp(-a*tau);

    alp[0]=p1+p2;
    alp[1]=p1*p2+p0;
    alp[3]=p0*p2;
}

// расчет коэффициентов адаптивного ПИД
void ADPIDCOEFF(float alp[], float a[], float b[], float q[]);
{
    float b1;
    b1=b[0];
    if (abs(b1)<1.E-10 && b1<0.0)
        b1=-1.E-10;
    if (abs(b1)<1.E-10 && b1>0.0)
        b1=1.E-10;
    
    q[0]=(alp[0]-a[1]+1)/b1;
    q[1]=(alp[1]-a[2]+a[1]-q[0]*b[1])/b1;
    q[2]=(alp[2]+a[2]-q[1]*b[1])/b1;
    q[3]=1;
    q[4]=0;
}

// вычисление требуемой угловой скоости
void PREGULATOR(float alpha, float angleErr, float &angleRateDesired)
{
    float k;
    float amax=90*M_PI/180;     // желаемое ускорение (90 град/с^2)
    k=sqrt(2*amax);             // по формуле равноускоренного замедленного движения
    if (abs(angleErr)<10/57.3)   //если ошибка управления меньше 5 град. следует уменьшить коэффициент до нуля
      k=abs(angleErr)/10*57.3*k;
    // знак
    if (angleErr<0.0)
      k=-k;
    // требуемое значение угловой скорости
    angleRateDesired=k*sqrt(abs(angleErr));
}

// формирование вектора требуемых угл. скоростей
void FindAngleRateDesired(float angleErr[], float angleRated[])
{
    float alp_roll=0.5, alp_pitch=0.5, alp_yaw=0.5;
    float rollrate, pitchrate, yawrate;

    PREGULATOR(alp_yaw,angleErr[YAW], yawrate);
    PREGULATOR(alp_pitch,angleErr[PITCH], pitchrate);
    PREGULATOR(alp_roll,angleErr[ROLL], rollrate);

    // учет ограничения
    angleRated[YAW]=LMT(yawrate,180*M_PI/180);
    angleRated[PITCH]=LMT(pitchrate,180*M_PI/180);
    angleRated[ROLL]=LMT(rollrate,180*M_PI/180);
}


// Реализация алгоритмов цифровых ПИД регуляторов в общем виде
 void PID_DIGITAL(float q[],  float err[], float u[])
 {           
    float unew;
    // расчет нового значения корректирующего сигнала управления
    unew=q[3]*u[0]+q[4]*u[1]+q[0]*err[0]+q[1]*err[1]+q[2]*err[2];    
    // хранение для следующего вызова
    u[2]=u[1];    u[1]=u[0];    u[0]=unew; 
 }

// Расчет ШИМ сигнала по требуемому значению тяги (Throttle to PWM)
int T2PWM(float T, float THROLLTE0)
{
    float PWM;
    PWM=(-b+sqrt(b*b-4*a*(c-abs(T))))*0.5/a/2;    
    //PWM=-21*T*T+392*abs(T)+97;
    //PWM=-0.5*T*T+1.5*T;
     // учет знака силы тяги
    if (T<0)
    {
      PWM=-PWM;
    }
  
    // итоговый ШИМ сигнал (сумма программного и корректирующего)
    PWM=PWM+THROLLTE0;
    
    // ограничение (saturation)
    //min
    if (PWM<THROTTLE_MINIMUM)
    {
      PWM=THROTTLE_MINIMUM;
    }
    //max
    if (PWM>THROTTLE_MAXIMUM)
    {
      PWM=THROTTLE_MAXIMUM;
    }
    // возврашение результата
    return (int)PWM;
}


 // Программое управление
float PROGCONTROL(uint32_t tmr)
{  
	float thetaDesired=0.0;

/*
  uint32_t tmr1=tmr%20000;  
  if (tmr1>0)
  {
    thetaDesired=-30.0*M_PI/180;  
  }

  if (tmr1>10000)
  {
    thetaDesired=30.0*M_PI/180;  
  }
*/
	if (tmr<10000)
	{
		thetaDesired=0.0*M_PI/180;  
	}

	if (tmr>=15000)
	{
		thetaDesired=-15.0*M_PI/180;  
	}

	if (tmr>20000)
	{
		thetaDesired=15.0*M_PI/180;
	}

	if (tmr>25000)
	{
		thetaDesired=0.0;
	}

	return thetaDesired;
}

