#include "control.h"

// Вычисление коэффициентов цифровых ПИД регуляторов - первый вариант (без фильтрации D члена)

void DIGPIDCOEFF(float q[], float kP, float kI,float kD, float TAU)
{  
  float K=kP, tI=kI/kP, tD=kD/kP;  //поменял tI (исходная формула tI=kP/kI) --> не делить на 0 при kI=0;
  q[0]=K*(1+tD/TAU);
  q[1]=-K*(1+2*tD/TAU-TAU*tI);
  q[2]=K*tD/TAU;
  q[3]=1;
  q[4]=0;
}

void DIGPIDCOEFF1(float q[], float kPa, float kIa, float kDa, float TAU)
{  
  float kP,kI,kD,d;
  int N=10;
  kP=kPa;
  kI=TAU*kIa;
  kD=N*kDa/(kDa+N*TAU);
  d=kD/N;
  // a1=1+d;
  // a2=-d;
  // b0=kP+kI+kD;
  // b1=-kP*(1+d)-kI*d-2*kD;
  // b2=kP*d+kD;  
  q[0]=kP+kI+kD;
  q[1]=-kP*(1+d)-kI*d-2*kD;
  q[2]=kP*d+kD;
  q[3]=1+d;
  q[4]=-d;
}

// расчет коэффициентов альфа для адаптивного ПИД
void SetAlpha(float Ts, float xi, float tau,float alp[])
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
    alp[2]=p0*p2;
}

// расчет коэффициентов адаптивного ПИД
void ADPIDCOEFF(float alp[], float a[], float b[], float q[])
{
    float b1;
    b1=b[0];
    if (-1.E-10<b1 && b1<0.0)
        b1=-1.E-10;
    if (0.0<b1 && b1<1.E-10)
        b1=1.E-10;
    
    q[0]=(alp[0]-a[1]+1)/b1;
    q[1]=(alp[1]-a[2]+a[1]-q[0]*b[1])/b1;
    q[2]=(alp[2]+a[2]-q[1]*b[1])/b1;
    q[3]=1;
    q[4]=0;
}

// вычисление требуемой угловой скоости
void PREGULATOR(float alpha, float angleErr, float &angleRateDesired, float k)
{
 /* 
    float k;
    float amax=90*M_PI/180;     // желаемое ускорение (90 град/с^2)
    k=sqrt(2*amax);             // по формуле равноускоренного замедленного движения
    if (fabs(angleErr)<10/57.3){   //если ошибка управления меньше 5 град. следует уменьшить коэффициент до нуля
      k=fabs(angleErr)/10*57.3*k;
    }

    if (angleErr<0.0){
      k=-k;
    }
    // требуемое значение угловой скорости
    //angleRateDesired=k*sqrt(fabs(angleErr));    
    //roll
    angleRateDesired=2.94*angleErr;
    //pitch
    angleRateDesired=5.1*angleErr;

    //nonlinear
    //angleRateDesired=2.6*angleErr+0.1*pow(angleErr,3.0);    
  */
   angleRateDesired=k*angleErr;
}

// формирование вектора требуемых угл. скоростей
void FindAngleRateDesired(float angleErr[], float angleRated[])
{
    float alp_roll=0.5, alp_pitch=0.5, alp_yaw=0.5;
    float rollrate, pitchrate, yawrate;

    PREGULATOR(alp_yaw,angleErr[YAW], yawrate,1.59);
    PREGULATOR(alp_pitch,angleErr[PITCH], pitchrate,1.8);
    PREGULATOR(alp_roll,angleErr[ROLL], rollrate,2.94);

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
int T2PWM(float T, int PWM0, int THROTTLE_MIN, int THROTTLE_MAX, float p[])
{
    float T0,TSUM,PWM;
    // сила тяги при THROTTLE0
    T0=p[0]*(PWM0-1000)*(PWM0-1000)+p[1]*(PWM0-1000)+p[2];
    // суммарная сила тяги
    TSUM=T+T0;
    if (T<0)
    {
      T=0;
    }
    // PWM соответствующий суммерной тяге
    PWM=(-p[1]+sqrt(p[1]*p[1]-4*p[0]*(p[2]-TSUM)))*0.5/p[0]+1000; 
    
    // ограничение (saturation)
    //min
    if (PWM<THROTTLE_MIN)
    {
      PWM=THROTTLE_MIN;
    }
    //max
    if (PWM>THROTTLE_MAX)
    {
      PWM=THROTTLE_MAX;
    }
    // возврашение результата
    return (int)PWM;
}


 // Программое управление
float PROGCONTROL(uint32_t tmr)
{  
	float thetaDesired=0.0;

  uint32_t tmr1=tmr%10000;  
  if (tmr1<5000)
	{
		thetaDesired=-30.0*M_PI/180;;  
	}
  else
  {
    thetaDesired=30.0*M_PI/180;  
  }

	return thetaDesired;
}

