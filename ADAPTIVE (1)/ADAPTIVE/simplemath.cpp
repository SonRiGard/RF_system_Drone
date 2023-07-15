#include "simplemath.h"

// Ограничение по thresold
float LMT(float x, float thrsd)
{
  if (fabs(x)>thrsd)
  {
    x=thrsd*fabs(x)/x;
  }
  return x;
}

// Зона нечувствительность
float DEADZONE(float x, float thrsd)
{
  if (fabs(x)<thrsd)
  {
    x=0.0;
  }
  return x;
}

// хранение данных в вектор
void vectorstore(float x[], float xnew)
{
    x[2]=x[1];
    x[1]=x[0];
    x[0]=xnew; 
}

// Ограничение по min max
float SATURATION(float x, float xmax, float xmin)
{  
  if (x>xmax){
    x=xmax; 
  }

  if (x<xmin){
    x= xmin; 
  }
  return x;
}

// Низкочастотный фильтр
float DigLowPassFil(float x_prev, float u, float u_prev, float DT, float w0)
{
  //float DT=0.01,w0=30;
  float a0=DT*w0+2, a1=DT*w0-2,b=DT*w0;
  return (-a1*x_prev+b*(u+u_prev))/a0;
}
