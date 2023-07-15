#ifndef _CONTROL_H_
#define _CONTROL_H_
//#include <stdio.h>
////#include <tchar.h>
//#include <stdlib.h>
//#include <math.h>
////#include <clocale>

void DIGPIDCOEFF(float q[], const float kP, const float kI,const float kD, const float TAU);
void SetAlpha(const float Ts, const float xi,const float tau,float alp[]);
void ADPIDCOEFF(float alp[], float a[], float b[], float q[]);
void PREGULATOR(float alpha, float angleErr, float &angleRateDesired);
void FindAngleRateDesired(float angleErr[], float angleRated[]);
void PID_DIGITAL(float q[],  float err[], float u[]);
float PROGCONTROL(uint32_t tmr);
int T2PWM(float T, float THROLLTE0);
#endif