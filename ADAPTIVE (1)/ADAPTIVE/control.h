#ifndef _CONTROL_H_
#define _CONTROL_H
#include <stdio.h>
//#include <stdlib.h>
//#include <math.h>

#include "simplemath.h"


// --- Индексы  для углов ----
# define YAW         0
# define PITCH       1
# define ROLL        2

void DIGPIDCOEFF(float q[], float kP, float kI,float kD, float TAU);
void DIGPIDCOEFF1(float q[], float kPa, float kIa, float kDa, float TAU);
void SetAlpha(float Ts, float xi,float tau,float alp[]);
void ADPIDCOEFF(float alp[], float a[], float b[], float q[]);
void PREGULATOR(float alpha, float angleErr, float &angleRateDesired);
void FindAngleRateDesired(float angleErr[], float angleRated[]);
void PID_DIGITAL(float q[],  float err[], float u[]);
float PROGCONTROL(uint32_t tmr);
int T2PWM(float T, int THROLLTE0, int THROTTLE_MIN, int THROTTLE_MAX, float p[]);
#endif