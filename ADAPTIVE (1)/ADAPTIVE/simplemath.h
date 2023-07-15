#ifndef _SIMPLEMATH_
#define _SIMPLEMATH_
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

float LMT(float x, float thrsd);
float DEADZONE(float x, float thrsd);
void vectorstore(float x[], float xnew);
float SATURATION(float x, float xmax, float xmin);
float DigLowPassFil(float x_prev, float u, float u_prev, float DT, float w0);

#endif