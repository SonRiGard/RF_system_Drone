#ifndef _adaptive_h_
#define _adaptive_h_
#include <stdio.h>
//#include <tchar.h>
#include <stdlib.h>
#include <math.h>
//#include <clocale>

//float yr[16], ur[16], wr[16], an[16], bn[16], q[16], p[16], pm[16];
//int noh, nd, mp, mq;
//float rho;

float glw_(float y00, int noh, float an[], float bn[], float cgl);
float grenz_(float xn, float xmax, float xmin);
float stell_(float ynn, float yr[], float wn, float wr[], float ur[], float u00, float p[], float q[], int mp, int mq);
void msf1_(int init, int m, int nd, float rho, float ynn, float y[], float u[], float a[], float b[], float &cgl, float gm[]);
void dbm_(float an[], float bn[], int noh, int nd, float r, float p[], float q[], int &mp, int &mq);
void mvm_(float an[], float bn[], int noh, float r, float p[], float q[], int &mp, int &mq);
void initcntrl(float ur[], float yr[], float wr[],  float an[], float bn[], float p[], float q[],float &cgl,float pm[], int &mp, int &mq);
void adapcntrl(float wn, float ynn,float yr[], float wr[], float ur[],float &cgl, float &unn, float an[], float bn[],float p[], float q[],float pm[],int &mp, int &mq);
void IDENT(float ynn, float unn, float yr[], float ur[],float &cgl, float an[], float bn[],float pm[]);

#endif