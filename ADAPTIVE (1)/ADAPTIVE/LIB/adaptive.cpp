#include "adaptive.h"

float glw_(float y00, int noh, float an[], float bn[], float cgl)
{
	double suma = 1.f;
	double sumb = 0.f;
	int i;
	for (i = 0; i < noh; ++i)
	{
		suma += an[i + 1];
		sumb += bn[i];
	}
	if (sumb == 0.f)
		sumb = 1e-6f;
	return (suma * y00 - cgl) / sumb;
}

float grenz_(float xn, float xmax, float xmin)
{
	float res = xn;
	if (xmax < xmin)
		res = 0.0f;
	if (xn > xmax)
		res = xmax;
	if (xn < xmin)
		res = xmin;
	return res;
}

float stell_(float ynn, float yr[], float wn, float wr[], float ur[], float u00,
	 float p[], float q[], int mp, int mq)
{
	float res;
	int i;

	res = q[0] * (wn - ynn);

	for (i = 0; i < mq - 1; ++i)
	{
		res += q[i+1] * (wr[i] - yr[i]);
	}
	for (i = 0; i < mp - 1; ++i)
	{
		res -= p[i+1] * (ur[i] - u00);
	}
	res = u00 + res / p[0];
	return res;
}

#define INU(i,j)	((int)(((i)*((i)+1))/2 + (j)))
void msf1_(int init, int m, int nd, float rho, float ynn,
	float y[], float u[], float a[], float b[], float &cgl, float gm[])
{
	int i, j, l;
	float teta[17];
	float gv[17];
	float psi[17];
	float hl, sig, sig1, gmlj, epsn, phim1, phiss;

	int n = m * 2;
	switch (init)
	{
	case 3:
		++n;
	case 1:
		for (i = 0; i < n; ++i)
		{
			for (j = 0; j < i; ++j)
			{
				gm[INU(i,j)] = 0.0;
			}
			gm[INU(i,i)] = 1e5;
		}
		return 0;
	case 4:
		++n;
		teta[n - 1] = cgl;
		psi[n - 1] = 1.0;
	case 2:
		break;
	default:
		return;
	}

	for (i = 0; i < m; ++i)
	{
		teta[2*i] = a[i];
		teta[2*i + 1] = b[i];
		psi[2*i] = -y[i];
		psi[2*i + 1] = u[i + nd];
	}

	phim1 = 1.0 / pow(rho,0.5);
	sig = rho;

	for (l = 0; l < n; ++l)
	{
		hl = 0.0;
		for (i = 0; i <= l; ++i)
		{
			hl += gm[INU(l,i)] * psi[i];
		}
		sig1 = sig;
		sig += hl * hl;
		phiss = phim1 * sqrt(sig1 / sig);
		gv[l] = hl * gm[INU(l,l)];
		gm[INU(l,l)] = phiss * gm[INU(l,l)];

		for (j = 0; j < l; ++j)
		{
			gmlj = gm[INU(l,j)];
			gm[INU(l,j)] = phiss * (gmlj - hl * gv[j] / sig1);
			gv[j] += hl * gmlj;
		}
	}

	epsn = ynn;
	for (i = 0; i < n; ++i)
	{
		epsn -= psi[i] * teta[i];
	}
	epsn /= sig;
	for (i = 0; i < n; ++i)
	{
		teta[i] += epsn * gv[i];
	}
	for (i = 0; i < m; ++i)
	{
		a[i] = teta[2*i];
		b[i] = teta[2*i + 1];
	}
	cgl = teta[n - 1];
}
#undef INU

void dbm_(float an[], float bn[], int noh, int nd, float r, float p[], float q[], int &mp, int &mq)
{
	float qmin, qmax;
	int i;

	for (i = 0; i < noh + nd + 2; ++i)
	{
		p[i] = 0.0;
		q[i] = 0.0;
	}
	p[0] = 1.0;
	qmax = 0.0;
	for (i = 0; i < noh; ++i)
	{
		qmax += bn[i];
	}
	if (qmax == 0.0)
		qmax = 1.2e-30;
	qmax = 1.0 / qmax;
	qmin = qmax / (1.0 - an[1]);
	q[0] = qmin + r * (qmax - qmin);
	q[1] = q[0] * (an[1] - 1) + qmax;
	p[nd + 1] = -q[0] * bn[0];
	for (i = 1; i < noh; ++i)
	{
		q[i + 1] = q[0] * (an[i + 1] - an[i]) + an[i] * qmax;
		p[i + 1 + nd] = -q[0] * (bn[i] - bn[i - 1]) - bn[i - 1] * qmax;
	}
	q[noh + 1] = an[noh] * (qmax - q[0]);
	p[noh + nd + 1] = -bn[noh-1] * (qmax - q[0]);
	mq = noh + 2;
	mp = noh + nd + 2;
}

void mvm_(float an[], float bn[], int noh, float r, float p[], float q[], int &mp, int &mq)
{
	int i;
	double fak;
	if (fabs(bn[0]) < 1.2E-10)
		bn[0] = 1.2E-10;
//  r=bn[0]*bn[0];	//добавление 13.12.2022
	fak = bn[0] + r / bn[0];

	for (i = 0; i < noh; ++i)
		{
		q[i] = -an[i + 1] / fak;
		p[i] = bn[i] / fak;
		}
	p[0] = 1.0;
	mp = noh;
	mq = noh;
}

void vers_(float xn, float xr[], int n)
{
	int i;
	for (i = n-1; i > 0; --i)
	{
		xr[i] = xr[i-1];
	}
	xr[0] = xn;
}

//void initcntrl(int noh, int nd, float rho, float u00, float y00, float ur[], float yr[], float wr[],  float an[], float bn[], float p[], float q[],float &cgl,float pm[], int &mp, int &mq)
void initcntrl(float ur[], float yr[], float wr[],  float an[], float bn[], float p[], float q[],float &cgl,float pm[], int &mp, int &mq)
{	
	float u00=0, y00=0,unn=0.0, ynn=0.0,wn=0.0;
	int iz=3;
	float r=0.0;
	const float rho = 0.999;
	int noh=2;
	int nd=0;
	for (int i = 0; i < 4; i++)
	{
		an[i+1] = 0.0;
		bn[i] = 0.0;
		p[i] = 0.0;
		q[i] = 0.0;
		ur[i] = u00;
		yr[i] = y00;
		wr[i] = y00;
	}
  
	an[0] = 1.0;
	bn[0] = 0.01;
	p[0] = 1.0;

	for (int i = 0; i < noh; i++)
	{
		an[i] = an[i+1];
	}

	msf1_(iz, noh, nd, rho, ynn, yr, ur, an, bn, cgl, pm);

	for (int i = noh; i > 0; i--)
	{
		an[i] = an[i-1];
	}
	an[0] = 1.0;
	
	mvm_(an, bn, noh, r, p, q, mp, mq);
//	dbm_(an, bn, noh, nd, r, p, q, mp, mq);

	vers_(ynn, yr, 4);
	vers_(unn, ur, 4);
	vers_(wn, wr, 4);	
}

void adapcntrl(float wn, float ynn,float yr[], float wr[], float ur[],float &cgl, float &unn, float an[], float bn[],float p[], float q[],float pm[],int &mp, int &mq)
//void adapcntrl(float wn, float ynn,float &unn)
{
//	static float yr[5]={0.0},wr[5]={0.0},ur[5]={0.0},cgl=0.0,an[5]={0.0}, bn[5]={0.0},p[5]={0.0},q[5]={0.0},pm[78]={0.0};
	float y00 = 0,	u00 = 0;
	float r=0.0;
	int iz = 4;
	int noh=2;
	int nd=0;
	const float rho = 0.999;
	const float umax = 0.1;
	const float umin = -0.1;

			
	y00 = wn;
	u00 = glw_(y00, noh, an, bn, cgl);
	unn = stell_(ynn, yr, wn, wr, ur, u00, p, q, mp, mq);
	unn = grenz_(unn, umax, umin);

	for (int i = 0; i < noh; i++)
	{
		an[i] = an[i+1];
	}

	msf1_(iz, noh, nd, rho, ynn, yr, ur, an, bn, cgl, pm);

	for (int i = noh; i > 0; i--)
	{
		an[i] = an[i-1];
	}
	an[0] = 1.0;

	mvm_(an, bn, noh, r, p, q, mp, mq);
//	dbm_(an, bn, noh, nd, r, p, q, mp, mq);

	vers_(ynn, yr, 4);
	vers_(unn, ur, 4);
	vers_(wn, wr, 4);
}

void IDENT(float ynn, float unn, float yr[], float ur[],float &cgl, float an[], float bn[],float pm[])
//void adapcntrl(float wn, float ynn,float &unn)
{
//	static float yr[5]={0.0},wr[5]={0.0},ur[5]={0.0},cgl=0.0,an[5]={0.0}, bn[5]={0.0},p[5]={0.0},q[5]={0.0},pm[78]={0.0};
	float y00 = 0,	u00 = 0;
	float r=0.0;
	int iz = 4;
	int noh=2;
	int nd=0;
	const float rho = 0.999;

	for (int i = 0; i < noh; i++)
	{
		an[i] = an[i+1];
	}

	msf1_(iz, noh, nd, rho, ynn, yr, ur, an, bn, cgl, pm);

	for (int i = noh; i > 0; i--)
	{
		an[i] = an[i-1];
	}
	an[0] = 1.0;

	vers_(ynn, yr, 4);
	vers_(unn, ur, 4);
}