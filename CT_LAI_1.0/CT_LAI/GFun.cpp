#include "GFun.h"

double gfun( double u1, double a, double b, double c )
{

	double theta = acos(u1);
	double g = (a + b*cos(2 * theta)+ c * cos(4 * theta)) / sin(theta);
	return g;
}

//根据测量值计算
double gfun(double u1, double gd[], const int _n_bins)	  //new: 2017-12-14 rhhu n_bins
{
	int n = int(acos(u1) / M_PI * 180 / (90.0 / _n_bins));

	double g = gd[n] / M_PI*(2 * _n_bins) / sqrt(1 - u1*u1);
	return g;
}


double AFun( double u1, double up )
{
	double A = 0;
	if ( (acos(u1)+ acos(up)) <= M_PI/2)
	{
		A = u1*up;
		return A;
	}
	else
	{
		A = 2/M_PI*( u1 * up * asin(u1 * up / sqrt((1-u1*u1)*(1-up*up)) ) + sqrt(1-up*up-u1*u1));
		return A;
	}
}

double GFun( double up )
{
	double interval = 1.0 /10000.0;
	double u = interval/2;
	double result = 0;

	while (u <  1.0)
	{
		result += gfun(u, sqrt(1-u*u), 0, 0) * AFun(u, up);
		u += interval;
	}
	result *= interval;
	return result;
}


double GFun( double up ,double _a, double _b, double _c )
{
	//double interval = 1.0 /100000.0;
	//double u = interval/2;
	//double result = 0;

	//
	//while (u <  1)
	//{
	//	result += gfun(u, _a, _b, _c) * AFun(u, up);
	//	u += interval;
	//}
	//result *= interval;
	//return result;


	//Simpson's rule
	double panels = 10000.0;
	double interval = 1.0 /panels;

	double result = 0;

	double uMin = 0.0;
	double uMax = 1.0 - interval/14.2;
	double u = uMin*2;


	for(int i=0;i<panels;i++)
	{
		if(i%2 == 0)
		{
			result += 2*gfun(u, _a, _b, _c) * AFun(u, up);
		}
		else
		{
			result += 4*gfun(u, _a, _b, _c) * AFun(u, up);
		}
		u += interval;
	}
	result += gfun(uMin, _a, _b, _c) * AFun(uMin, up);
	result += gfun(uMax, _a, _b, _c) * AFun(uMax, up);
	result *= interval/3;
	return result;
}

double GFun(double up, double sigma, double v)
{
	double interval = 1.0 / 10000.0;
	double u = interval / 2;
	double result = 0;

	while (u <  1.0)
	{
		//result += gfun(u, sqrt(1-u*u), 0, 0) * AFun(u, up);
		result += gfun(u, sigma, v) * AFun(u, up);
		u += interval;
	}
	result *= interval;
	return result;
}

double gfun(double u1, double sigma, double v)
{
	double GamaMuNu, GamaMu, GamaNu;
	double theta = acos(u1);
	double x = 2 / M_PI*theta;
	GamaMuNu = gama(sigma + v);
	GamaMu = gama(sigma);
	GamaNu = gama(v);
	double g = (2.0 / M_PI)*(GamaMuNu / GamaMu / GamaNu)*pow(x, sigma - 1)*pow(1 - x, v - 1) / sqrt(1 - u1*u1);
	return g;
}


double gama(double x)
{
	double y, result;
	y = 1 / (12.0*x) - 1 / (360.0*x*x*x) - x;
	result = sqrt((2 * M_PI) / x)*pow(x, x)*exp(y);
	return result;
}

//对应测量数据   
double GFun(double up, double gd[], const int _n_bins)		//new: 2017-12-14 rhhu n_bins
{
	double interval = 1.0 / 100000.0;
	double u = interval / 2;
	double result = 0;

	while (u <  1.0)
	{
		result += gfun(u, gd, _n_bins) * AFun(u, up);
		u += interval;
	}
	result *= interval;
	return result;
}

int ReadMes( wchar_t *fMes ,double *tmpg, const int _n_bins) //new: 2017-12-14 rhhu n_bins
{
	double _sum = 0;
	FILE* fpg = NULL;
	errno_t err; 
	if (err = _wfopen_s(&fpg, fMes,L"r"))
	{
		return err;
	}

	for (int i = 0; i< _n_bins; i++)
	{
		fscanf_s(fpg,"%*d %lf\n",&tmpg[i]);
		_sum += tmpg[i];
	}
	fclose(fpg);
	fpg =0;

	for (int i = 0; i< _n_bins; i++)		//new: 2017-12-14 rhhu normalize
	{
		tmpg[i] /= _sum;
	}

	return 0;
}


double GMean(double * G, double _zenith_min, double _zenith_max, double _angle_interval)
{
	
	size_t i_min = round(_zenith_min / _angle_interval);
	size_t i_max = round(_zenith_max / _angle_interval);
	double sum = 0;
	for (size_t i = i_min; i <= i_max; i++)
	{
		sum += G[i];
	}
	return sum / (i_max - i_min + 1);
}
