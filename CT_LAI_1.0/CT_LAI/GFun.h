#pragma once
// G函数计算程序 by 胡容海 2011.11.30
// 读取测量数据	new: 2017-12-14 rhhu n_bins


#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif // !1



#include <cstdio>
#include <cmath>
using namespace std;




double gfun(double theta, double a, double b, double c);	//三角形分布表示g函数
double gfun(double u1, double sigma, double v);			//Beta分布表示g函数
double gfun(double u1, double gd[], const int _n_bins);	  //根据测量值计算   new: 2017-12-14 rhhu n_bins

double AFun(double u1, double up);

double GFun(double up);									//三角函数球形分布
double GFun(double up, double _a, double _b, double _c);	//根据三角形函数分布的三个参数计算G
double GFun(double up, double sigma, double v);			//根据Beta分布的两个参数计算G
double GFun(double up, double gd[], const int _n_bins);	//根据测量值计算	new: 2017-12-14 rhhu n_bins
double GMean(double * G, double _zenith_min, double _zenith_max, double _angle_interval);	//根据测量值计算	new: 2017-12-14 rhhu n_bins
int ReadMes(wchar_t *fMes, double *tmpg, const int _n_bins);	//读取测量数据	new: 2017-12-14 rhhu n_bins

double gama(double x);

