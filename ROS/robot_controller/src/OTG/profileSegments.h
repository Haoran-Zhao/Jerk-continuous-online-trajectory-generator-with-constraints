#pragma once
#include<iostream>
#include<vector>
#include<string>
#include <cmath>
#include<math.h>
#include<algorithm>
#include <complex>
#include <numeric>
using namespace std;

class profileSeg
{
public:
	double t1_,t2_,t3_,t4_,t11_,t22_,t33_;
	double Jpeak_, Apeak_, Vpeak_;
	double alpha_;
	double v0_, a0_;
	double p0_, pG_;
	profileSeg(double v0, double a0, double p0, double pG, double alpha, double t1, double t2, double t3, double t4, double t11, double t22, double t33, double maxJ, double maxA, double maxV);
	~profileSeg();
	vector<double> Seg1(double t);
	vector<double> Seg2( double t);
	vector<double> Seg3( double t);
	vector<double> Seg4( double t);
	vector<double> Seg5( double t);
	vector<double> Seg6( double t);
	vector<double> Seg7( double t);
	vector<double> Seg8( double t);
	vector<double> Seg9( double t);
	vector<double> Seg10( double t);
	vector<double> Seg11( double t);
	vector<double> Seg12( double t);
	vector<double> Seg13( double t);
	vector<double> Seg14( double t);
	vector<double> Seg15( double t);
	vector<double> Seg16(double t);
private:
	double mA_, mV_;
};