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
	double t1_,t2_,t11_,t22_,t33_;
	double mJ_, mA_, mV_;
	double alpha_;
	double v0_, a0_;
	double p0_;
	double sa_;
	vector<double> pre1_,pre2_,pre3_,pre4_,pre5_,pre6_,pre7_;
	profileSeg(double v0, double a0, double p0, double alpha, double t11, double t22, double t33, double t1, double t2, double sa, double maxJ, double maxA, double maxV);
	~profileSeg();
	vector<double> Seg1(double t);
	vector<double> Seg2( double t);
	vector<double> Seg3( double t);
	vector<double> Seg4( double t);
	vector<double> Seg5( double t);
	vector<double> Seg6( double t);
	vector<double> Seg7( double t);
	vector<double> Seg8( double t);
private:
	double Jpeak_,Apeak_, Vpeak_;
};
