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

class profileSegB
{
public:
	double t1_, t2_, t3_, t11_, t22_;
	double Jpeak_, Apeak_, Vpeak_;
	double alpha_;
	double v0_, a0_, mA_, mV_;
	double p0_, pG_;
	int sa_, sv_;
	profileSegB(double v0, double a0, double p0, double pG, double alpha, double t1, double t2, double t3, double t11, double t22, double maxJ, double maxA, double maxV);
	~profileSegB();
	vector<double> Seg1(double t);
	vector<double> Seg2(double t);
	vector<double> Seg3(double t);
	vector<double> Seg9(double t);
	vector<double> Seg10(double t);
	vector<double> Seg11(double t);
	vector<double> Seg12(double t);
	vector<double> Seg13(double t);
	vector<double> Seg14(double t);
	vector<double> Seg15(double t);
	vector<double> Seg16(double t);
};