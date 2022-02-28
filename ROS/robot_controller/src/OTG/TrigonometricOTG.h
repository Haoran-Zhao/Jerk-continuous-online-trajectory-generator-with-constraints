#pragma once
#include<iostream>
#include<vector>
#include<string>
#include <cmath>
#include<math.h>
#include<algorithm>
#include <complex>
#include <numeric>
#include<profileSegments.h>
#include <unsupported/Eigen/Polynomials>

using namespace std;

class TrigonometricOTG {
public:
	TrigonometricOTG();
	TrigonometricOTG(int num_dof,double rate);
	TrigonometricOTG(int num_dof, double mJ, double mA, double mV, double rate);
	TrigonometricOTG(int num_dof, vector<double> mJ, vector<double> mA, vector<double> mV, vector<double> a0, vector<double> v0, vector<double> p0, vector<double> pG, vector<double> alpha, double rate);
	TrigonometricOTG(int num_dof, double mJ, double mA, double mV, vector<double> a0, vector<double> v0, vector<double> p0, vector<double> pG, double alpha, double rate);

	~TrigonometricOTG();

	struct Times{
		bool brake = false;
		double dist;
		double s;
		double sa;
		double sv;
		vector<double> twoPiece; // t11,t22,t33,t1,t2,ta1,ta2,ta3,tb1,tb2,t4,tc1,tc2,tc3;
		vector<double> onePiece; // ta1,ta2,ta3,tb1,tb2,t4,tc1,tc2,tc3;
	};

	vector<vector<double>> typeI(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	vector<vector<double>> typeII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	vector<vector<double>> typeIII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	vector<vector<double>> typeIV(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	vector<vector<double>> typeV(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	vector<vector<double>> typeVI(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	vector<vector<double>> typeVII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	vector<vector<double>> typeVIII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);

	vector<vector<double>> computeTypeI(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	vector<vector<double>> computeTypeII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	vector<vector<double>> computeTypeIII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	vector<vector<double>> computeTypeIV(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	vector<vector<double>> computeTypeV(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	vector<vector<double>> computeTypeVI(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	vector<vector<double>> computeTypeVII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	vector<vector<double>> computeTypeVIII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);

	vector<double> isValid(vector<double> candidate);
	vector<double> filterResults(vector<vector<double>>& candidates, double a0, double v0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	vector<double> profileGeneratorH(double a0, double v0, double p0, double alpha, double mJ, double mA, double mV, double t11, double t22, double t33, double t1, double t2, double sa, double t);
	vector<vector<double>> trajGeneratorT(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double duration);
	vector<vector<vector<double>>> trajGenerator();
	vector<vector<double>> trajGeneratorS();
	double minimumTime();
	vector<double> trajTimeT(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	vector<double> trajTimeN(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	vector<double> trajTimeT1(double a0, double v0, double p0, double pG, double alpha, double maxJ, double maxA, double maxV);
	vector<double> trajTimeCall1(double a0, double v0, double p0, double vf, double alpha, double mJ, double mA, double mV,double s);
	vector<double> trajTimeCall2(double a0, double v0, double p0, double alpha, double mJ, double mA, double mV);
	int num_dof_; // degree of freedom
	double minT_; //minimal time
	bool complete_;
	vector<double> alpha_; // time ratio
	vector<double> mJ_; //max jerk
	vector<double> mA_; //max acceleration
	vector<double> mV_; //max velocity
	vector<double> a0_; //initial acceleration
	vector<double> v0_; //initial velocity
	vector<double> p0_; //initial position
	vector<double> pG_; //target position
	double rate_; // control loop rate (time interval)
	vector<Times> trajTimes_;
	int idx_;
};
