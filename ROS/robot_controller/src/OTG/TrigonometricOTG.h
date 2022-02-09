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
#include<profileSegmentsB.h>
#include<poly34.h>

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
		vector<double> twoPiece; // t1b,t2b,t3b,t11b,t22b, t1,t2,t3,t4;
		vector<double> onePiece; // t1,t2,t3,t4,t11,t22,t33;
	};

	double typeI(double v0, double a0, double p0, double pG, double alpha, double t1, double t2 ,double t3, double t11, double t22, double t33, double Jpeak, double Apeak, double Vpeak);
	vector<double> typeII(double v0, double a0, double p0, double pG, double alpha, double t1, double t2, double t11, double t22, double Jpeak, double Apeak, double Vpeak);
	vector<double> typeIII(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	vector<double> typeIV(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	vector<double> typeV(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	vector<double> typeVI(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	vector<double> typeVII(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	vector<double> typeVIII(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	vector<double> brakeCalculate(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	vector<double> isValid(vector<double> candidate);
	vector<double> filterResults(vector<vector<double>>& candidates, double v0, double a0, double p0, double pG, double Jpeak, double Apeak, double Vpeak);
	vector<double> profileGenerator(double v0, double a0, double p0, double pG, double alpha, double t1, double t2, double t3, double t4, double t11, double t22, double t33, double Jpeak, double Apeak, double Vpeak,double t);
	vector<double> profileGeneratorB(double v0, double a0, double p0, double pG, double alpha, double t1, double t2, double t3, double t11, double t22, double Jpeak, double Apeak, double Vpeak,double t);
	vector<vector<double>> trajGeneratorB(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	vector<vector<double>> trajGeneratorT(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak, double duration);
	vector<vector<double>> trajGeneratorZ(double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak, double t, double pret, double duration);
	vector<vector<vector<double>>> trajGenerator();
	vector<vector<double>> trajGeneratorS();
	double minimumTime();
	vector<double> trajTimeB(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	vector<double> trajTimeT(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	vector<double> trajTimeZ(double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);

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
