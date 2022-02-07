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
	~TrigonometricOTG();
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
	vector<vector<double>> trajGeneratorT(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	vector<vector<double>> trajGeneratorZ(double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak, double t, double pret);

	vector<double> trajTimeB(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	vector<double> trajTimeT(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	vector<double> trajTimeZ(double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);

	int num_dof_; // degree of freedom
	vector<double> mJ_; //max jerk
	vector<double> mA_; //max acceleration
	vector<double> mV_; //max velocity
	double rate_; // control loop rate (time interval)

};