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
	double t1_,t2_,t11_,t22_,t33_; //time factors
	double mJ_, mA_, mV_; // maximum jerk, accelration, and velocity
	double alpha_; // time factor ratio
	double v0_, a0_;//initial velocity and accelration
	double p0_;//initial position
	double sa_;//trajectpry direction
	vector<double> pre1_,pre2_,pre3_,pre4_,pre5_,pre6_,pre7_; //pre trajecotry segments
	/**
	* Constructor
	* construct profileSeg object
	* @param maxJ maximum jerk
	* @param maxA maximum accerleration
	* @param maxV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram sa trajecotry direction
	* @param alpha time factor ratio
	* @param t11 t22 t33 t1 t2 time factors
	*/
	profileSeg(double v0, double a0, double p0, double alpha, double t11, double t22, double t33, double t1, double t2, double sa, double maxJ, double maxA, double maxV);
	/**
	* Destructor
	*/
	~profileSeg();
	/**
	* compute profile of segment 1
	* @param t current time
	*/
	vector<double> Seg1(double t);
	/**
	* compute profile of segment 2
	* @param t current time
	*/
	vector<double> Seg2( double t);
	/**
	* compute profile of segment 3
	* @param t current time
	*/
	vector<double> Seg3( double t);
	/**
	* compute profile of segment 4
	* @param t current time
	*/
	vector<double> Seg4( double t);
	/**
	* compute profile of segment 5
	* @param t current time
	*/
	vector<double> Seg5( double t);
	/**
	* compute profile of segment 6
	* @param t current time
	*/
	vector<double> Seg6( double t);
	/**
	* compute profile of segment 7
	* @param t current time
	*/
	vector<double> Seg7( double t);
	/**
	* compute profile of segment 8
	* @param t current time
	*/
	vector<double> Seg8( double t);
private:
	double Jpeak_,Apeak_, Vpeak_;// peak jerk, accelration, and velocity
};
