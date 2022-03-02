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
	/**
	* Constructor
	* Creates TrigonometricOTG object
	*/
	TrigonometricOTG();
	/**
	* Constructor
	* Creates TrigonometricOTG object
	* @param num_dof number of degree of freedom
	* @param rate control rate
	*/
	TrigonometricOTG(int num_dof,double rate);
	/**
	* Constructor
	* Creates TrigonometricOTG object
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param rate control rate
	*/
	TrigonometricOTG(int num_dof, double mJ, double mA, double mV, double rate);
	/**
	* Constructor
	* Creates TrigonometricOTG object
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param rate control rate
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	TrigonometricOTG(int num_dof, vector<double> mJ, vector<double> mA, vector<double> mV, vector<double> a0, vector<double> v0, vector<double> p0, vector<double> pG, vector<double> alpha, double rate);
	/**
	* Constructor
	* Creates TrigonometricOTG object
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param rate control rate
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	TrigonometricOTG(int num_dof, double mJ, double mA, double mV, vector<double> a0, vector<double> v0, vector<double> p0, vector<double> pG, double alpha, double rate);
	/**
	* Destructor
	*/
	~TrigonometricOTG();

	/**
	* struct to store the computed time factors of trajectory
	*/
	struct Times{
		bool brake = false;
		double dist;
		double s;
		double sa;
		double sv;
		vector<double> twoPiece; // t11,t22,t33,t1,t2,ta1,ta2,ta3,tb1,tb2,t4,tc1,tc2,tc3;
		vector<double> onePiece; // ta1,ta2,ta3,tb1,tb2,t4,tc1,tc2,tc3;
	};
	/**
	* compute time factors of type I
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> typeI(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	/**
	* compute time factors of type II
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> typeII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	/**
	* compute time factors of type III
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> typeIII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	/**
	* compute time factors of type IV
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> typeIV(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	/**
	* compute time factors of type V
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> typeV(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	/**
	* compute time factors of type VI
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> typeVI(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	/**
	* compute time factors of type VII
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> typeVII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	/**
	* compute time factors of type VIII
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> typeVIII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	/**
	* Math to compute time factors of type I
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> computeTypeI(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	/**
	* Math to compute time factors of type II
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> computeTypeII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	/**
	* Math to compute time factors of type III
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> computeTypeIII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	/**
	* Math to compute time factors of type IV
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> computeTypeIV(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	/**
	* Math to compute time factors of type V
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> computeTypeV(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	/**
	* Math to compute time factors of type VI
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> computeTypeVI(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	/**
	* Math to compute time factors of type VII
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> computeTypeVII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	/**
	* Math to compute time factors of type VIII
	* @param num_dof number of degree of freedom
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<vector<double>> computeTypeVIII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv);
	/**
	* Check if the results are valid
	* @param candidate potential time factor solution
	*/
	vector<double> isValid(vector<double> candidate);
	/**
	* filter all potnetial time factor set and find the minimum time trajectpry time factors which satisfy all conditions
	* @param candidates all trjaectory time factor sets
	* @param Jpeak maximum jerk
	* @param Apeak maximum accerleration
	* @param Vpeak maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<double> filterResults(vector<vector<double>>& candidates, double a0, double v0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak);
	/**
	* Generate trajectory profile
	* @param candidates all trjaectory time factor sets
	* @param mJ maximum jerk
	* @param mA maximum accerleration
	* @param mV maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	* @param t current time
	* @param t11 t22 t33 t1 t2 t3 time factors
	* @param sa direction
	*/
	vector<double> profileGeneratorH(double a0, double v0, double p0, double alpha, double mJ, double mA, double mV, double t11, double t22, double t33, double t1, double t2, double sa, double t);
	/**
	* main function to compute time factors and profile
	* @param Jpeak maximum jerk
	* @param Apeak maximum accerleration
	* @param Vpeak maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	* @param duration total trajectory time
	*/
	vector<vector<double>> trajGeneratorT(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double duration);
	/**
	* generate trajectory profile in once
	*/
	vector<vector<vector<double>>> trajGenerator();
	/**
	* generate trajectory profile with current time
	*/
	vector<vector<double>> trajGeneratorS();
	/**
	* calculate minimum time of trajectory of all DoF
	*/
	double minimumTime();
	/**
	* main function to compute time factors
	* @param Jpeak maximum jerk
	* @param Apeak maximum accerleration
	* @param Vpeak maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<double> trajTimeT(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	/**
	* compute time factors of normal case
	* @param Jpeak maximum jerk
	* @param Apeak maximum accerleration
	* @param Vpeak maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<double> trajTimeN(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV);
	/**
	* compute time factors case but if the solution can not be found by normal case then output -1
	* @param Jpeak maximum jerk
	* @param Apeak maximum accerleration
	* @param Vpeak maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<double> trajTimeT1(double a0, double v0, double p0, double pG, double alpha, double maxJ, double maxA, double maxV);
	/**
	* compute time factors with given accelration and velocity to target velocity
	* @param Jpeak maximum jerk
	* @param Apeak maximum accerleration
	* @param Vpeak maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<double> trajTimeCall1(double a0, double v0, double p0, double vf, double alpha, double mJ, double mA, double mV,double s);
	/**
	* compute time factors to brake the initial acceleration and velocity to zero
	* @param Jpeak maximum jerk
	* @param Apeak maximum accerleration
	* @param Vpeak maximum velocity
	* @param a0 initial accelration
	* @param v0 initial velocity
	* @param p0 initial position
	* @pram pG target position
	* @param alpha time factor ratio
	*/
	vector<double> trajTimeCall2(double a0, double v0, double p0, double alpha, double mJ, double mA, double mV);
	int num_dof_; // degree of freedom
	double minT_; //minimal time
	bool complete_; // if trajecotry is complete
	vector<double> alpha_; // time ratio
	vector<double> mJ_; //max jerk
	vector<double> mA_; //max acceleration
	vector<double> mV_; //max velocity
	vector<double> a0_; //initial acceleration
	vector<double> v0_; //initial velocity
	vector<double> p0_; //initial position
	vector<double> pG_; //target position
	double rate_; // control loop rate (time interval)
	vector<Times> trajTimes_; // store all time factors for each DoF
	int idx_; // current profile index
};
