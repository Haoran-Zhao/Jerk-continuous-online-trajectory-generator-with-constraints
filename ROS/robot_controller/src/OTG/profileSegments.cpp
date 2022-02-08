#include<profileSegments.h>
# define Pi           3.14159265358979323846

profileSeg::profileSeg(double v0, double a0, double p0, double pG, double alpha, double t1, double t2, double t3, double t4, double t11, double t22, double t33, double maxJ, double maxA, double maxV) :v0_(v0), a0_(a0), p0_(p0), pG_(pG), alpha_(alpha), t1_(t1), t2_(t2), t3_(t3), t4_(t4), t11_(t11), t22_(t22), t33_(t33), Jpeak_(maxJ), Apeak_(maxA), Vpeak_(maxV) {};
profileSeg::~profileSeg() {};
vector<double> profileSeg::Seg1(double t) {
	double jerk, accel, vel, pos;
	if (t11_ == 0)
	{
		jerk = 0;
		accel = a0_;
		vel = v0_ + a0_ * t;
		pos = p0_ + a0_ * pow(t, 2) / 2 + v0_ * t;
	}
	else
	{
		jerk = Jpeak_ * sin(Pi*t/(2*t11_));
		accel = a0_ + (2 * Jpeak_ * t11_) / Pi - (2 * Jpeak_ * t11_ * cos((Pi * t) / (2. * t11_))) / Pi;
		vel = a0_ * t + (2 * Jpeak_ * t * t11_) / Pi + v0_ - (4 * Jpeak_ * pow(t11_, 2) * sin((Pi * t) / (2. * t11_))) / pow(Pi, 2);
		pos = p0_ + (a0_ * pow(t, 2)) / 2 + (Jpeak_ * pow(t, 2) * t11_) / Pi - (8 * Jpeak_ * pow(t11_, 3)) / pow(Pi, 3) +
			t * v0_ + (8 * Jpeak_ * pow(t11_, 3) * cos((Pi * t) / (2 * t11_))) / pow(Pi, 3);
	}
	return {jerk,accel,vel,pos};
};
vector<double> profileSeg::Seg2( double t) {
	double jerk, accel, vel, pos;
	pre1_ = pre1_.empty()? Seg1(t11_):pre1_;
	if (t11_ == 0)
	{
		jerk = Jpeak_;
		accel = pre1_[1]+Jpeak_*t;
		vel = pre1_[2]+a0_*t+Jpeak_*pow(t,2)/2;
		pos = pre1_[3]+ (Jpeak_ * pow(t, 3)) / 6. + a0_ * t * t11_ + (a0_ * pow(t, 2)) / 2. + t * v0_;
	}
	else
	{
		jerk = Jpeak_;
		accel = pre1_[1] + Jpeak_ * t;
		vel = pre1_[2] + a0_ * t + Jpeak_ * pow(t, 2) / 2+(2*Jpeak_*t*t11_)/Pi;
		pos =pre1_[3]+ (a0_ * pow(t, 2)) / 2. + (Jpeak_ * pow(t, 3)) / 6. + a0_ * t * t11_ + (Jpeak_ * pow(t, 2) * t11_) / Pi - (4 * Jpeak_ * t * pow(t11_, 2)) / pow(Pi, 2) +
			(2 * Jpeak_ * t * pow(t11_, 2)) / Pi + t * v0_;
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg3( double t)
{
	double jerk, accel, vel, pos;
	pre2_ = pre2_.empty()? Seg2(t22_): pre2_;
	if (t11_ == 0)
	{
		jerk = 0;
		accel = pre2_[1];
		vel = pre2_[2] + a0_ * t + Jpeak_ * t* t22_;
		pos = pre2_[3] + (a0_ * pow(t, 2)) / 2. + a0_ * t * t1_ + a0_ * t * t22_ + (Jpeak_ * pow(t, 2) * t22_) / 2. + (Jpeak_ * t * pow(t22_, 2)) / 2. + t * v0_;
	}
	else
	{
		jerk = Jpeak_ * sin((Pi * (1 + t / t11_)) / 2.);
		accel = pre2_[1] + (2 * Jpeak_ * t11_ * sin((Pi * t) / (2. * t11_))) / Pi;
		vel = pre2_[2] + a0_ * t + (2 * Jpeak_ * t11_ * (Pi * t + 2 * t11_)) / pow(Pi, 2) + Jpeak_ * t * t22_ - (4 * Jpeak_ * pow(t11_, 2) * cos((Pi * t) / (2. * t11_))) / pow(Pi, 2);
		pos = pre2_[3] + (t * (Jpeak_ * (2 * t * t11_ + 4 * pow(t11_, 2) + Pi * t * t22_ + 4 * t11_ * t22_ + Pi * pow(t22_, 2)) + a0_ * Pi * (t + 2 * (t11_ + t22_)) + 2 * Pi * v0_)) / (2. * Pi) -
			(8 * Jpeak_ * pow(t11_, 3) * sin((Pi * t) / (2. * t11_))) / pow(Pi, 3);
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg4( double t) {
	double jerk, accel, vel, pos;
	pre3_ = pre3_.empty()? Seg3(t11_): pre3_;
	mA_ = pre3_[1];
	if (t11_ == 0)
	{
		jerk = 0;
		accel = mA_;
		vel = pre3_[2] + mA_ * t;
		pos = pre3_[3] + (t * (Apeak_ * t + (2 * t11_ + t22_) * (2 * a0_ + Jpeak_ * t22_) + 2 * v0_)) / 2.;
	}
	else
	{
		jerk = 0;
		accel = mA_;
		vel = pre3_[2] + mA_ * t;
		pos = pre3_[3] + (t * (mA_ * Pi * t + (2 * t11_ + t22_) * (2 * a0_ * Pi + 4 * Jpeak_ * t11_ + Jpeak_ * Pi * t22_) + 2 * Pi * v0_)) / (2. * Pi);
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg5( double t) {
	double jerk, accel, vel, pos;
	pre4_ = pre4_.empty()? Seg4(t33_):pre4_;
	if (t1_ == 0)
	{
		jerk = 0;
		accel = pre4_[1];
		vel = pre4_[2] + accel*t;
		pos = pre4_[3] + (accel * pow(t, 2)) / 2. + t * pre4_[2];
	}
	else
	{
		jerk = -(Jpeak_ * sin((Pi * t) / (2. * t1_)));
		accel = pre4_[1] + (-2 * Jpeak_ * t1_) / Pi + (2 * Jpeak_ * t1_ * cos((Pi * t) / (2. * t1_))) / Pi;
		vel = pre4_[2] + (mA_ * Pi * t - 2 * Jpeak_ * t * t1_ + (4 * Jpeak_ * pow(t1_, 2) * sin((Pi * t) / (2. * t1_))) / Pi) / Pi;
		pos = pre4_[3] + (8 * Jpeak_ * pow(t1_, 3)) / pow(Pi, 3) + (t * (-2 * Jpeak_ * t * t1_ + Jpeak_ * (2 * t11_ + t22_) * (4 * t11_ + Pi * t22_) + mA_ * Pi * (t + 2 * t33_) +
			2 * Pi * (a0_ * (2 * t11_ + t22_) + v0_))) / (2. * Pi) - (8 * Jpeak_ * pow(t1_, 3) * cos((Pi * t) / (2. * t1_))) / pow(Pi, 3);
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg6( double t) {
	double jerk, accel, vel, pos;
	pre5_ = pre5_.empty()? Seg5(t1_):pre5_;
	if (t1_ == 0)
	{
		jerk = -Jpeak_;
		accel = pre5_[1] - Jpeak_ * t;
		vel = pre5_[2] + accel * t;
		pos = pre5_[3] - (Jpeak_ * pow(t, 3)) / 6. + (accel * pow(t, 2)) / 2. + t * pre5_[2];
	}
	else
	{
		jerk = -Jpeak_;
		accel = pre5_[1] - Jpeak_ * t;
		vel = pre5_[2] + mA_ * t - (Jpeak_ * pow(t, 2)) / 2. - (2 * Jpeak_ * t * t1_) / Pi;
		pos = pre5_[3]+(t* ((Jpeak_* (24 * pow(t1_, 2) - 6 * Pi * (t1_ * (t + 2 * t1_) - 2 * t11_ * (2 * t11_ + t22_)) + pow(Pi, 2) * (-pow(t, 2) + 3 * t22_ * (2 * t11_ + t22_)))) / pow(Pi, 2) +
			3 * mA_ * (t + 2 * (t1_ + t33_)) + 6 * (a0_ * (2 * t11_ + t22_) + v0_))) / 6.;
	}

	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg7( double t) {
	double jerk, accel, vel, pos;
	pre6_ = pre6_.empty()? Seg6(t2_):pre6_;
	if (t1_ == 0)
	{
		jerk = 0;
		accel = pre6_[1];
		vel = pre6_[2] + accel * t;
		pos = pre6_[3] + (accel * pow(t, 2)) / 2. + t * pre6_[2];
	}
	else
	{
		jerk = -(Jpeak_ * sin((Pi * (1 + t / t1_)) / 2.));
		accel = pre6_[1] + (-2 * Jpeak_ * t1_ * sin((Pi * t) / (2. * t1_))) / Pi;
		vel = pre6_[2] + mA_ * t - (2 * Jpeak_ * t1_ * (Pi * t + 2 * t1_)) / pow(Pi, 2) - Jpeak_ * t * t2_ + (4 * Jpeak_ * pow(t1_, 2) * cos((Pi * t) / (2. * t1_))) / pow(Pi, 2);
		pos = pre6_[3] + (Jpeak_ * t * (-4 * pow(t1_, 2) - 4 * t1_ * t2_ - Pi * pow(t2_, 2) - t * (2 * t1_ + Pi * t2_) + (2 * t11_ + t22_) * (4 * t11_ + Pi * t22_))) / (2. * Pi) +
			(mA_ * t * (t + 2 * (t1_ + t2_ + t33_))) / 2. + t * (a0_ * (2 * t11_ + t22_) + v0_) + (8 * Jpeak_ * pow(t1_, 3) * sin((Pi * t) / (2. * t1_))) / pow(Pi, 3);
	}

	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg8( double t) {
	double jerk, accel, vel, pos;
	pre7_ = pre7_.empty()? Seg7(t1_):pre7_;
	mV_ = pre7_[2];
	if (t1_ == 0)
	{
		jerk = 0;
		accel = pre7_[1];
		vel = pre7_[2] + accel * t;
		pos = pre7_[3] + (accel * pow(t, 2)) / 2. + t * pre7_[2];
	}
	else
	{
		jerk = 0;
		accel = pre7_[1];
		vel =mV_;
		pos = pre7_[3] + mV_*t;
	}

	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg9( double t) {
	double jerk, accel, vel, pos;
	pre8_ = pre8_.empty()? Seg8(t4_) : pre8_;
	if (t1_ == 0)
	{
		jerk = 0;
		accel = pre8_[1];
		vel = pre8_[2] + accel * t;
		pos = pre8_[3] + (accel * pow(t, 2)) / 2. + t * pre8_[2];
	}
	else
	{
		jerk = -(Jpeak_ * sin((Pi * t) / (2. * t1_)));
		accel = pre8_[1] + (-2 * Jpeak_ * t1_) / Pi + (2 * Jpeak_ * t1_ * cos((Pi * t) / (2. * t1_))) / Pi;
		vel = pre8_[2] + (mA_ * Pi * t - 6 * Jpeak_ * t * t1_ - Jpeak_ * Pi * t * t2_ + (4 * Jpeak_ * pow(t1_, 2) * sin((Pi * t) / (2. * t1_))) / Pi) / Pi;
		pos = pre8_[3] + mV_ * t + (mA_ * pow(t, 2)) / 2. - (3 * Jpeak_ * pow(t, 2) * t1_) / Pi + (8 * Jpeak_ * pow(t1_, 3)) / pow(Pi, 3) - (Jpeak_ * pow(t, 2) * t2_) / 2. -
			(8 * Jpeak_ * pow(t1_, 3) * cos((Pi * t) / (2. * t1_))) / pow(Pi, 3);
	}

	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg10( double t) {
	double jerk, accel, vel, pos;
	pre9_ = pre9_.empty()? Seg9(t1_):pre9_;
	if (t1_ == 0)
	{
		jerk = -Jpeak_;
		accel = pre9_[1] - Jpeak_ * t;
		vel = pre9_[2] + accel * t;
		pos = pre9_[3] - (Jpeak_ * pow(t, 3)) / 6. + (accel * pow(t, 2)) / 2. + t * pre9_[2];
	}
	else
	{
		jerk = -Jpeak_;
		accel = pre9_[1] - Jpeak_ * t;
		vel = pre9_[2] + mA_ * t - (Jpeak_ * pow(t, 2)) / 2. - (6 * Jpeak_ * t * t1_) / Pi - Jpeak_ * t * t2_;
		pos = pre9_[3]+(t* (6 * mV_ + 3 * mA_ * (t + 2 * t1_) - (Jpeak_ * (-24 * pow(t1_, 2) + 18 * Pi * t1_ * (t + 2 * t1_) + pow(Pi, 2) * (pow(t, 2) + 3 * t * t2_ + 6 * t1_ * t2_))) / pow(Pi, 2))) / 6.;
	}

	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg11( double t) {
	double jerk, accel, vel, pos;
	pre10_ = pre10_.empty()? Seg10(t2_):pre10_;
	if (t1_ == 0)
	{
		jerk = 0;
		accel = pre10_[1];
		vel = pre10_[2] + accel * t;
		pos = pre10_[3] + (accel * pow(t, 2)) / 2. + t * pre10_[2];
	}
	else
	{
		jerk = -(Jpeak_ * sin((Pi * (1 + t / t1_)) / 2.));
		accel = pre10_[1]+ (-2 * Jpeak_ * t1_ * sin((Pi * t) / (2. * t1_))) / Pi;
		vel = pre10_[2] + mA_ * t - (2 * Jpeak_ * t1_ * (3 * Pi * t + 2 * t1_)) / pow(Pi, 2) - 2 * Jpeak_ * t * t2_ + (4 * Jpeak_ * pow(t1_, 2) * cos((Pi * t) / (2. * t1_))) / pow(Pi, 2);
		pos = pre10_[3] + mV_ * t - (Jpeak_ * t * (6 * t1_ * (t + 2 * t1_) + 2 * (6 * t1_ + Pi * (t + t1_)) * t2_ + 3 * Pi * pow(t2_, 2))) / (2. * Pi) + (mA_ * t * (t + 2 * (t1_ + t2_))) / 2. +
			(8 * Jpeak_ * pow(t1_, 3) * sin((Pi * t) / (2. * t1_))) / pow(Pi, 3);
	}

	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg12( double t) {
	double jerk, accel, vel, pos;
	pre11_ = pre11_.empty()? Seg11(t1_):pre11_;
	if (t1_ == 0)
	{
		jerk = 0;
		accel = pre11_[1];
		vel = pre11_[2] + accel * t;
		pos = pre11_[3] + (accel * pow(t, 2)) / 2. + t * pre11_[2];
	}
	else
	{
		jerk = 0;
		accel = pre11_[1];
		vel = pre11_[2] + mA_ * t - (8 * Jpeak_ * t * t1_) / Pi - 2 * Jpeak_ * t * t2_;
		pos = pre11_[3] + mV_ * t + (mA_ * t * (t + 4 * t1_ + 2 * t2_)) / 2. - (Jpeak_ * t * (2 * t + 6 * t1_ + 3 * t2_) * (4 * t1_ + Pi * t2_)) / (2. * Pi);
	}

	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg13( double t) {
	double jerk, accel, vel, pos;
	pre12_ = pre12_.empty()? Seg12(t3_): pre12_;
	if (t1_ == 0)
	{
		jerk = 0;
		accel = pre12_[1];
		vel = pre12_[2] + accel * t;
		pos = pre12_[3] + (accel * pow(t, 2)) / 2. + t * pre12_[2];
	}
	else
	{
		jerk = Jpeak_ * sin((Pi * t) / (2. * t1_));
		accel = pre12_[1]+ (2 * Jpeak_ * t1_) / Pi - (2 * Jpeak_ * t1_ * cos((Pi * t) / (2. * t1_))) / Pi;
		vel = pre12_[2] + (mA_ * Pi * t - 6 * Jpeak_ * t * t1_ - 2 * Jpeak_ * Pi * t * t2_ - (4 * Jpeak_ * pow(t1_, 2) * sin((Pi * t) / (2. * t1_))) / Pi) / Pi;
		pos = pre12_[3] + mV_ * t - (8 * Jpeak_ * pow(t1_, 3)) / pow(Pi, 3) + (mA_ * t * (t + 2 * (2 * t1_ + t2_ + t3_))) / 2. -
			(Jpeak_ * t * (2 * t * (3 * t1_ + Pi * t2_) + (4 * t1_ + Pi * t2_) * (6 * t1_ + 3 * t2_ + 4 * t3_))) / (2. * Pi) + (8 * Jpeak_ * pow(t1_, 3) * cos((Pi * t) / (2. * t1_))) / pow(Pi, 3);
	}

	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg14( double t) {
	double jerk, accel, vel, pos;
	pre13_ = pre13_.empty()? Seg13(t1_):pre13_;
	if (t1_ == 0)
	{
		jerk = Jpeak_;
		accel = pre13_[1] + Jpeak_ * t;
		vel = pre13_[2] + accel * t;
		pos = pre13_[3] + (Jpeak_ * pow(t, 3)) / 6. + (accel * pow(t, 2)) / 2. + t * pre13_[2];
	}
	else
	{
		jerk = Jpeak_;
		accel = pre13_[1] + Jpeak_ * t;
		vel = pre13_[2] + mA_ * t + (Jpeak_ * pow(t, 2)) / 2. - (6 * Jpeak_ * t * t1_) / Pi - 2 * Jpeak_ * t * t2_;
		pos = pre13_[3] + (t * (6 * mV_ + 3 * mA_ * (t + 2 * (3 * t1_ + t2_ + t3_)) + (Jpeak_ * (-24 * pow(t1_, 2) - 6 * Pi * t1_ * (3 * t + 18 * t1_ + 6 * t2_ + 8 * t3_) +
			pow(Pi, 2) * (pow(t, 2) - 6 * t * t2_ - 3 * t2_ * (10 * t1_ + 3 * t2_ + 4 * t3_)))) / pow(Pi, 2))) / 6.;
	}

	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg15( double t) {
	double jerk, accel, vel, pos;
	pre14_ = pre14_.empty()? Seg14(t2_):pre14_;
	if (t1_ == 0)
	{
		jerk = 0;
		accel = pre14_[1];
		vel = pre14_[2] + accel * t;
		pos = pre14_[3] + (accel * pow(t, 2)) / 2. + t * pre14_[2];
	}
	else
	{
		jerk = Jpeak_ * sin((Pi * (1 + t / t1_)) / 2.);
		accel = pre14_[1]+(2 * Jpeak_ * t1_ * sin((Pi * t) / (2. * t1_))) / Pi;
		vel = pre14_[2] + mA_ * t + (2 * Jpeak_ * t1_ * (-3 * Pi * t + 2 * t1_)) / pow(Pi, 2) - Jpeak_ * t * t2_ - (4 * Jpeak_ * pow(t1_, 2) * cos((Pi * t) / (2. * t1_))) / pow(Pi, 2);
		pos = pre14_[3] + mV_ * t + (mA_ * t * (t + 6 * t1_ + 4 * t2_ + 2 * t3_)) / 2. - (Jpeak_ * t * (6 * t * t1_ + 36 * pow(t1_, 2) + Pi * t * t2_ + 24 * t1_ * t2_ + 10 * Pi * t1_ * t2_ + 6 * Pi * pow(t2_, 2) +
			16 * t1_ * t3_ + 4 * Pi * t2_ * t3_)) / (2. * Pi) - (8 * Jpeak_ * pow(t1_, 3) * sin((Pi * t) / (2. * t1_))) / pow(Pi, 3);
	}

	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg16(double t) {
	double jerk, accel, vel, pos;
	pre15_ = pre15_.empty()? Seg15(t1_):pre15_;
	jerk = 0;
	accel = pre15_[1];
	vel = pre15_[2];
	pos = pre15_[3];
	return { jerk,accel,vel,pos };
};
