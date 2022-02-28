#include<profileSegments.h>
# define Pi           3.14159265358979323846

profileSeg::profileSeg(double v0, double a0, double p0, double alpha, double t11, double t22, double t33, double t1, double t2, double sa, double maxJ, double maxA, double maxV) :v0_(v0), a0_(a0), p0_(p0), alpha_(alpha), t1_(t1), t2_(t2), t11_(t11), t22_(t22), t33_(t33), sa_(sa), mJ_(maxJ), mA_(maxA), mV_(maxV) { Jpeak_ = maxJ; Apeak_ = 0; Vpeak_ = 0; };
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
		jerk = sa_*Jpeak_ * sin(Pi*t/(2*t11_));
		accel = a0_ + (2 *sa_* Jpeak_ * t11_) / Pi - (2 * sa_ * Jpeak_ * t11_ * cos((Pi * t) / (2. * t11_))) / Pi;
		vel = a0_ * t + (2 * sa_ * Jpeak_ * t * t11_) / Pi + v0_ - (4 * sa_ * Jpeak_ * pow(t11_, 2) * sin((Pi * t) / (2. * t11_))) / pow(Pi, 2);
		pos = p0_ + (a0_ * pow(t, 2)) / 2 + (sa_*Jpeak_ * pow(t, 2) * t11_) / Pi - (8 * sa_*Jpeak_ * pow(t11_, 3)) / pow(Pi, 3) +
			t * v0_ + (8 * sa_*Jpeak_ * pow(t11_, 3) * cos((Pi * t) / (2 * t11_))) / pow(Pi, 3);
	}
	return {jerk,accel,vel,pos};
};
vector<double> profileSeg::Seg2( double t) {
	double jerk, accel, vel, pos;
	pre1_ = pre1_.empty()? Seg1(t11_):pre1_;
	if (t11_ == 0)
	{
		jerk = sa_ * Jpeak_;
		accel = pre1_[1]+ sa_ * Jpeak_*t;
		vel = pre1_[2]+a0_*t+sa_*Jpeak_*pow(t,2)/2;
		pos = pre1_[3]+ (sa_*Jpeak_ * pow(t, 3)) / 6. + a0_ * t * t11_ + (a0_ * pow(t, 2)) / 2. + t * v0_;
	}
	else
	{
		jerk = sa_*Jpeak_;
		accel = pre1_[1] + sa_*Jpeak_ * t;
		vel = pre1_[2] + pre1_[1]*t + 0.5*Jpeak_*sa_*pow(t,2);
		pos = pre1_[3] + (pre1_[1] * pow(t, 2)) / 2. + (Jpeak_ * sa_ * pow(t, 3)) / 6. + t * pre1_[2];
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
		vel = pre2_[2] + a0_ * t + sa_*Jpeak_ * t* t22_;
		pos = pre2_[3] + (a0_ * pow(t, 2)) / 2. + a0_ * t * t1_ + a0_ * t * t22_ + (sa_*Jpeak_ * pow(t, 2) * t22_) / 2. + (sa_*Jpeak_ * t * pow(t22_, 2)) / 2. + t * v0_;
	}
	else
	{
		jerk = sa_*Jpeak_ * sin((Pi * (1 + t / t11_)) / 2.);
		accel = pre2_[1] + (2 * sa_*Jpeak_ * t11_ * sin((Pi * t) / (2. * t11_))) / Pi;
		vel = pre2_[1] * t + (4 * Jpeak_ * sa_ * pow(t11_, 2)) / pow(Pi, 2) + pre2_[2] - (4 * Jpeak_ * sa_ * pow(t11_, 2) * cos((Pi * t) / (2. * t11_))) / pow(Pi, 2);
		pos = pre2_[3] + (t * (pre2_[1] * t + (8 * Jpeak_ * sa_ * pow(t11_, 2)) / pow(Pi, 2) + 2 * pre2_[2])) / 2. - (8 * Jpeak_ * sa_ * pow(t11_, 3) * sin((Pi * t) / (2. * t11_))) / pow(Pi, 3);
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg4( double t) {
	double jerk, accel, vel, pos;
	pre3_ = pre3_.empty()? Seg3(t11_): pre3_;
	Apeak_ = pre3_[1];
	if (t11_ == 0)
	{
		jerk = 0;
		accel = Apeak_;
		vel = pre3_[2] + Apeak_ * t;
		pos = pre3_[3] + pre3_[2]*t + Apeak_ * pow(t,2)/2;
	}
	else
	{
		jerk = 0;
		accel = Apeak_;
		vel = pre3_[2] + Apeak_ * t;
		pos = pre3_[3] + pre3_[2] * t + Apeak_ * pow(t, 2) / 2;
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
		jerk = -(sa_*Jpeak_ * sin((Pi * t) / (2. * t1_)));
		accel = pre4_[1] + (-2 * sa_*Jpeak_ * t1_) / Pi + (2 * sa_* Jpeak_ * t1_ * cos((Pi * t) / (2. * t1_))) / Pi;
		vel = pre4_[2] + (pre4_[1] * Pi * t - 2 * Jpeak_ * sa_ * t * t1_ + (4 * Jpeak_ * sa_ * pow(t1_, 2) * sin((Pi * t) / (2. * t1_))) / Pi) / Pi;
		pos = pre4_[3] + (pre4_[1] * pow(t, 2)) / 2. + (Jpeak_ * sa_ * t1_ * (-(pow(Pi, 2) * pow(t, 2)) + 8 * pow(t1_, 2))) / pow(Pi, 3) + t * pre4_[2] -
			(8 * Jpeak_ * sa_ * pow(t1_, 3) * cos((Pi * t) / (2. * t1_))) / pow(Pi, 3);
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg6( double t) {
	double jerk, accel, vel, pos;
	pre5_ = pre5_.empty()? Seg5(t1_):pre5_;
	if (t1_ == 0)
	{
		jerk = -sa_*Jpeak_;
		accel = pre5_[1] - sa_*Jpeak_ * t;
		vel = pre5_[2] + accel * t;
		pos = pre5_[3] - (sa_*Jpeak_ * pow(t, 3)) / 6. + (sa_*accel * pow(t, 2)) / 2. + t * pre5_[2];
	}
	else
	{
		jerk = -sa_*Jpeak_;
		accel = pre5_[1] - sa_*Jpeak_ * t;
		vel = pre5_[2] + pre5_[1] * t - (Jpeak_ * sa_ * pow(t, 2)) / 2.;
		pos = pre5_[3]+ (t * (3 * pre5_[1] * t - Jpeak_ * sa_ * pow(t, 2) + 6 * pre5_[2])) / 6.;
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
		jerk = -(sa_*Jpeak_ * sin((Pi * (1 + t / t1_)) / 2.));
		accel = pre6_[1] + (-2 * sa_* Jpeak_ * t1_ * sin((Pi * t) / (2. * t1_))) / Pi;
		vel = pre6_[2] + pre6_[1] * t - (4 * Jpeak_ * sa_ * pow(t1_, 2)) / pow(Pi, 2) + (4 * Jpeak_ * sa_ * pow(t1_, 2) * cos((Pi * t) / (2. * t1_))) / pow(Pi, 2);
		pos = pre6_[3] + (t * (pre6_[1] * t - (8 * Jpeak_ * sa_ * pow(t1_, 2)) / pow(Pi, 2) + 2 * pre6_[2])) / 2. + (8 * Jpeak_ * sa_ * pow(t1_, 3) * sin((Pi * t) / (2. * t1_))) / pow(Pi, 3);
	}

	return { jerk,accel,vel,pos };
};
vector<double> profileSeg::Seg8( double t) {
	double jerk, accel, vel, pos;
	pre7_ = pre7_.empty()? Seg7(t1_):pre7_;
	Vpeak_ = pre7_[2];
	if (t1_ == 0)
	{
		jerk = 0;
		accel = pre7_[1];
		vel = pre7_[2] + accel * t;
		pos = pre7_[3] + (accel * pow(t, 2)) / 2. + t * Vpeak_;
	}
	else
	{
		jerk = 0;
		accel = pre7_[1];
		vel =Vpeak_;
		pos = pre7_[3] + Vpeak_*t;
	}

	return { jerk,accel,vel,pos };
};