#include<profileSegmentsB.h>
# define Pi           3.14159265358979323846

profileSegB::profileSegB(double v0, double a0, double p0, double pG, double alpha, double t1, double t2, double t3, double t11, double t22, double maxJ, double maxA, double maxV) :v0_(v0), a0_(a0), p0_(p0), pG_(pG), alpha_(alpha), t1_(t1), t2_(t2), t3_(t3), t11_(t11), t22_(t22), Jpeak_(maxJ), Apeak_(maxA), Vpeak_(maxV) {};
profileSegB::~profileSegB() {};
vector<double> profileSegB::Seg1(double t) {
	sa_ = a0_ >= 0 ? 1 : -1;
	double jerk, accel, vel, pos;
	if (t11_ == 0)
	{
		jerk = 0;
		accel = a0_;
		vel = v0_ + accel * t;
		pos = p0_ + a0_ * pow(t, 2) / 2 + v0_ * t;


	}
	else
	{
		jerk = -sa_ * Jpeak_ * sin((Pi * t / (2 * t11_)));
		accel = a0_ - (2 * Jpeak_ * sa_ * t11_) / Pi + (2 * Jpeak_ * sa_ * t11_ * cos((Pi * t) / (2. * t11_))) / Pi;
		vel = a0_ * t - (2 * Jpeak_ * sa_ * t * t11_) / Pi + v0_ + (4 * Jpeak_ * sa_ * pow(t11_, 2) * sin((Pi * t) / (2. * t11_))) / pow(Pi, 2);
		pos = p0_ + (a0_ * pow(t, 2)) / 2. + (Jpeak_ * sa_ * t11_ * (-(pow(Pi, 2) * pow(t, 2)) + 8 * pow(t11_, 2))) / pow(Pi, 3) + t * v0_ -
			(8 * Jpeak_ * sa_ * pow(t11_, 3) * cos((Pi * t) / (2. * t11_))) / pow(Pi, 3);
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSegB::Seg2(double t) {
	double jerk, accel, vel, pos;
	pre1_ =pre1_.empty()? Seg1(t11_):pre1_;
	if (t11_ == 0)
	{
		jerk = -sa_ * Jpeak_;
		accel = pre1_[1] - sa_ * Jpeak_ * t;
		vel = pre1_[2] + accel * t;
		pos = pre1_[3] - (sa_ * Jpeak_ * pow(t, 3)) / 6. + (accel * pow(t, 2)) / 2. + t * pre1_[2];

	}
	else
	{
		jerk = -sa_ * Jpeak_;
		accel = pre1_[1] - sa_ * Jpeak_ * t;
		vel = pre1_[2] + a0_ * t - (Jpeak_ * sa_ * pow(t, 2)) / 2. - (2 * Jpeak_ * sa_ * t * t11_) / Pi;
		pos = pre1_[3] + (t * (3 * a0_ * (t + 2 * t11_) - (Jpeak_ * sa_ * (pow(Pi, 2) * pow(t, 2) + 6 * Pi * t * t11_ + 12 * (-2 + Pi) * pow(t11_, 2))) / pow(Pi, 2) + 6 * v0_)) / 6.;
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSegB::Seg3(double t) {
	double jerk, accel, vel, pos;
	pre2_ = pre2_.empty()? Seg2(t22_):pre2_;
	if (t11_ == 0)
	{
		jerk = 0;
		accel = pre2_[1];
		vel = pre2_[2] + accel * t;
		pos = pre2_[3] + (accel * pow(t, 2)) / 2. + t * pre2_[2];

	}
	else
	{
		jerk = -(Jpeak_ * sa_ * sin((Pi * (1 + t / t11_)) / 2.));
		accel = pre2_[1] + (-2 * Jpeak_ * sa_ * t11_ * sin((Pi * t) / (2. * t11_))) / Pi;
		vel = pre2_[2] + a0_ * t + Jpeak_ * sa_ * ((-2 * t11_ * (Pi * t + 2 * t11_)) / pow(Pi, 2) - t * t22_) + (4 * Jpeak_ * sa_ * pow(t11_, 2) * cos((Pi * t) / (2. * t11_))) / pow(Pi, 2);
		pos =pre2_[3] -((Jpeak_ * sa_ * t * t11_ * (t + 2 * (t11_ + t22_))) / Pi) + (t * (-(Jpeak_ * sa_ * t22_ * (t + t22_)) + a0_ * (t + 2 * (t11_ + t22_)) + 2 * v0_)) / 2. +
			(8 * Jpeak_ * sa_ * pow(t11_, 3) * sin((Pi * t) / (2. * t11_))) / pow(Pi, 3);
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSegB::Seg9(double t) {
	double jerk, accel, vel, pos;
	pre3_ = pre3_.empty()? Seg3(t11_):pre3_;
	sv_ = pre3_[2] >= 0 ? 1 : -1;
	if (t1_ == 0)
	{
		jerk = 0;
		accel = pre3_[1];
		vel = pre3_[2] + accel * t;
		pos = pre3_[3] + (accel * pow(t, 2)) / 2. + t * pre3_[2];

	}
	else
	{
		jerk = -(Jpeak_ * sv_ * sin((Pi * t) / (2. * t1_)));
		accel = pre3_[1] + (-2 * Jpeak_ * sv_ * t1_) / Pi + (2 * Jpeak_ * sv_ * t1_ * cos((Pi * t) / (2. * t1_))) / Pi;
		vel = pre3_[2] + a0_ * t - (Jpeak_ * t * (2 * sv_ * t1_ + 4 * sa_ * t11_ + Pi * sa_ * t22_)) / Pi + (4 * Jpeak_ * sv_ * pow(t1_, 2) * sin((Pi * t) / (2. * t1_))) / pow(Pi, 2);
		pos = pre3_[3] + (8 * Jpeak_ * sv_ * pow(t1_, 3)) / pow(Pi, 3) + (a0_ * t * (t + 4 * t11_ + 2 * t22_)) / 2. - (Jpeak_ * t * (2 * sv_ * t * t1_ + sa_ * (t + 2 * t11_ + t22_) * (4 * t11_ + Pi * t22_))) / (2. * Pi) +
			t * v0_ - (8 * Jpeak_ * sv_ * pow(t1_, 3) * cos((Pi * t) / (2. * t1_))) / pow(Pi, 3);
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSegB::Seg10(double t) {
	double jerk, accel, vel, pos;
	pre9_ =pre9_.empty()? Seg9(t1_):pre9_;
	if (t1_ == 0)
	{
		jerk = -sv_ * Jpeak_;
		accel = pre9_[1] - sv_ * Jpeak_ * t;
		vel = pre9_[2] + accel * t;
		pos = pre9_[3] - (sv_ * Jpeak_ * pow(t, 3)) / 6. + (accel * pow(t, 2)) / 2. + t * pre9_[2];

	}
	else
	{
		jerk = -sv_ * Jpeak_;
		accel = pre9_[1] - sv_ * Jpeak_ * t;
		vel = pre9_[2] - 0.5 * (Jpeak_ * sv_ * pow(t, 2)) - (2 * Jpeak_ * sv_ * t * t1_) / Pi + (t * (a0_ * Pi - 2 * Jpeak_ * sa_ * t11_ - Jpeak_ * sa_ * (2 * t11_ + Pi * t22_))) / Pi;
		pos = pre9_[3] + (t * (3 * a0_ * (t + 2 * (t1_ + 2 * t11_ + t22_)) - (Jpeak_ * (-24 * sv_ * pow(t1_, 2) + 6 * Pi * (sv_ * t1_ * (t + 2 * t1_) + 2 * sa_ * t11_ * (t + 2 * (t1_ + t11_) + t22_)) +
			pow(Pi, 2) * (sv_ * pow(t, 2) + 3 * sa_ * t22_ * (t + 2 * (t1_ + t11_) + t22_)))) / pow(Pi, 2) + 6 * v0_)) / 6.;
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSegB::Seg11(double t) {
	double jerk, accel, vel, pos;
	pre10_ =pre10_.empty()? Seg10(t2_):pre10_;
	if (t1_ == 0)
	{
		jerk = 0;
		accel = pre10_[1];
		vel = pre10_[2] + accel * t;
		pos = pre10_[3] + (accel * pow(t, 2)) / 2. + t * pre10_[2];

	}
	else
	{
		jerk = -(Jpeak_ * sv_ * sin((Pi * (1 + t / t1_)) / 2.));
		accel = pre10_[1] + (-2 * Jpeak_ * sv_ * t1_ * sin((Pi * t) / (2. * t1_))) / Pi;
		vel = pre10_[2] + a0_ * t - (4 * Jpeak_ * sv_ * pow(t1_, 2)) / pow(Pi, 2) - (Jpeak_ * t * (2 * sv_ * t1_ + 4 * sa_ * t11_ + Pi * sv_ * t2_ + Pi * sa_ * t22_)) / Pi +
			(4 * Jpeak_ * sv_ * pow(t1_, 2) * cos((Pi * t) / (2. * t1_))) / pow(Pi, 2);
		pos = pre10_[3] + (a0_ * t * (t + 2 * (t1_ + 2 * t11_ + t2_ + t22_))) / 2. - (Jpeak_ * t * (sv_ * (2 * t * t1_ + 4 * pow(t1_, 2) + Pi * t * t2_ + 4 * t1_ * t2_ + Pi * pow(t2_, 2)) +
			sa_ * (t + 2 * (t1_ + t11_ + t2_) + t22_) * (4 * t11_ + Pi * t22_))) / (2. * Pi) + t * v0_ + (8 * Jpeak_ * sv_ * pow(t1_, 3) * sin((Pi * t) / (2. * t1_))) / pow(Pi, 3);
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSegB::Seg12(double t) {
	double jerk, accel, vel, pos;
	pre11_ =pre11_.empty()? Seg11(t1_):pre11_;
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
		vel = pre11_[2] + (-4 * Jpeak_ * sv_ * t * t1_) / Pi - Jpeak_ * sv_ * t * t2_ + (t * (a0_ * Pi - 2 * Jpeak_ * sa_ * t11_ - Jpeak_ * sa_ * (2 * t11_ + Pi * t22_))) / Pi;
		pos = pre11_[3] + (a0_ * t * (t + 2 * (2 * t1_ + 2 * t11_ + t2_ + t22_))) / 2. - (Jpeak_ * t * (sv_ * (t + 2 * t1_ + t2_) * (4 * t1_ + Pi * t2_) +
			sa_ * (t + 2 * (2 * t1_ + t11_ + t2_) + t22_) * (4 * t11_ + Pi * t22_))) / (2. * Pi) + t * v0_;
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSegB::Seg13(double t) {
	double jerk, accel, vel, pos;
	pre12_ =pre12_.empty()? Seg12(t3_):pre12_;
	if (t1_ == 0)
	{
		jerk = 0;
		accel = pre12_[1];
		vel = pre12_[2] + accel * t;
		pos = pre12_[3] + (accel * pow(t, 2)) / 2. + t * pre12_[2];

	}
	else
	{
		jerk = Jpeak_ * sv_ * sin((Pi * t) / (2. * t1_));
		accel = pre12_[1] + (2 * Jpeak_ * sv_ * t1_) / Pi - (2 * Jpeak_ * sv_ * t1_ * cos((Pi * t) / (2. * t1_))) / Pi;
		vel = pre12_[2] + a0_ * t - (Jpeak_ * t * (2 * sv_ * t1_ + 4 * sa_ * t11_ + Pi * sv_ * t2_ + Pi * sa_ * t22_)) / Pi - (4 * Jpeak_ * sv_ * pow(t1_, 2) * sin((Pi * t) / (2. * t1_))) / pow(Pi, 2);
		pos = pre12_[3] + (-8 * Jpeak_ * sv_ * pow(t1_, 3)) / pow(Pi, 3) + (a0_ * t * (t + 2 * (2 * t1_ + 2 * t11_ + t2_ + t22_ + t3_))) / 2. -
			(Jpeak_ * t * (sv_ * t * (2 * t1_ + Pi * t2_) + sv_ * (4 * t1_ + Pi * t2_) * (2 * t1_ + t2_ + 2 * t3_) + sa_ * (4 * t11_ + Pi * t22_) * (t + 4 * t1_ + 2 * t11_ + 2 * t2_ + t22_ + 2 * t3_))) /
			(2. * Pi) + t * v0_ + (8 * Jpeak_ * sv_ * pow(t1_, 3) * cos((Pi * t) / (2. * t1_))) / pow(Pi, 3);
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSegB::Seg14(double t) {
	double jerk, accel, vel, pos;
	pre13_ =pre13_.empty()? Seg13(t1_):pre13_;
	if (t1_ == 0)
	{
		jerk = sv_ * Jpeak_;
		accel = pre13_[1] + sv_ * Jpeak_ * t;
		vel = pre13_[2] + accel * t;
		pos = pre13_[3] + (sv_ * Jpeak_ * pow(t, 3)) / 6. + (accel * pow(t, 2)) / 2. + t * pre13_[2];

	}
	else
	{
		jerk = sv_ * Jpeak_;
		accel = pre13_[1] + Jpeak_ * sv_ * t;
		vel = pre13_[2] + (Jpeak_ * sv_ * pow(t, 2)) / 2. - (2 * Jpeak_ * sv_ * t * t1_) / Pi - Jpeak_ * sv_ * t * t2_ + (t * (a0_ * Pi - 2 * Jpeak_ * sa_ * t11_ - Jpeak_ * sa_ * (2 * t11_ + Pi * t22_))) / Pi;
		pos = pre13_[3] + (-4 * Jpeak_ * sv_ * t * pow(t1_, 2)) / pow(Pi, 2) - (Jpeak_ * t * (2 * sa_ * t11_ * (t + 6 * t1_ + 2 * t11_ + 2 * t2_ + t22_ + 2 * t3_) + sv_ * t1_ * (t + 6 * t1_ + 2 * t2_ + 4 * t3_))) / Pi +
			(t * (-3 * Jpeak_ * sa_ * t22_ * (t + 6 * t1_ + 2 * t11_ + 2 * t2_ + t22_ + 2 * t3_) + 3 * a0_ * (t + 2 * (3 * t1_ + 2 * t11_ + t2_ + t22_ + t3_)) +
				Jpeak_ * sv_ * (pow(t, 2) - 3 * t * t2_ - 3 * t2_ * (4 * t1_ + t2_ + 2 * t3_)) + 6 * v0_)) / 6.;
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSegB::Seg15(double t) {
	double jerk, accel, vel, pos;
	pre14_ =pre14_.empty()? Seg14(t2_):pre14_;
	if (t1_ == 0)
	{
		jerk = 0;
		accel = pre14_[1];
		vel = pre14_[2] + accel * t;
		pos = pre14_[3] + (accel * pow(t, 2)) / 2. + t * pre14_[2];

	}
	else
	{
		jerk = Jpeak_ * sv_ * sin((Pi * (1 + t / t1_)) / 2.);
		accel = pre14_[1] + (2 * Jpeak_ * sv_ * t1_ * sin((Pi * t) / (2. * t1_))) / Pi;
		vel = pre14_[2] + a0_ * t + (4 * Jpeak_ * sv_ * pow(t1_, 2)) / pow(Pi, 2) - (Jpeak_ * t * (2 * sv_ * t1_ + 4 * sa_ * t11_ + Pi * sa_ * t22_)) / Pi -
			(4 * Jpeak_ * sv_ * pow(t1_, 2) * cos((Pi * t) / (2. * t1_))) / pow(Pi, 2);
		pos = pre14_[3] + (a0_ * t * (t + 2 * (3 * t1_ + 2 * t11_ + 2 * t2_ + t22_ + t3_))) / 2. - (Jpeak_ * t *
			(sa_ * (4 * t11_ + Pi * t22_) * (t + 6 * t1_ + 2 * t11_ + 4 * t2_ + t22_ + 2 * t3_) + 2 * sv_ * (t * t1_ + 6 * pow(t1_, 2) + 2 * (2 + Pi) * t1_ * t2_ + 4 * t1_ * t3_ + Pi * t2_ * (t2_ + t3_)))
			) / (2. * Pi) + t * v0_ - (8 * Jpeak_ * sv_ * pow(t1_, 3) * sin((Pi * t) / (2. * t1_))) / pow(Pi, 3);
	}
	return { jerk,accel,vel,pos };
};
vector<double> profileSegB::Seg16(double t) {
	double jerk, accel, vel, pos;
	pre15_ =pre15_.empty()? Seg15(t1_):pre15_;
	jerk = 0;
	accel = pre15_[1];
	vel = pre15_[2] + accel * t;
	pos = pre15_[3] + (accel * pow(t, 2)) / 2. + t * pre15_[2];

	return { jerk,accel,vel,pos };
};
