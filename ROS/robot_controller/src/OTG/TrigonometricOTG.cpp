#include<TrigonometricOTG.h>
# define Pi           3.14159265358979323846
#define eps           0.00000000001
TrigonometricOTG::TrigonometricOTG() {};

TrigonometricOTG::TrigonometricOTG(int num_dof, double rate) : num_dof_(num_dof), mJ_(vector<double>(num_dof)), mA_(vector<double>(num_dof)), mV_(vector<double>(num_dof)), rate_(rate),trajTimes_(vector<Times>(num_dof_)) { complete_ = false; minT_ = 0; };

TrigonometricOTG::TrigonometricOTG(int num_dof, double mJ, double mA, double mV, double rate) :num_dof_(num_dof), mJ_(vector<double>(num_dof, mJ)), mA_(vector<double>(num_dof, mA)), mV_(vector<double>(num_dof, mV)), rate_(rate),trajTimes_(vector<Times>(num_dof_)) { complete_ = false; minT_ = 0;};

TrigonometricOTG::TrigonometricOTG(int num_dof, vector<double> mJ, vector<double> mA, vector<double> mV, vector<double> a0, vector<double> v0, vector<double> p0, vector<double> pG, vector<double> alpha, double rate) : num_dof_(num_dof), mJ_(mJ), mA_(mA), mV_(mV), a0_(a0), v0_(v0), p0_(p0), pG_(pG), alpha_(alpha), rate_(rate),trajTimes_(vector<Times>(num_dof_)) { complete_ = false; minT_ = 0;};

TrigonometricOTG::TrigonometricOTG(int num_dof, double mJ, double mA, double mV, vector<double> a0, vector<double> v0, vector<double> p0, vector<double> pG, double alpha, double rate):num_dof_(num_dof), mJ_(vector<double>(num_dof, mJ)), mA_(vector<double>(num_dof, mA)), mV_(vector<double>(num_dof, mV)), a0_(a0), v0_(v0), p0_(p0), pG_(pG), alpha_(vector<double>(num_dof, alpha)), rate_(rate),trajTimes_(vector<Times>(num_dof_)) { complete_ = false; minT_ = 0;};


TrigonometricOTG::~TrigonometricOTG() {};

double TrigonometricOTG::typeI(double v0, double a0, double p0, double pG, double alpha, double t1, double t2, double t3, double t11, double t22, double t33, double Jpeak, double Apeak, double Vpeak) {
    double t4 = -((p0 + (-6.0 * pG + 3.0 * a0 * (2.0 * t11 + t22) * (4.0 * t1 + 2.0 * t11 + 2.0 * t2 + t22 + 2.0 * t33) +
        3.0 * Apeak * (20.0 * pow(t1, 2.0) + 5 * pow(t2, 2.0) + pow(t3, 2.0) + pow(t33, 2.0) + 2.0 * t2 * (2.0 * t3 + t33) + 4 * t1 * (5.0 * t2 + 2.0 * t3 + t33)) +
        (Jpeak * ((96.0 - 312.0 * pow(Pi, 2.0)) * pow(t1, 3.0) + 24.0 * (-4.0 + pow(Pi, 2.0)) * pow(t11, 3.0) -
            3.0 * Pi * pow(t1, 2.0) * ((-8.0 + Pi * (104.0 + 25.0 * Pi)) * t2 + 56.0 * Pi * t3) +
            3.0 * pow(Pi, 2.0) * t1 * (2.0 * (2.0 * t11 + t22) * (4.0 * t11 + Pi * t22) - 14.0 * Pi * t2 * t3 - 8.0 * pow(t3, 2.0) - t2 * ((26.0 + 25.0 * Pi) * t2 + 28.0 * t3)) +
            3.0 * pow(Pi, 2.0) * (2.0 + Pi) * t11 * t22 * (2.0 * t2 + t22 + 2.0 * t33) + 3.0 * Pi * pow(t11, 2.0) * (-8.0 * t22 + pow(Pi, 2.0) * t22 + 8.0 * Pi * (t2 + t22 + t33)) +
            pow(Pi, 3.0) * (-19.0 * pow(t2, 3.0) - 21.0 * pow(t2, 2.0) * t3 + 3.0 * t2 * (pow(t22, 2.0) - 2 * pow(t3, 2.0)) + pow(t22, 2.0) * (t22 + 3.0 * t33)))) /
        pow(Pi, 3.0) + 6.0 * (2.0 * t1 + 2.0 * t11 + t2 + t22 + t33) * v0 + 6.0 * (4.0 * t1 + 2.0 * t2 + t3) * Vpeak) / 6.0) / Vpeak);
    return t4;
};
vector<double> TrigonometricOTG::typeII(double v0, double a0, double p0, double pG, double alpha, double t1, double t2, double t11, double t22, double Jpeak, double Apeak, double Vpeak) {
    vector<double> ans(3);
    double t4 = 0;
    //vector<double> temp = { -0.08333333333333333 * (6.0 * pow(a0, 2) * Pi * (1 + alpha) - 18 * pow(Apeak, 2) * Pi * (1 + alpha) +
    //    Jpeak * (Pi * (-1 + alpha) - 4 * alpha) * (12 * v0 + sqrt(6) * sqrt((3 * pow(a0, 4) * pow(Pi, 2) * (Pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2) +
    //        4 * pow(a0, 3) * Apeak * (-24 * Pi * (-1 + alpha) * pow(alpha, 2) + 96 * pow(alpha, 3) + 6 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) -
    //            pow(Pi, 3) * (-1 + alpha) * (2 + alpha) * (1 + 2 * alpha)) +
    //        6 * (Pi * (-1 + alpha) - 4 * alpha) * (pow(Apeak, 4) * pow(Pi, 2) * pow(1 + alpha, 2) + 2 * pow(Apeak, 2) * Jpeak * Pi * v0 * (1 + alpha) * (-Pi + (-4 + Pi) * alpha) -
    //            4 * Apeak * pow(Jpeak, 2) * (p0 - pG) * pow(Pi + 4 * alpha - Pi * alpha, 2) + 2 * pow(Jpeak, 2) * pow(v0, 2) * pow(Pi + 4 * alpha - Pi * alpha, 2)) +
    //        12 * a0 * Apeak * (-2 * Jpeak * Pi * v0 * (1 + alpha) * pow(Pi + 4 * alpha - Pi * alpha, 2) +
    //            pow(Apeak, 2) * alpha * (-(pow(Pi, 3) * (-1 + alpha)) - 24 * Pi * (-1 + alpha) * alpha + 96 * pow(alpha, 2) - 2 * pow(Pi, 2) * pow(1 + alpha, 2))) +
    //        6 * pow(a0, 2) * (2 * Jpeak * Pi * v0 * (1 + alpha) * pow(Pi + 4 * alpha - Pi * alpha, 2) +
    //            pow(Apeak, 2) * (48 * Pi * (-1 + alpha) * pow(alpha, 2) - 192 * pow(alpha, 3) + pow(Pi, 3) * (-1 + alpha) * (1 + alpha * (4 + alpha))))) /
    //        (pow(Jpeak, 2) * pow(Pi * (-1 + alpha) - 4 * alpha, 3))))) / (Apeak * Jpeak * (Pi * (-1 + alpha) - 4 * alpha)),
    //    (-6.0 * pow(a0, 2) * Pi * (1 + alpha) + 18 * pow(Apeak, 2) * Pi * (1 + alpha) +
    //        Jpeak * (Pi * (-1 + alpha) - 4 * alpha) * (-12 * v0 + sqrt(6) * sqrt((3 * pow(a0, 4) * pow(Pi, 2) * (Pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2) +
    //            4 * pow(a0, 3) * Apeak * (-24 * Pi * (-1 + alpha) * pow(alpha, 2) + 96 * pow(alpha, 3) + 6 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) -
    //                pow(Pi, 3) * (-1 + alpha) * (2 + alpha) * (1 + 2 * alpha)) +
    //            6 * (Pi * (-1 + alpha) - 4 * alpha) * (pow(Apeak, 4) * pow(Pi, 2) * pow(1 + alpha, 2) + 2 * pow(Apeak, 2) * Jpeak * Pi * v0 * (1 + alpha) * (-Pi + (-4 + Pi) * alpha) -
    //                4 * Apeak * pow(Jpeak, 2) * (p0 - pG) * pow(Pi + 4 * alpha - Pi * alpha, 2) + 2 * pow(Jpeak, 2) * pow(v0, 2) * pow(Pi + 4 * alpha - Pi * alpha, 2)) +
    //            12 * a0 * Apeak * (-2 * Jpeak * Pi * v0 * (1 + alpha) * pow(Pi + 4 * alpha - Pi * alpha, 2) +
    //                pow(Apeak, 2) * alpha * (-(pow(Pi, 3) * (-1 + alpha)) - 24 * Pi * (-1 + alpha) * alpha + 96 * pow(alpha, 2) - 2 * pow(Pi, 2) * pow(1 + alpha, 2))) +
    //            6 * pow(a0, 2) * (2 * Jpeak * Pi * v0 * (1 + alpha) * pow(Pi + 4 * alpha - Pi * alpha, 2) +
    //                pow(Apeak, 2) * (48 * Pi * (-1 + alpha) * pow(alpha, 2) - 192 * pow(alpha, 3) + pow(Pi, 3) * (-1 + alpha) * (1 + alpha * (4 + alpha))))) /
    //            (pow(Jpeak, 2) * pow(Pi * (-1 + alpha) - 4 * alpha, 3))))) / (12. * Apeak * Jpeak * (Pi * (-1 + alpha) - 4 * alpha)) };

    //temp = isValid(temp);
    //double t33 = temp.empty() ? -1 : temp[0];
    double c2 = Apeak;
    double c1 = (pow(a0, 2) * Pi * (1 + alpha) - 3 * pow(Apeak, 2) * Pi * (1 + alpha) + 2 * Jpeak * v0 * (-Pi + (-4 + Pi) * alpha)) / (-(Jpeak * Pi) + Jpeak * (-4 + Pi) * alpha);
    double c0 = (3 * pow(a0, 4) * pow(Pi, 2) * (Pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2) +
        4 * pow(a0, 3) * Apeak * (24 * Pi * (-1 + alpha) * pow(alpha, 2) - 96 * pow(alpha, 3) - 6 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) + pow(Pi, 3) * (-1 + alpha) * (2 + alpha) * (1 + 2 * alpha)) +
        12 * (Pi * (-1 + alpha) - 4 * alpha) * (4 * pow(Apeak, 4) * pow(Pi, 2) * pow(1 + alpha, 2) + 2 * Apeak * p0 * pow(Jpeak, 2) * pow(Pi + 4 * alpha - Pi * alpha, 2) +
            pow(Jpeak, 2) * pow(v0, 2) * pow(Pi + 4 * alpha - Pi * alpha, 2) + 7 * pow(Apeak, 2) * Jpeak * Pi * v0 * (Pi + 4 * alpha - (-4 + Pi) * pow(alpha, 2))) +
        12 * a0 * Apeak * (2 * Jpeak * Pi * v0 * (1 + alpha) * pow(Pi + 4 * alpha - Pi * alpha, 2) +
            pow(Apeak, 2) * alpha * (pow(Pi, 3) * (-1 + alpha) + 24 * Pi * (-1 + alpha) * alpha - 96 * pow(alpha, 2) + 2 * pow(Pi, 2) * pow(1 + alpha, 2))) +
        6 * pow(a0, 2) * (2 * Jpeak * Pi * v0 * (1 + alpha) * pow(Pi + 4 * alpha - Pi * alpha, 2) +
            pow(Apeak, 2) * (-48 * Pi * (-1 + alpha) * pow(alpha, 2) + 192 * pow(alpha, 3) + 24 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) + pow(Pi, 3) * (7 + alpha * (9 - alpha * (9 + 7 * alpha)))))) /
        (24. * Apeak * pow(Jpeak, 2) * pow(Pi * (-1 + alpha) - 4 * alpha, 3))-pG;

    // x - array of size 2
        // return 2: 2 real roots x[0], x[1]
        // return 0: pair of complex roots: x[0]�i*x[1]
    vector<double> x(2);
    int idx = SolveP2(x, c1 / c2, c0 / c2);
    double t33;
    if (idx == 2)
    {
        x = isValid(x);
        t33 =x.empty()? -1: *max_element(x.begin(), x.end());
    }
    else
    {
        t33 = -1;
    }

    double t3 = (Jpeak * (56 *pow(t1, 2) + 14 * (2 + Pi) * t1 * t2 + 7 * Pi * pow(t2, 2) - (2 * t11 + t22) * (4 * t11 + Pi * t22)) - 2 * Apeak * Pi * (6 * t1 + 3 * t2 + t33) -
        2 * Pi * (a0 * (2 * t11 + t22) + v0)) / (2 * Apeak * Pi - 4 * Jpeak * (4 * t1 + Pi * t2));
    return { t3,t4,t33 };
};
vector<double> TrigonometricOTG::typeIII(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak) {
    double t1 = sqrt(Pi*Vpeak / (Jpeak * (4 + Pi * (-1 + pow(alpha, -2)) + 4 / alpha)));
    double t2 = (1 - alpha) * t1 / alpha;
    double t11 = t1 + (a0 * Pi * alpha) / (-(Jpeak * Pi) + Jpeak * (-4 + Pi) * alpha);
    double t22 = (1 - alpha) * t11 / alpha;
    double t3 = 0;
    double t33 = (-(((pow(t1, 2) + pow(t11, 2)) * (1 + alpha)) / alpha) + (2 * Pi * ((v0 - Vpeak) * alpha + a0 * t11 * (1 + alpha))) / (-(Jpeak * Pi) + Jpeak * (-4 + Pi) * alpha)) / (2. * t1);
    double t4 = -((p0 + (Jpeak * (-24 * Pi * (pow(t1, 3) - pow(t11, 3)) * (-1 + alpha) * pow(alpha, 2) + 96 * (pow(t1, 3) - pow(t11, 3)) * pow(alpha, 3) -
        6 * pow(Pi, 2) * (3 * pow(t1, 3) - 2 * t1 * pow(t11, 2) - pow(t11, 3)) * alpha * pow(1 + alpha, 2) +
        pow(Pi, 3) * (t1 - t11) * (-1 + alpha) * (pow(2 * t1 + t11, 2) + (7 * pow(t1, 2) + 7 * t1 * t11 + pow(t11, 2)) * alpha +
            pow(2 * t1 + t11, 2) * pow(alpha, 2))) + 3 * pow(Pi, 3) * alpha *
        (a0 * t11 * (2 * t1 + t11) * pow(1 + alpha, 2) + 2 * alpha * (-(pG * alpha) + ((t1 + t11) * v0 + 2 * t1 * Vpeak) * (1 + alpha)))) / (6. * pow(Pi, 3) * pow(alpha, 3))) / Vpeak);

    return {t1,t2,t3,t4,t11,t22,t33};
};
vector<double> TrigonometricOTG::typeIV(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak) {
    double t3 = 0;
    double t4 = 0;
    vector<double> temp = { (alpha * (3 * (2 * a0 - Apeak) * Pi * (1 + alpha) - sqrt(3 * Pi) * sqrt(-((1 + alpha) *
        (2 * pow(a0, 2) * Pi * (1 + alpha) - 3 * pow(Apeak, 2) * Pi * (1 + alpha) + 4 * Jpeak * v0 * (-Pi + (-4 + Pi) * alpha)))))) /
        (6. * Jpeak * (Pi * (-1 + alpha) - 4 * alpha) * (1 + alpha)), (alpha * (3 * (2 * a0 - Apeak) * Pi * (1 + alpha) +
            sqrt(3 * Pi) * sqrt(-((1 + alpha) * (2 * pow(a0, 2) * Pi * (1 + alpha) - 3 * pow(Apeak, 2) * Pi * (1 + alpha) + 4 * Jpeak * v0 * (-Pi + (-4 + Pi) * alpha)))))) /
        (6. * Jpeak * (Pi * (-1 + alpha) - 4 * alpha) * (1 + alpha)) };
    temp = isValid(temp);
    double t11 = temp.empty()? -1 : *max_element(temp.begin(), temp.end());
    double t1 = t11 + a0 * Pi * alpha / (Jpeak * (Pi + 4 * alpha - Pi * alpha));
    double t22 = (1 - alpha) * t11 / alpha;
    double t2 = (1 - alpha) * t1 / alpha;
    double t33;
    if (abs(t1) < eps)
    {
        t33 = 0;
    }
    else
    {
        temp = {(Pi * alpha * (-v0 + (7 * Jpeak * pow(t11, 2) * (Pi * (-1 + alpha) - 4 * alpha) * (1 + alpha)) / (2. * Pi * pow(alpha, 2)) - (7 * a0 * t11 * (1 + alpha)) / alpha +
            (3 * pow(a0, 2) * Pi * (1 + alpha)) / (-(Jpeak * Pi) + Jpeak * (-4 + Pi) * alpha) -
            sqrt((99 * pow(Jpeak, 4) * Pi * pow(t11, 4) * pow(Pi * (-1 + alpha) - 4 * alpha, 5) * pow(1 + alpha, 2) -
                16 * pow(a0, 3) * Jpeak * pow(Pi, 2) * t11 * (Pi * (-1 + alpha) - 4 * alpha) * pow(alpha, 3) *
                (-24 * Pi * (-1 + alpha) * pow(alpha, 2) + 96 * pow(alpha, 3) - 102 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) + pow(Pi, 3) * (-1 + alpha) * (25 + alpha * (49 + 25 * alpha))) +
                4 * pow(a0, 4) * pow(Pi, 3) * pow(alpha, 4) * (-24 * Pi * (-1 + alpha) * pow(alpha, 2) + 96 * pow(alpha, 3) - 102 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) +
                    pow(Pi, 3) * (-1 + alpha) * (25 + alpha * (49 + 25 * alpha))) +
                12 * pow(Jpeak, 2) * Pi * pow(alpha, 2) * pow(Pi + 4 * alpha - Pi * alpha, 2) *
                (-2 * a0 * (p0 - pG) * pow(Pi, 2) * (Pi * (-1 + alpha) - 4 * alpha) * pow(alpha, 2) + pow(Pi, 2) * pow(v0, 2) * (Pi * (-1 + alpha) - 4 * alpha) * pow(alpha, 2) +
                    2 * pow(a0, 2) * pow(t11, 2) * (-24 * Pi * (-1 + alpha) * pow(alpha, 2) + 96 * pow(alpha, 3) - 102 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) +
                        pow(Pi, 3) * (-1 + alpha) * (25 + alpha * (49 + 25 * alpha)))) -
                12 * t11 * alpha * pow(-(Jpeak * Pi) + Jpeak * (-4 + Pi) * alpha, 3) *
                (-(pow(Pi, 2) * (Pi * (-1 + alpha) - 4 * alpha) * alpha * (2 * (p0 - pG) * alpha + t11 * v0 * (1 + alpha))) +
                    a0 * pow(t11, 2) * (-24 * Pi * (-1 + alpha) * pow(alpha, 2) + 96 * pow(alpha, 3) - 134 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) +
                        pow(Pi, 3) * (-1 + alpha) * (33 + alpha * (65 + 33 * alpha))))) / (pow(Jpeak, 2) * pow(Pi * (-1 + alpha) - 4 * alpha, 3) * pow(alpha, 4))) /
            (2. * sqrt(3) * pow(Pi, 1.5)))) / (a0 * Pi * alpha + Jpeak * t11 * (Pi + 4 * alpha - Pi * alpha)),
            (Pi * alpha * ((-7 * a0 * t11 * (1 + alpha)) / alpha + (3 * pow(a0, 2) * Pi * (1 + alpha)) / (-(Jpeak * Pi) + Jpeak * (-4 + Pi) * alpha) +
                (-6 * v0 + (21 * Jpeak * pow(t11, 2) * (Pi * (-1 + alpha) - 4 * alpha) * (1 + alpha)) / (Pi * pow(alpha, 2)) +
                    (sqrt(3) * sqrt((99 * pow(Jpeak, 4) * Pi * pow(t11, 4) * pow(Pi * (-1 + alpha) - 4 * alpha, 5) * pow(1 + alpha, 2) -
                        16 * pow(a0, 3) * Jpeak * pow(Pi, 2) * t11 * (Pi * (-1 + alpha) - 4 * alpha) * pow(alpha, 3) *
                        (-24 * Pi * (-1 + alpha) * pow(alpha, 2) + 96 * pow(alpha, 3) - 102 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) + pow(Pi, 3) * (-1 + alpha) * (25 + alpha * (49 + 25 * alpha)))
                        + 4 * pow(a0, 4) * pow(Pi, 3) * pow(alpha, 4) *
                        (-24 * Pi * (-1 + alpha) * pow(alpha, 2) + 96 * pow(alpha, 3) - 102 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) + pow(Pi, 3) * (-1 + alpha) * (25 + alpha * (49 + 25 * alpha)))
                        + 12 * pow(Jpeak, 2) * Pi * pow(alpha, 2) * pow(Pi + 4 * alpha - Pi * alpha, 2) *
                        (-2 * a0 * (p0 - pG) * pow(Pi, 2) * (Pi * (-1 + alpha) - 4 * alpha) * pow(alpha, 2) + pow(Pi, 2) * pow(v0, 2) * (Pi * (-1 + alpha) - 4 * alpha) * pow(alpha, 2) +
                            2 * pow(a0, 2) * pow(t11, 2) * (-24 * Pi * (-1 + alpha) * pow(alpha, 2) + 96 * pow(alpha, 3) - 102 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) +
                                pow(Pi, 3) * (-1 + alpha) * (25 + alpha * (49 + 25 * alpha)))) -
                        12 * t11 * alpha * pow(-(Jpeak * Pi) + Jpeak * (-4 + Pi) * alpha, 3) *
                        (-(pow(Pi, 2) * (Pi * (-1 + alpha) - 4 * alpha) * alpha * (2 * (p0 - pG) * alpha + t11 * v0 * (1 + alpha))) +
                            a0 * pow(t11, 2) * (-24 * Pi * (-1 + alpha) * pow(alpha, 2) + 96 * pow(alpha, 3) - 134 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) +
                                pow(Pi, 3) * (-1 + alpha) * (33 + alpha * (65 + 33 * alpha))))) / (pow(Jpeak, 2) * pow(Pi * (-1 + alpha) - 4 * alpha, 3) * pow(alpha, 4)))) / pow(Pi, 1.5)) /
                6.)) / (a0 * Pi * alpha + Jpeak * t11 * (Pi + 4 * alpha - Pi * alpha))};
        temp = isValid(temp);
         t33 = temp.empty()? -1:temp[0];
    }
    return {t1,t2,t3,t4,t11,t22,t33};
};
vector<double> TrigonometricOTG::typeV(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak) {
    double t33 = 0;
    double t1 = Pi * alpha * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    double t2 = Pi * (1 - alpha) * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    vector<double> temp = { (alpha * (a0 * Pi * sqrt(1 + alpha) + sqrt(Pi) * sqrt(pow(a0, 2) * Pi * (1 + alpha) - pow(Apeak, 2) * Pi * (1 + alpha) + 2 * Jpeak * (v0 - Vpeak) * (-Pi + (-4 + Pi) * alpha)))) /
        (Jpeak * (Pi * (-1 + alpha) - 4 * alpha) * sqrt(1 + alpha)), (alpha * (a0 * Pi * sqrt(1 + alpha) -
            sqrt(Pi) * sqrt(pow(a0, 2) * Pi * (1 + alpha) - pow(Apeak, 2) * Pi * (1 + alpha) + 2 * Jpeak * (v0 - Vpeak) * (-Pi + (-4 + Pi) * alpha)))) /
        (Jpeak * (Pi * (-1 + alpha) - 4 * alpha) * sqrt(1 + alpha)) };
    temp = isValid(temp);
    double t11 = temp.empty() ? -1 : temp[0];
    double t22 = (1 - alpha) * t11 / alpha;
    double t3 = ((2 * t1 + t2) * (-2 * Apeak * Pi + 3 * Jpeak * (4 * t1 + Pi * t2)) - Pi * Vpeak) / (Apeak * Pi - 2 * Jpeak * (4 * t1 + Pi * t2));
    double t4 = -((p0 + (3 * pow(Pi, 3) * alpha * (a0 * t11 * (2 * t1 + t11) * pow(1 + alpha, 2) + 2 * alpha * (-(pG * alpha) + t3 * Vpeak * alpha + t11 * v0 * (1 + alpha) + t1 * (v0 + 2 * Vpeak) * (1 + alpha)) +
        Apeak * (pow(t3, 2) * pow(alpha, 2) + 4 * t1 * t3 * alpha * (1 + alpha) + 5 * pow(t1, 2) * pow(1 + alpha, 2))) +
        Jpeak * (-24 * Pi * (pow(t1, 3) - pow(t11, 3)) * (-1 + alpha) * pow(alpha, 2) + 96 * (pow(t1, 3) - pow(t11, 3)) * pow(alpha, 3) -
            6 * pow(Pi, 2) * alpha * (14 * pow(t1, 2) * t3 * alpha * (1 + alpha) + 13 * pow(t1, 3) * pow(1 + alpha, 2) - pow(t11, 3) * pow(1 + alpha, 2) -
                2 * t1 * (-2 * pow(t3, 2) * pow(alpha, 2) + pow(t11, 2) * pow(1 + alpha, 2))) +
            pow(Pi, 3) * (-1 + alpha) * (21 * pow(t1, 2) * t3 * alpha * (1 + alpha) - pow(t11, 3) * (1 + alpha + pow(alpha, 2)) -
                3 * t1 * (-2 * pow(t3, 2) * pow(alpha, 2) + pow(t11, 2) * pow(1 + alpha, 2)) + pow(t1, 3) * (19 + alpha * (37 + 19 * alpha))))) /
        (6. * pow(Pi, 3) * pow(alpha, 3))) / Vpeak);

    return {t1,t2,t3,t4,t11,t22,t33};
};
vector<double> TrigonometricOTG::typeVI(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak) {
    double c0, c1, c2, c3, c4;
    c0 = p0 + pow(v0, 2) / (2. * Apeak) + (2 * Apeak * Pi * v0 * (1 + alpha)) / (Jpeak * (Pi + 4 * alpha - Pi * alpha)) +
        (pow(Apeak, 3) * (96 * Pi * (-1 + alpha) * pow(alpha, 2) - 384 * pow(alpha, 3) - 60 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) + pow(Pi, 3) * (-1 + alpha) * (17 + alpha * (38 + 17 * alpha)))) /
        (24. * pow(Jpeak, 2) * pow(Pi * (-1 + alpha) - 4 * alpha, 3))-pG;

    c1 = -(((1 + alpha) * (2 * a0 * pow(Apeak, 2) * Pi + (a0 + Apeak) * Jpeak * Pi * v0 + 2 * a0 * pow(Apeak, 2) * Pi * alpha - (a0 + Apeak) * Jpeak * (-4 + Pi) * v0 * alpha)) /
        (Apeak * Jpeak * (Pi * (-1 + alpha) - 4 * alpha) * alpha));

    c2 = ((1 + alpha) * (pow(a0, 2) * Pi * (1 + alpha) + a0 * Apeak * Pi * (1 + alpha) + 2 * pow(Apeak, 2) * Pi * (1 + alpha) + Jpeak * v0 * (Pi + 4 * alpha - Pi * alpha))) / (2. * Apeak * Pi * pow(alpha, 2));

    c3 = (Jpeak * (3 * a0 * pow(Pi, 2) * (Pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2) +
        Apeak * (-24 * Pi * (-1 + alpha) * pow(alpha, 2) + 96 * pow(alpha, 3) - 6 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) + pow(Pi, 3) * (-1 + pow(alpha, 3))))) /
        (-6*Apeak * pow(Pi, 3) * pow(alpha, 3));

    c4 = (pow(Jpeak, 2) * pow(1 + alpha, 2) * pow(Pi + 4 * alpha - Pi * alpha, 2)) / (8. * Apeak * pow(Pi, 2) * pow(alpha, 4));
    /*printf("coef: %f %f %f %f %f\n",c0, c1,c2, c3,c4);*/
    vector<double> x(4);
    int idx = SolveP4(x,  c3/c4,  c2/c4,  c1/c4,  c0/c4); // solve equation x^4 + a*x^3 + b*x^2 + c*x + d by Dekart-Euler method
    // x - array of size 4
    // return 4: 4 real roots x[0], x[1], x[2], x[3], possible multiple roots
    // return 2: 2 real roots x[0], x[1] and complex x[2]�i*x[3],
    // return 0: two pair of complex roots: x[0]�i*x[1],  x[2]�i*x[3]
    double t1, t2, t3, t4, t11, t22, t33;
    if (idx == 4)
    {
        x = isValid(x);
        t11 =x.empty()? -1: *max_element(x.begin(), x.end());
    }
    else if(idx==2)
    {
        vector<double> temp = { x[0],x[1] };
        x = isValid(temp);
        t11 = x.empty()? -1:*max_element(x.begin(), x.end());
    }
    else
    {
        t11 = -1;
    }
    /*printf("type VI: idx: %d, ans: %f %f %f %f, t11: %f\n", idx, x[0], x[1], x[2], x[3], t11);*/
    t1 = Pi * alpha * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    t22 = (1 - alpha) * t11 / alpha;
    t2 = (1 - alpha) * t1 / alpha;
    t3 = (2 * v0 + (2 * a0 * t11 * (1 + alpha)) / alpha + (pow(Apeak, 2) * Pi * (1 + alpha)) / (-(Jpeak * Pi) + Jpeak * (-4 + Pi) * alpha) +
        (Jpeak * pow(t11, 2) * (Pi + 4 * alpha - (-4 + Pi) * pow(alpha, 2))) / (Pi * pow(alpha, 2))) / (2. * Apeak);
    t33 = 0;
    t4 = 0;
    return { t1,t2,t3,t4,t11,t22,t33 };
};
vector<double> TrigonometricOTG::typeVII(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak) {
    double t1 = sqrt(Pi * Vpeak / (Jpeak * (4 + Pi * (-1 + pow(alpha, -2)) + 4 / alpha)));
    double t2 = (1 - alpha) * t1 / alpha;
    double t11 = t1 + (a0 * Pi * alpha) / (-(Jpeak * Pi) + Jpeak * (-4 + Pi) * alpha);
    double t22 = (1 - alpha) * t11 / alpha;
    double t3 = 0;
    double t33 = 0;
    double t4 = -((p0 - pG + (a0 * (2 * t11 + t22) * (4 * t1 + 2 * (t11 + t2) + t22)) / 2. +
        (Jpeak * ((96 - 72 * pow(Pi, 2)) * pow(t1, 3) + 24 * (-4 + pow(Pi, 2)) * pow(t11, 3) - 3 * Pi * (-8 + Pi * (24 + 5 * Pi)) * pow(t1, 2) * t2 +
            3 * pow(Pi, 2) * (2 + Pi) * t11 * t22 * (2 * t2 + t22) - pow(Pi, 3) * (t2 - t22) * pow(2 * t2 + t22, 2) +
            3 * Pi * pow(t11, 2) * (8 * Pi * t2 + (-8 + Pi * (8 + Pi)) * t22) +
            3 * pow(Pi, 2) * t1 * (-((6 + 5 * Pi) * pow(t2, 2)) + 2 * (2 * t11 + t22) * (4 * t11 + Pi * t22)))) / (6. * pow(Pi, 3)) +
        (2 * t1 + 2 * t11 + t2 + t22) * v0 + 4 * t1 * Vpeak + 2 * t2 * Vpeak) / Vpeak);

    return {t1,t2,t3,t4,t11,t22,t33};
};
vector<double> TrigonometricOTG::typeVIII(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak) {
    double c0, c1, c2, c3;
    c0 = p0 + (-18 * a0 * Jpeak * Pi * v0 * (1 + alpha) * pow(Pi + 4 * alpha - Pi * alpha, 2) +
        pow(a0, 3) * (24 * Pi * (-1 + alpha) * pow(alpha, 2) - 96 * pow(alpha, 3) - 6 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) + pow(Pi, 3) * (-1 + alpha) * (2 + alpha) * (1 + 2 * alpha))) /
        (6. * pow(Jpeak, 2) * pow(Pi * (-1 + alpha) - 4 * alpha, 3))-pG;

    c1 = (8 * v0 * (1 + alpha) + (pow(a0, 2) * (-24 * Pi * (-1 + alpha) * pow(alpha, 2) + 96 * pow(alpha, 3) + 30 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) +
            pow(Pi, 3) * (8 + alpha * (9 - alpha * (9 + 8 * alpha))))) / (Jpeak * Pi * pow(Pi + 4 * alpha - Pi * alpha, 2))) / (2. * alpha);

    c2 = (a0 * (24 * Pi * (-1 + alpha) * pow(alpha, 2) - 96 * pow(alpha, 3) - 46 * pow(Pi, 2) * alpha * pow(1 + alpha, 2) + pow(Pi, 3) * (-1 + alpha) * (4 + 3 * alpha) * (3 + 4 * alpha))) /
        (2. * pow(Pi, 2) * (Pi * (-1 + alpha) - 4 * alpha) * pow(alpha, 2));

    c3 = (-2 * Jpeak * (Pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2)) / (Pi * pow(alpha, 3));

    vector<double> x(3);
    int idx = SolveP3(x, c2/c3,  c1/c3,  c0/c3);// solve cubic equation x^3 + a*x^2 + b*x + c = 0
    //3 real roots : = > x[0], x[1], x[2], return 3
        //         2 real roots: x[0], x[1],          return 2
        //         1 real root : x[0], x[1] � i*x[2], return 1
    double t1, t2, t3, t4, t11, t22, t33;
    if (idx == 1)
    {
        t11 = x[0];
    }
    else
    {
        x = isValid(x);
        t11 = x.empty()? -1 : *max_element(x.begin(), x.end());
    }
    t1 = t11 + (a0 * Pi * alpha) / (Jpeak * (Pi + 4 * alpha - Pi * alpha));
    t22 = (1 - alpha) * t11 / alpha;
    t2 = (1 - alpha) * t1 / alpha;
    t3 = 0; t33 = 0;
    t4 = 0;
    return {t1,t2,t3,t4,t11,t22,t33};
};
vector<double> TrigonometricOTG::brakeCalculate(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak) {
    int sa = a0 >= 0 ? 1 : -1;
    double t11 = -(a0 * Pi * alpha) / (Jpeak * sa * (-Pi - 4 * alpha + Pi * alpha));
    double t22 = (1 - alpha) * t11 / alpha;
    double vt = (2 * a0 * Pi * (2 * t11 + t22) - Jpeak * sa * (8 * pow(t11, 2) + 2 * (2 * t11 + Pi * t11) * t22 + Pi * pow(t22, 2)) + 2 * Pi * v0) / (2. * Pi);
    double sv = vt >= 0 ? 1: -1;
    double t1 = Pi * alpha * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    double t2 = Pi * (1 - alpha) * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    double vref = Apeak * (2 * t1 + t2);
    double t3;
    if (vref > abs(vt))
    {
        double c0 = v0 + (pow(a0, 2) * Pi * (1 + alpha)) / (2. * Jpeak * sa * (Pi + 4 * alpha - Pi * alpha));
        double c1 = 0;
        double c2 = (Jpeak * sv * (Pi * (-1 + alpha) - 4 * alpha) * (1 + alpha)) / (Pi * pow(alpha, 2));

        // x - array of size 2
        // return 2: 2 real roots x[0], x[1]
        // return 0: pair of complex roots: x[0]�i*x[1]
        vector<double> x(2);
        int idx = SolveP2(x, c1/c2, c0/c2);
        if (idx == 2)
        {
            x = isValid(x);
            t1 =x.empty()? -1: *max_element(x.begin(), x.end());
        }
        else
        {
            t1 = -1;
        }
        t2 = (1 - alpha) * t1 / alpha;
        t3 = 0;
    }
    else
    {
        t3 = (-(pow(a0, 2) * Pi * (1 + alpha)) + 2*sa * (pow(Apeak, 2) * Pi * sv * (1 + alpha) + Jpeak * v0 * (-Pi + (-4 + Pi) * alpha))) / (2. * Apeak * Jpeak * sa * sv * (Pi * (-1 + alpha) - 4 * alpha));
    }
    double vf = a0 * (2 * t11 + t22) - (Jpeak * (sa * (2 * t11 + t22) * (4 * t11 + Pi * t22) + 2 * sv * (4 * t1 + Pi * t2) * (2 * t1 + t2 + t3))) / (2. * Pi) + v0;
    double df = (16 * Jpeak * sa * pow(t11, 3)) / pow(Pi, 3) + (4 * Jpeak * sa * pow(t11, 2) * t22) / pow(Pi, 2) +
        (a0 * (2 * t11 + t22) * (8 * t1 + 2 * t11 + 4 * t2 + t22 + 2 * t3)) / 2. -
        (Jpeak * (2 * sv * t1 * (2 * t1 + t2 + t3) * (4 * t1 + 2 * t2 + t3) + sa * t11 * (2 * t11 + t22) * (8 * t1 + 2 * t11 + 4 * t2 + t22 + 2 * t3))) / Pi -
        (Jpeak * (3 * sv * t2 * (2 * t1 + t2 + t3) * (4 * t1 + 2 * t2 + t3) + sa * t22 *
            (3 * pow(t11, 2) + 12 * t1 * (2 * t11 + t22) + 3 * t11 * (4 * t2 + t22 + 2 * t3) + t22 * (6 * t2 + t22 + 3 * t3)))) / 6. +
        (4 * t1 + 2 * t11 + 2 * t2 + t22 + t3) * v0;
    return {t1,t2,t3,t11,t22,df};
};
vector<double> TrigonometricOTG::isValid(vector<double> candidate) {
    int n = candidate.size();
    vector<double> ans;
    if (n == 0) return ans;
    for (int i = 0; i < n; i++)
    {
        if (candidate[i] >= 0) ans.push_back(candidate[i]);
    }
    return ans;
};
vector<double> TrigonometricOTG::filterResults(vector<vector<double>>& candidates, double v0, double a0, double p0, double pG, double Jpeak, double Apeak, double Vpeak) {
    int n = candidates.size();
    double t1, t2, t3, t4, t11, t22, t33;
    double dur = INT16_MAX;
    vector<double> ans;
    for (int i = 0; i < n; i++)
    {
        vector<double> candidate = candidates[i];
        /*printf("cand: %f %f %f %f %f %f %f\n", candidate[0], candidate[1], candidate[2], candidate[3], candidate[4], candidate[5], candidate[6]);*/
        candidate = isValid(candidate);
        if (candidate.size() < 7) continue;
        t1 = candidate[0];
        t2 = candidate[1];
        t3 = candidate[2];
        t4 = candidate[3];
        t11 = candidate[4];
        t22 = candidate[5];
        t33 = candidate[6];
        /*printf("%f %f %f %f %f %f %f\n", t1, t2, t3, t4, t11, t22, t33);*/
        double mA1 = 4 * Jpeak * t1 / Pi + Jpeak * t2;
        double mA2 = a0 + 4 * Jpeak * t11 / Pi + Jpeak * t22;
        if (abs(mA1-mA2) > eps) continue;
        double mA = mA1;
        double mV = (Jpeak * (-8 * pow(t1, 2) + 8 * pow(t11, 2) - 2 * (2 * t1 + Pi * t1) * t2 - Pi * pow(t2, 2) + 2 * (2 + Pi) * t11 * t22 + Pi * pow(t22, 2)) +
            2 * mA * Pi * (2 * t1 + t2 + t33) + 2 * Pi * (2 * a0 * t11 + a0 * t22 + v0)) / (2. * Pi);
        if ((abs(mV) > abs(Vpeak) && abs(mV-Vpeak)>eps) || (abs(mA) > abs(Apeak)&&abs(mA-Apeak)>eps)) continue;
        double vf = mV + mA * (4 * t1 + 2 * t2 + t3) - (Jpeak * (4 * t1 + Pi * t2) * (6 * t1 + 3 * t2 + 2 * t3)) / Pi;
        double df = p0 + (3 * a0 * (2 * t11 + t22) * (4 * t1 + 2 * t11 + 2 * t2 + t22 + 2 * t33) +
            3 * mA * (20 * pow(t1, 2) + 5 * pow(t2, 2) + pow(t3, 2) + pow(t33, 2) + 2 * t2 * (2 * t3 + t33) + 4 * t1 * (5 * t2 + 2 * t3 + t33)) +
            (Jpeak * ((96 - 312 * pow(Pi, 2)) * pow(t1, 3) + 24 * (-4 + pow(Pi, 2)) * pow(t11, 3) -
                3 * Pi * pow(t1, 2) * ((-8 + Pi * (104 + 25 * Pi)) * t2 + 56 * Pi * t3) +
                3 * pow(Pi, 2) * t1 * (2 * (2 * t11 + t22) * (4 * t11 + Pi * t22) - 14 * Pi * t2 * t3 - 8 * pow(t3, 2) - t2 * ((26 + 25 * Pi) * t2 + 28 * t3)) +
                3 * pow(Pi, 2) * (2 + Pi) * t11 * t22 * (2 * t2 + t22 + 2 * t33) + 3 * Pi * pow(t11, 2) * (-8 * t22 + pow(Pi, 2) * t22 + 8 * Pi * (t2 + t22 + t33)) +
                pow(Pi, 3) * (-19 * pow(t2, 3) - 21 * pow(t2, 2) * t3 + 3 * t2 * (pow(t22, 2) - 2 * pow(t3, 2)) + pow(t22, 2) * (t22 + 3 * t33)))) / pow(Pi, 3)\
            + 6 * mV * (4 * t1 + 2 * t2 + t3 + t4) + 6 * (2 * t1 + 2 * t11 + t2 + t22 + t33) * v0) / 6.;
        double af = mA - 4 * Jpeak * t1 / Pi - Jpeak * t2;
        if (abs(vf) > eps || abs(af) > eps || abs(df - pG) > eps) continue;
        double total = accumulate(candidate.begin(), candidate.end(), 0.0);
        if (dur > total)
        {
            dur = total;
            ans = candidate;
        }
    }
    return ans;
};
vector<double> TrigonometricOTG::profileGenerator(double v0, double a0, double p0, double pG, double alpha, double t1, double t2, double t3, double t4, double t11, double t22, double t33, double Jpeak, double Apeak, double Vpeak,double t) {
    double temp;
    double dur1 = t11 + t22 + t11 + t33;
    double dur2 = t1 + t2 + t1;
    vector<double> ans;
    profileSeg prof(v0, a0, p0, pG, alpha, t1, t2, t3, t4, t11, t22, t33, Jpeak, Apeak, Vpeak);
    if (t < t11)
    {
        ans = prof.Seg1(t);
    }
    else if (t>=t11 && t<t11+t22)
    {
        temp = t - t11;
        ans = prof.Seg2(temp);
    }
    else if (t >= t11+t22 && t < 2*t11 + t22)
    {
        temp = t - t11-t22;
        ans = prof.Seg3(temp);
    }
    else if (t >= 2*t11+t22 && t < dur1)
    {
        temp = t - 2*t11-t22;
        ans = prof.Seg4(temp);
    }
    else if (t >= dur1 && t < dur1 + t1)
    {
        temp = t - dur1;
        ans = prof.Seg5(temp);
    }
    else if (t >= dur1+t1 && t < dur1 + t1 + t2)
    {
        temp = t - dur1-t1;
        ans = prof.Seg6(temp);
    }
    else if (t >= dur1 + t1+t2 && t < dur1 + dur2)
    {
        temp = t - dur1 - t1-t2;
        ans = prof.Seg7(temp);
    }
    else if (t >= dur1 + dur2 && t < dur1 + dur2+t4)
    {
        temp = t - dur1 - dur2;
        ans = prof.Seg8(temp);
    }
    else if (t >= dur1 +dur2+t4 && t < dur1 + dur2+t4+t1)
    {
        temp = t - dur1 - dur2-t4;
        ans = prof.Seg9(temp);
    }
    else if (t >= dur1 + dur2 + t4 +t1 && t < dur1 + dur2 + t4 + t1+t2)
    {
        temp = t - dur1 - dur2 - t4-t1;
        ans = prof.Seg10(temp);
    }
    else if (t >= dur1 + dur2 + t4+t1+t2 && t < dur1 + 2*dur2 + t4)
    {
        temp = t - dur1 - dur2 - t4-t1-t2;
        ans = prof.Seg11(temp);
    }
    else if (t >= dur1 + 2 * dur2 + t4 && t < dur1 + 2 * dur2 + t4+t3)
    {
        temp = t - dur1 - 2*dur2 - t4;
        ans = prof.Seg12(temp);
    }
    else if (t >= dur1 + 2 * dur2 + t4+t3 && t < dur1 + 2 * dur2 + t4 + t3+t1)
    {
        temp = t - dur1 - 2 * dur2 - t4-t3;
        ans = prof.Seg13(temp);
    }
    else if (t >= dur1 + 2 * dur2 + t4 + t3+t1 && t < dur1 + 2 * dur2 + t4 + t3 + t1+t2)
    {
        temp = t - dur1 - 2 * dur2 - t4 - t3-t1;
        ans = prof.Seg14(temp);
    }
    else if (t >= dur1 + 2 * dur2 + t4 + t3 + t1+t2 && t < dur1 + 3 * dur2 + t4 + t3 )
    {
        temp = t - dur1 - 2 * dur2 - t4 - t3 - t1-t2;
        ans = prof.Seg15(temp);
    }
    else
    {
        temp = t - dur1 - 3 * dur2 - t4 - t3;
        ans = prof.Seg16(temp);
    }
    return ans;
};
vector<double> TrigonometricOTG::profileGeneratorB(double v0, double a0, double p0, double pG, double alpha, double t1, double t2, double t3, double t11, double t22, double Jpeak, double Apeak, double Vpeak,double t) {
    double temp;
    double dur1 = t11 + t22 + t11;
    double dur2 = t1 + t2 + t1;
    vector<double> ans;
    profileSegB prof(v0, a0, p0, pG, alpha, t1, t2, t3, t11, t22, Jpeak, Apeak, Vpeak);
    if (t < t11)
    {
        ans = prof.Seg1(t);
    }
    else if (t >= t11 && t < t11 + t22)
    {
        temp = t - t11;
        ans = prof.Seg2(temp);
    }
    else if (t >= t11 + t22 && t < dur1)
    {
        temp = t - t11 - t22;
        ans = prof.Seg3(temp);
    }
    else if (t >= dur1 && t < dur1+t1)
    {
        temp = t -dur1;
        ans = prof.Seg9(temp);
    }
    else if (t >= dur1+t1 && t < dur1 + t1+t2)
    {
        temp = t - dur1-t1;
        ans = prof.Seg10(temp);
    }
    else if (t >= dur1 + t1+t2 && t < dur1 + dur2)
    {
        temp = t - dur1 - t1-t2;
        ans = prof.Seg11(temp);
    }
    else if (t >= dur1 + dur2 && t < dur1 + dur2+t3)
    {
        temp = t - dur1 - dur2;
        ans = prof.Seg12(temp);
    }
    else if (t >= dur1 + dur2+t3 && t < dur1 + dur2 + t3+t1)
    {
        temp = t - dur1 - dur2-t3;
        ans = prof.Seg13(temp);
    }
    else if (t >= dur1 + dur2 + t3+t1 && t < dur1 + dur2 + t3+t1+t2)
    {
        temp = t - dur1 - dur2 - t3-t1;
        ans = prof.Seg14(temp);
    }
    else if (t >= dur1 + dur2 + t3 + t1+t2 && t < dur1 + 2*dur2 + t3)
    {
        temp = t - dur1 - dur2 - t3-t1 - t2;
        ans = prof.Seg15(temp);
    }
    else
    {
        temp = t - dur1 - 2 * dur2 - t3;
        ans = prof.Seg16(temp);
    }
    return ans;
};
vector<vector<double>> TrigonometricOTG::trajGeneratorB(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak) {
    vector<double> res = brakeCalculate(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
    vector<vector<double>> ans;
    double t1b = res[0];
    double t2b = res[1];
    double t3b = res[2];
    double t11b = res[3];
    double t22b = res[4];
    double dist = res[5];
    double pt = p0 + dist;
    double dur1 = 2 * t11b + t22b;
    double dur2 = t1b + t2b + t1b;
    double total = dur1 + 2 * dur2 + t3b;
    double t = 0;
    while (t <= total)
    {
        vector<double> temp = profileGeneratorB(v0, a0, p0, pt, alpha, t1b, t2b, t3b, t11b, t22b, Jpeak, Apeak, Vpeak,t);
        temp.insert(temp.begin(), t);
        ans.push_back(temp);
        t += rate_;
    }

    return ans;
};
vector<vector<double>> TrigonometricOTG::trajGeneratorT(double v0, double a0, double p0, double pG, double alpha, double maxJ, double maxA, double maxV, double duration) {
    double Vpeak=maxV, Apeak=maxA, Jpeak=maxJ;
    double t1, t2, t3, t4, t11, t22, t33;
    vector<vector<double>> ans;
    if (pG - p0 < 0) {
        Vpeak = -Vpeak;
        Apeak = -Apeak;
        Jpeak = -Jpeak;
    }
    t1 = Pi * alpha * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    t2 = Pi * (1-alpha) * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    t11 = Pi * alpha * (Apeak-a0) / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    t22 = Pi * (1 - alpha) * (Apeak-a0) / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    t3 = ((2 * t1 + t2) * (-2 * Apeak * Pi + 3 * Jpeak * (4 * t1 + Pi * t2)) - Pi * Vpeak) / (Apeak * Pi - 2 * Jpeak * (4 * t1 + Pi * t2));
    t33 = (-2 * Apeak * Pi * (2 * t1 + t2) + Jpeak * (8 * pow(t1, 2) + 2 * (2 + Pi) * t1 * t2 + Pi * pow(t2, 2) - (2 * t11 + t22) * (4 * t11 + Pi * t22)) -
        2 * Pi * (a0 * (2 * t11 + t22) + v0 - Vpeak)) / (2. * Apeak * Pi);
    t4 = typeI(v0,a0,p0,pG,alpha,t1,t2,t3,t11,t22,t33,Jpeak,Apeak,Vpeak);
    vector<double> cand1 = { t1,t2,t3,t4,t11,t22,t33 };
    vector<double> temp = typeII(v0, a0, p0, pG, alpha, t1, t2, t11, t22, Jpeak, Apeak, Vpeak);
    vector<double> cand2 = { t1,t2,temp[0],temp[1],t11,t22,temp[2] };
    vector<double> cand3 = typeIII(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
    vector<double> cand4 = typeIV(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
    vector<double> cand5 = typeV(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
    vector<double> cand6 = typeVI(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
    vector<double> cand7 = typeVII(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
    vector<double> cand8 = typeVIII(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
    vector<vector<double>> candidates = { cand1,cand2,cand3,cand4,cand5, cand6, cand7, cand8 };
    vector<double> res = filterResults(candidates, v0, a0, p0, pG, Jpeak, Apeak, Vpeak);
    //cout << res.size()<<endl;
    if (!res.empty())
    {
        t1 = res[0];
        t2 = res[1];
        t3 = res[2];
        t4 = res[3];
        t11 = res[4];
        t22 = res[5];
        t33 = res[6];
        //printf("no brake: %f %f %f %f %f %f %f\n", t1, t2, t3, t4, t11, t22, t33);
        double dur1 = 2 * t11 + t22 + t33;
        double dur2 = t1 + t2 + t1;
        double total = dur1 + 3 * dur2 + t3 + t4;
        double t = 0;
        while (t <= minT_)
        {
            vector<double> temp = profileGenerator(v0,a0,p0,pG,alpha,t1,t2,t3,t4,t11,t22,t33,Jpeak,Apeak,Vpeak,t);
            temp.insert(temp.begin(), t);
            ans.push_back(temp);
            t += rate_;
        }
    }
    else
    {
        vector<double> brakeresult = brakeCalculate(v0, a0, p0, pG, alpha, maxJ, maxA, maxV);
        /*cout << brakeresult.size()<<endl;*/
        double t1b = brakeresult[0];
        double t2b = brakeresult[1];
        double t3b = brakeresult[2];
        double t11b = brakeresult[3];
        double t22b = brakeresult[4];
        double dist = brakeresult[5];
        double pt = p0 + dist;
        double dur1 = 2 * t11b + t22b;
        double dur2 = t1b + t2b + t1b;
        double total = dur1 + 2 * dur2 + t3b;
        double t = 0;
        //printf("brake: %f %f %f %f %f\n", t1b, t2b, t3b, t11b, t22b);
        while (t < total)
        {
            vector<double> temp = profileGeneratorB(v0, a0, p0, pt, alpha, t1b, t2b, t3b, t11b, t22b, maxJ, maxA, maxV,t);
            temp.insert(temp.begin(), t);
            ans.push_back(temp);
            t += rate_;
        }
       /* printf("%f %f", total, t);*/
        vector<vector<double>> profileZ = trajGeneratorZ(pt, pG, alpha, maxJ, maxA, maxV,t-total, total,minT_);
        ans.insert(ans.end(),profileZ.begin(), profileZ.end());
    }
    return ans;
};
vector<vector<double>> TrigonometricOTG::trajGeneratorZ(double p0, double pG, double alpha, double maxJ, double maxA, double maxV, double t, double pret,double duration) {
    double Vpeak = maxV, Apeak = maxA, Jpeak = maxJ;
    double t1, t2, t3, t4;
    vector<vector<double>> ans;
    double D = abs(pG - p0);
    t1 = Pi * alpha * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    t2 = Pi * (1 - alpha) * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    t3 = Vpeak / Apeak - (2 * t1 + t2);
    t4 = D / Vpeak - (4 * t1 + 2 * t2 + t3);
    double Varef = Apeak * (2 * t1 + t2);

    if (Vpeak >= Varef)
    {
        double Dvref2 = Vpeak * (4 * t1 + 2 * t2 + t3);
        if (D >= Dvref2)
        {
            /*printf("Type I");*/
            t1 = Pi * alpha * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
            t2 = Pi * (1 - alpha) * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
            t3 = Vpeak / Apeak - (2 * t1 + t2);
            t4 = D / Vpeak - (4 * t1 + 2 * t2 + t3);
        }
        else
        {
            double Daref = Varef * (4 * t1 + 2 * t2);
            if (D < Daref)
            {
                /*printf("Type IV, t3=t4=0\n");*/
                double temp = -0.5 * (D * Pi * pow(alpha, 3)) / (Jpeak * (Pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2));
                t1 = pow(temp, 0.33333333333333333333);
                t2 = (1 - alpha) * t1 / alpha;
                t3 = 0;
                t4 = 0;
            }
            else
            {
                /*printf("Type II t4 =0;");*/
                t1 = Pi * alpha * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
                t2 = Pi * (1 - alpha) * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
                t4 = 0;
                t3 = -3 * t1 + sqrt(D / Apeak + pow(t1 + 0.5 * t2, 2)) - 1.5 * t2;
            }
        }
    }
    else
    {
        t1 = sqrt(Pi * Vpeak / (Jpeak * (4 + Pi * (-1 + pow(alpha, -2)) + 4 / alpha)));
        t2 = (1 - alpha) * t1 / alpha;
        t4 = D / Vpeak - (4 * t1 + 2 * t2);
        double Dvref1 = Vpeak * (4 * t1 + 2 * t2);
        if (D >= Dvref1)
        {
            /*printf("Type III t3 =0 ");*/
            t3 = 0;
        }
        else
        {
            /*printf("Type IV t3=0 t4=0");*/
            double temp = -0.5 * (D * Pi * pow(alpha, 3)) / (Jpeak * (Pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2));
            t1 = pow(temp, 0.33333333333333333333);
            t2 = (1 - alpha) * t1 / alpha;
            t3 = 0;
            t4 = 0;
        }
    }

    if (pG - p0 < 0)
    {
        Jpeak = -Jpeak;
    }
    //printf("zTraj: %f %f %f %f\n", t1, t2, t3, t4);
    double total = 8 * t1 + 4 * t2 + 2 * t3 + t4;
    while (t <= minT_-pret)
    {
        vector<double> temp = profileGenerator(0, 0, p0, pG, alpha, t1, t2, t3, t4, t1, t2, t3, Jpeak, Apeak, Vpeak,t);
        temp.insert(temp.begin(), t+pret);
        ans.push_back(temp);
        t += rate_;
    }

    return ans;
};
double TrigonometricOTG::minimumTime(){
  for (int i = 0; i < num_dof_; i++)
  {
      double v0 = v0_[i];
      double a0 = a0_[i];
      double p0 = p0_[i];
      double pG = pG_[i];
      double Jpeak = mJ_[i];
      double Vpeak = mV_[i];
      double Apeak = mA_[i];
      double alpha = alpha_[i];

      vector<double> temp = trajTimeT(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);

      if(temp.size()==11)
      {
        //printf("TypeII\n");
        trajTimes_[i].twoPiece = vector<double>(temp.begin(),temp.end()-2);
        trajTimes_[i].brake = true;
        trajTimes_[i].dist = temp[9];

      }
      else
      {
        //printf("TypeI\n");
        trajTimes_[i].onePiece=vector<double>(temp.begin(),temp.end()-1);
        trajTimes_[i].brake = false;
      }
      minT_ = max(minT_, temp.back());
  }
  return minT_;
};

vector<vector<vector<double>>> TrigonometricOTG::trajGenerator(){
    minimumTime();
    //printf("duration: %f \n", minT_);
    vector<vector<vector<double>>> ans;
    for (int i = 0; i < num_dof_; i++)
    {
        double v0 = v0_[i];
        double a0 = a0_[i];
        double p0 = p0_[i];
        double pG = pG_[i];
        double Jpeak = mJ_[i];
        double Vpeak = mV_[i];
        double Apeak = mA_[i];
        double alpha = alpha_[i];
        double t = 0;
        vector<vector<double>> res;
        if(!trajTimes_[i].brake)
        {
          if (pG - p0 < 0) {
              Vpeak = -Vpeak;
              Apeak = -Apeak;
              Jpeak = -Jpeak;
          }
          double t1 = trajTimes_[i].onePiece[0], t2 = trajTimes_[i].onePiece[1], t3 = trajTimes_[i].onePiece[2], t4 = trajTimes_[i].onePiece[3], t11 = trajTimes_[i].onePiece[4], t22 = trajTimes_[i].onePiece[5], t33 = trajTimes_[i].onePiece[6];
          while (t <= minT_)
          {
              vector<double> temp = profileGenerator(v0,a0,p0,pG,alpha,t1,t2,t3,t4,t11,t22,t33,Jpeak,Apeak,Vpeak,t);
              temp.insert(temp.begin(), t);
              res.push_back(temp);
              t += rate_;
          }
        }
        else
        {
          double t1b = trajTimes_[i].twoPiece[0];
          double t2b = trajTimes_[i].twoPiece[1];
          double t3b = trajTimes_[i].twoPiece[2];
          double t11b = trajTimes_[i].twoPiece[3];
          double t22b = trajTimes_[i].twoPiece[4];
          double dist = trajTimes_[i].dist;
          double pt = p0 + dist;
          double dur1 = 2 * t11b + t22b;
          double dur2 = t1b + t2b + t1b;
          double total = dur1 + 2 * dur2 + t3b;
          //printf("brake: %f %f %f %f %f\n", t1b, t2b, t3b, t11b, t22b);
          while (t < total)
          {
              vector<double> temp = profileGeneratorB(v0, a0, p0, pt, alpha, t1b, t2b, t3b, t11b, t22b, Jpeak, Apeak, Vpeak,t);
              temp.insert(temp.begin(), t);
              res.push_back(temp);
              t += rate_;
          }
         /* printf("%f %f", total, t);*/
          vector<vector<double>> profileZ;
          double t1 = trajTimes_[i].twoPiece[5];
          double t2 = trajTimes_[i].twoPiece[6];
          double t3 = trajTimes_[i].twoPiece[7];
          double t4 = trajTimes_[i].twoPiece[8];
          double pret = total;
          t = t-pret;
          if (pG - pt < 0)
          {
              Jpeak = -Jpeak;
          }
          total = 8 * t1 + 4 * t2 + 2 * t3 + t4;
          while (t <= minT_-pret)
          {
              vector<double> temp = profileGenerator(0, 0, pt, pG, alpha, t1, t2, t3, t4, t1, t2, t3, Jpeak, Apeak, Vpeak,t);
              temp.insert(temp.begin(), t+pret);
              profileZ.push_back(temp);
              t += rate_;
          }
          res.insert(res.end(),profileZ.begin(), profileZ.end());
        }
        ans.push_back(res);
    }

    return ans;
}
vector<double> TrigonometricOTG::trajTimeB(double v0, double a0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak) {
    vector<double> res = brakeCalculate(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
    vector<double> ans;
    double t1b = res[0];
    double t2b = res[1];
    double t3b = res[2];
    double t11b = res[3];
    double t22b = res[4];
    double dist = res[5];
    return {t1b,t2b,t3b,t11b,t22b};
};
vector<double> TrigonometricOTG::trajTimeT(double v0, double a0, double p0, double pG, double alpha, double maxJ, double maxA, double maxV) {
    double Vpeak = maxV, Apeak = maxA, Jpeak = maxJ;
    double t1, t2, t3, t4, t11, t22, t33;
    vector<double> ans;
    if (pG - p0 < 0) {
        Vpeak = -Vpeak;
        Apeak = -Apeak;
        Jpeak = -Jpeak;
    }
    t1 = Pi * alpha * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    t2 = Pi * (1 - alpha) * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    t11 = Pi * alpha * (Apeak - a0) / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    t22 = Pi * (1 - alpha) * (Apeak - a0) / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    t3 = ((2 * t1 + t2) * (-2 * Apeak * Pi + 3 * Jpeak * (4 * t1 + Pi * t2)) - Pi * Vpeak) / (Apeak * Pi - 2 * Jpeak * (4 * t1 + Pi * t2));
    t33 = (-2 * Apeak * Pi * (2 * t1 + t2) + Jpeak * (8 * pow(t1, 2) + 2 * (2 + Pi) * t1 * t2 + Pi * pow(t2, 2) - (2 * t11 + t22) * (4 * t11 + Pi * t22)) -
        2 * Pi * (a0 * (2 * t11 + t22) + v0 - Vpeak)) / (2. * Apeak * Pi);
    t4 = typeI(v0, a0, p0, pG, alpha, t1, t2, t3, t11, t22, t33, Jpeak, Apeak, Vpeak);
    vector<double> cand1 = { t1,t2,t3,t4,t11,t22,t33 };
    vector<double> temp = typeII(v0, a0, p0, pG, alpha, t1, t2, t11, t22, Jpeak, Apeak, Vpeak);
    vector<double> cand2 = { t1,t2,temp[0],temp[1],t11,t22,temp[2] };
    vector<double> cand3 = typeIII(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
    vector<double> cand4 = typeIV(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
    vector<double> cand5 = typeV(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
    vector<double> cand6 = typeVI(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
    vector<double> cand7 = typeVII(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
    vector<double> cand8 = typeVIII(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
    vector<vector<double>> candidates = { cand1,cand2,cand3,cand4,cand5, cand6, cand7, cand8 };
    vector<double> res = filterResults(candidates, v0, a0, p0, pG, Jpeak, Apeak, Vpeak);
    //cout << res.size()<<endl;
    if (!res.empty())
    {
        t1 = res[0];
        t2 = res[1];
        t3 = res[2];
        t4 = res[3];
        t11 = res[4];
        t22 = res[5];
        t33 = res[6];
        double dur1 = 2 * t11 + t22 + t33;
        double dur2 = t1 + t2 + t1;
        double total = dur1 + 3 * dur2 + t3 + t4;
        ans = { t1,t2,t3,t4,t11,t22,t33, total};
    }
    else
    {
        vector<double> brakeresult = brakeCalculate(v0, a0, p0, pG, alpha, maxJ, maxA, maxV);
        /*cout << brakeresult.size()<<endl;*/
        double t1b = brakeresult[0];
        double t2b = brakeresult[1];
        double t3b = brakeresult[2];
        double t11b = brakeresult[3];
        double t22b = brakeresult[4];
        double dist = brakeresult[5];
        double pt = p0 + dist;
        /*printf("brake: %f %f %f %f %f %f\n", t1b, t2b, t3b, t11b, t22b);*/
        double dur1 = 2 * t11b + t22b;
        double dur2 = t1b + t2b + t1b;
        double total = dur1 + 2 * dur2 + t3b;
        ans = { t1b,t2b,t3b,t11b,t22b };
        vector<double> timeZ = trajTimeZ(pt, pG, alpha, maxJ, maxA, maxV);
        total += (8*timeZ[0] + 4*timeZ[1]+ 2*timeZ[2]+timeZ[3]);
        ans.insert(ans.end(), timeZ.begin(), timeZ.end());
        ans.push_back(dist);
        ans.push_back(total);
    }
    return ans;
};
vector<double> TrigonometricOTG::trajTimeZ(double p0, double pG, double alpha, double maxJ, double maxA, double maxV) {
    double Vpeak = maxV, Apeak = maxA, Jpeak = maxJ;
    double t1, t2, t3, t4;
    vector<vector<double>> ans;
    double D = abs(pG - p0);
    t1 = Pi * alpha * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    t2 = Pi * (1 - alpha) * Apeak / ((4 * alpha + Pi - Pi * alpha) * Jpeak);
    t3 = Vpeak / Apeak - (2 * t1 + t2);
    t4 = D / Vpeak - (4 * t1 + 2 * t2 + t3);
    double Varef = Apeak * (2 * t1 + t2);
    if (Vpeak >= Varef)
    {
        double Dvref2 = Vpeak * (4 * t1 + 2 * t2 + t3);
        if (D < Dvref2)
        {
            double Daref = Varef * (4 * t1 + 2 * t2);
            if (D < Daref)
            {
                double temp = -0.5 * (D * Pi * pow(alpha, 3)) / (Jpeak * (Pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2));
                t1 = pow(temp, 0.33333333333333333333);
                t2 = (1 - alpha) * t1 / alpha;
                t3 = 0;
                t4 = 0;
            }
            else
            {
                t4 = 0;
                t3 = -3 * t1 + sqrt(D / Apeak + pow(t1 + 0.5 * t2, 2)) - 1.5 * t2;
            }
        }
    }
    else
    {
        t1 = sqrt(Pi * Vpeak / (Jpeak * (4 + Pi * (-1 + pow(alpha, -2)) + 4 / alpha)));
        t2 = (1 - alpha) * t1 / alpha;
        t4 = D / Vpeak - (4 * t1 + 2 * t2);
        double Dvref1 = Vpeak * (4 * t1 + 2 * t2);
        if (D >= Dvref1)
        {
            t3 = 0;
        }
        else
        {
            double temp = -0.5 * (D * Pi * pow(alpha, 3)) / (Jpeak * (Pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2));
            t1 = pow(temp, 0.33333333333333333333);
            t2 = (1 - alpha) * t1 / alpha;
            t3 = 0;
            t4 = 0;
        }
    }

    /*printf("zTraj: %f %f %f %f", t1, t2, t3, t4);*/

    return {t1,t2,t3,t4};
};
