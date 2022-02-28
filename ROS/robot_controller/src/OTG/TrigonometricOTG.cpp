#include<TrigonometricOTG.h>
# define pi           3.14159265358979323846
#define eps           0.00001
TrigonometricOTG::TrigonometricOTG() {};

TrigonometricOTG::TrigonometricOTG(int num_dof, double rate) : num_dof_(num_dof), mJ_(vector<double>(num_dof)), mA_(vector<double>(num_dof)), mV_(vector<double>(num_dof)), rate_(rate),trajTimes_(vector<Times>(num_dof_)) { complete_ = false; minT_ = 0; idx_=0;};

TrigonometricOTG::TrigonometricOTG(int num_dof, double mJ, double mA, double mV, double rate) :num_dof_(num_dof), mJ_(vector<double>(num_dof, mJ)), mA_(vector<double>(num_dof, mA)), mV_(vector<double>(num_dof, mV)), rate_(rate),trajTimes_(vector<Times>(num_dof_)) { complete_ = false; minT_ = 0;idx_=0;};

TrigonometricOTG::TrigonometricOTG(int num_dof, vector<double> mJ, vector<double> mA, vector<double> mV, vector<double> a0, vector<double> v0, vector<double> p0, vector<double> pG, vector<double> alpha, double rate) : num_dof_(num_dof), mJ_(mJ), mA_(mA), mV_(mV), a0_(a0), v0_(v0), p0_(p0), pG_(pG), alpha_(alpha), rate_(rate),trajTimes_(vector<Times>(num_dof_)) { complete_ = false; minT_ = 0;idx_=0;};

TrigonometricOTG::TrigonometricOTG(int num_dof, double mJ, double mA, double mV, vector<double> a0, vector<double> v0, vector<double> p0, vector<double> pG, double alpha, double rate):num_dof_(num_dof), mJ_(vector<double>(num_dof, mJ)), mA_(vector<double>(num_dof, mA)), mV_(vector<double>(num_dof, mV)), a0_(a0), v0_(v0), p0_(p0), pG_(pG), alpha_(vector<double>(num_dof, alpha)), rate_(rate),trajTimes_(vector<Times>(num_dof_)) { complete_ = false; minT_ = 0;idx_=0;};

TrigonometricOTG::~TrigonometricOTG() {};

vector<vector<double>> TrigonometricOTG::typeI(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV) {
    vector<vector<double>> ans;
    vector<vector<double>> res1 = computeTypeI(a0, v0, p0, pG, alpha, mJ, mA, mV, 1, -1);
    vector<vector<double>> res2 = computeTypeI(a0, v0, p0, pG, alpha, mJ, mA, mV, -1, 1);
    vector<vector<double>> res3 = computeTypeI(a0, v0, p0, pG, alpha, mJ, mA, mV, 1, 1);

    ans.insert(ans.end(), res1.begin(), res1.end());
    ans.insert(ans.end(), res2.begin(), res2.end());
    ans.insert(ans.end(), res3.begin(), res3.end());

    return ans;
};
vector<vector<double>> TrigonometricOTG::typeII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV) {
    vector<vector<double>> ans;
    vector<vector<double>> res1 = computeTypeII(a0, v0, p0, pG, alpha, mJ, mA, mV, 1, -1);
    vector<vector<double>> res2 = computeTypeII(a0, v0, p0, pG, alpha, mJ, mA, mV, -1, 1);

    ans.insert(ans.end(), res1.begin(), res1.end());
    ans.insert(ans.end(), res2.begin(), res2.end());

    return ans;
};
vector<vector<double>> TrigonometricOTG::typeIII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV) {
    vector<vector<double>> ans;
    vector<vector<double>> res1 = computeTypeIII(a0, v0, p0, pG, alpha, mJ, mA, mV, 1, -1);
    vector<vector<double>> res2 = computeTypeIII(a0, v0, p0, pG, alpha, mJ, mA, mV, -1, 1);

    ans.insert(ans.end(), res1.begin(), res1.end());
    ans.insert(ans.end(), res2.begin(), res2.end());

    return ans;
};
vector<vector<double>> TrigonometricOTG::typeIV(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV) {
    vector<vector<double>> ans;
    vector<vector<double>> res1 = computeTypeIV(a0, v0, p0, pG, alpha, mJ, mA, mV, 1, -1);
    vector<vector<double>> res2 = computeTypeIV(a0, v0, p0, pG, alpha, mJ, mA, mV, -1, 1);

    ans.insert(ans.end(), res1.begin(), res1.end());
    ans.insert(ans.end(), res2.begin(), res2.end());

    return ans;
};
vector<vector<double>> TrigonometricOTG::typeV(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV) {
    vector<vector<double>> ans;
    vector<vector<double>> res1 = computeTypeV(a0, v0, p0, pG, alpha, mJ, mA, mV, 1, -1);
    vector<vector<double>> res2 = computeTypeV(a0, v0, p0, pG, alpha, mJ, mA, mV, -1, 1);

    ans.insert(ans.end(), res1.begin(), res1.end());
    ans.insert(ans.end(), res2.begin(), res2.end());

    return ans;
};
vector<vector<double>> TrigonometricOTG::typeVI(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV) {
    vector<vector<double>> ans;
    vector<vector<double>> res1 = computeTypeVI(a0, v0, p0, pG, alpha, mJ, mA, mV, 1, -1);
    vector<vector<double>> res2 = computeTypeVI(a0, v0, p0, pG, alpha, mJ, mA, mV, -1, 1);

    ans.insert(ans.end(), res1.begin(), res1.end());
    ans.insert(ans.end(), res2.begin(), res2.end());

    return ans;
};
vector<vector<double>> TrigonometricOTG::typeVII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV) {
    vector<vector<double>> ans;
    vector<vector<double>> res1 = computeTypeVII(a0, v0, p0, pG, alpha, mJ, mA, mV, 1, -1);
    vector<vector<double>> res2 = computeTypeVII(a0, v0, p0, pG, alpha, mJ, mA, mV, -1, 1);

    ans.insert(ans.end(), res1.begin(), res1.end());
    ans.insert(ans.end(), res2.begin(), res2.end());

    return ans;
};
vector<vector<double>> TrigonometricOTG::typeVIII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV) {
    vector<vector<double>> ans;
    vector<vector<double>> res1 = computeTypeVIII(a0, v0, p0, pG, alpha, mJ, mA, mV, 1, -1);
    vector<vector<double>> res2 = computeTypeVIII(a0, v0, p0, pG, alpha, mJ, mA, mV, -1, 1);

    ans.insert(ans.end(), res1.begin(), res1.end());
    ans.insert(ans.end(), res2.begin(), res2.end());

    return ans;
};

vector<vector<double>> TrigonometricOTG::computeTypeI(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv) {
    /*cout << "I" << endl;*/
    double Jpeak = mJ;
    double ta3 = 0;
    double tc3 = 0;
    vector<vector<double>> ans = {};

    //find coefficient for final velocity equation
    double c0, c1, c2, c3;
    c0 = v0 + (pow(a0, 2) * pi * (1 + alpha)) / (2. * Jpeak * sa * (pi + 4 * alpha - pi * alpha));
    c1 = (2 * a0 * (1 + alpha)) / alpha;
    c2 = (Jpeak * sa * (pi + 4 * alpha - (-4 + pi) * pow(alpha, 2))) / (pi * pow(alpha, 2));
    c3 = (Jpeak * sv * (pi - pi * pow(alpha, 2) + 4 * alpha * (1 + alpha))) / (pi * pow(alpha, 2));

    vector<double> cc = { c0,c1,c2,c3 };

    //find coefficient for final displacement equation
    double d0, d1, d2, d3, d4, d5, d6, d7;
    d0 = p0 - pG - (a0 * pi * v0 * (1 + alpha)) / (Jpeak * sa * (pi * (-1 + alpha) - 4 * alpha)) +
        (pow(a0, 3) * (24 * pi * (-1 + alpha) * pow(alpha, 2) - 96 * pow(alpha, 3) - 6 * pow(pi, 2) * alpha * pow(1 + alpha, 2) + pow(pi, 3) * (-1 + alpha) * (2 + alpha) * (1 + 2 * alpha))) /
        (6. * pow(Jpeak, 2) * pow(sa, 2) * pow(pi * (-1 + alpha) - 4 * alpha, 3));
    d1 = (-12 * pow(a0, 2) * (-1 + alpha) * alpha) / (Jpeak * sa * pow(pi * (-1 + alpha) - 4 * alpha, 2)) + (48 * pow(a0, 2) * pow(alpha, 2)) / (Jpeak * pi * sa * pow(pi * (-1 + alpha) - 4 * alpha, 2)) +
        (7 * pow(a0, 2) * pi * pow(1 + alpha, 2)) / (Jpeak * sa * pow(pi * (-1 + alpha) - 4 * alpha, 2)) +
        (2 * v0 * (1 + alpha) * pow(pi + 4 * alpha - pi * alpha, 2)) / (pow(pi * (-1 + alpha) - 4 * alpha, 2) * alpha) -
        (pow(a0, 2) * pow(pi, 2) * (-1 + alpha) * (4 + alpha * (9 + 4 * alpha))) / (2. * Jpeak * sa * pow(pi * (-1 + alpha) - 4 * alpha, 2) * alpha);
    d2 = (12 * a0 * (-1 + alpha) * pow(pi + 4 * alpha - pi * alpha, 2)) / (pi * pow(pi * (-1 + alpha) - 4 * alpha, 3)) -
        (48 * a0 * alpha * pow(pi + 4 * alpha - pi * alpha, 2)) / (pow(pi, 2) * pow(pi * (-1 + alpha) - 4 * alpha, 3)) -
        (11 * a0 * pow(1 + alpha, 2) * pow(pi + 4 * alpha - pi * alpha, 2)) / (pow(pi * (-1 + alpha) - 4 * alpha, 3) * alpha) +
        (a0 * pi * (-1 + alpha) * (3 + 2 * alpha) * (2 + 3 * alpha) * pow(pi + 4 * alpha - pi * alpha, 2)) / (2. * pow(pi * (-1 + alpha) - 4 * alpha, 3) * pow(alpha, 2));
    d3 = -((Jpeak * sa * pow(1 + alpha, 2) * pow(pi + 4 * alpha - pi * alpha, 4)) / (pi * pow(pi * (-1 + alpha) - 4 * alpha, 3) * pow(alpha, 3)));
    d4 = (4 * pow(a0, 2) * pi * pow(1 + alpha, 2)) / (Jpeak * sa * pow(pi * (-1 + alpha) - 4 * alpha, 2)) -
        (pow(a0, 2) * pow(pi, 2) * (-1 + alpha) * pow(1 + alpha, 2)) / (Jpeak * sa * pow(pi * (-1 + alpha) - 4 * alpha, 2) * alpha) +
        (2 * v0 * (1 + alpha) * pow(pi + 4 * alpha - pi * alpha, 2)) / (pow(pi * (-1 + alpha) - 4 * alpha, 2) * alpha);
    d5 = (4 * a0 * pi * (-1 + alpha) * pow(1 + alpha, 2) * pow(pi + 4 * alpha - pi * alpha, 2)) / (pow(pi * (-1 + alpha) - 4 * alpha, 3) * pow(alpha, 2)) -
        (16 * a0 * pow(1 + alpha, 2) * pow(pi + 4 * alpha - pi * alpha, 2)) / (pow(pi * (-1 + alpha) - 4 * alpha, 3) * alpha);
    d6 = (-2 * Jpeak * sa * pow(1 + alpha, 2) * pow(pi + 4 * alpha - pi * alpha, 4)) / (pi * pow(pi * (-1 + alpha) - 4 * alpha, 3) * pow(alpha, 3));
    d7 = -((Jpeak * sv * pow(1 + alpha, 2) * pow(pi + 4 * alpha - pi * alpha, 4)) / (pi * pow(pi * (-1 + alpha) - 4 * alpha, 3) * pow(alpha, 3)));
    vector<double> dd = {d0,d1,d2,d3,d4,d5,d6,d7 };
    //6th degree polynomial to find ta1
    double b0, b1, b2, b3, b4, b5, b6;
    b0 = pow(c3, 3) * pow(d0, 2) + c0 * pow(c3, 2) * pow(d4, 2) - 2 * pow(c0, 2) * c3 * d4 * d7 + pow(c0, 3) * pow(d7, 2);
    b1 = 2 * pow(c3, 3) * d0 * d1 + c1 * pow(c3, 2) * pow(d4, 2) + 2 * c0 * pow(c3, 2) * d4 * d5 - 4 * c0 * c1 * c3 * d4 * d7 - 2 * pow(c0, 2) * c3 * d5 * d7 +
        3 * pow(c0, 2) * c1 * pow(d7, 2);
    b2 = pow(c3, 3) * pow(d1, 2) + 2 * pow(c3, 3) * d0 * d2 + c2 * pow(c3, 2) * pow(d4, 2) + 2 * c1 * pow(c3, 2) * d4 * d5 + c0 * pow(c3, 2) * pow(d5, 2) +
        2 * c0 * pow(c3, 2) * d4 * d6 - 2 * pow(c1, 2) * c3 * d4 * d7 - 4 * c0 * c2 * c3 * d4 * d7 - 4 * c0 * c1 * c3 * d5 * d7 - 2 * pow(c0, 2) * c3 * d6 * d7 +
        3 * c0 * pow(c1, 2) * pow(d7, 2) + 3 * pow(c0, 2) * c2 * pow(d7, 2);
    b3 = 2 * pow(c3, 3) * d1 * d2 + 2 * pow(c3, 3) * d0 * d3 + 2 * c2 * pow(c3, 2) * d4 * d5 + c1 * pow(c3, 2) * pow(d5, 2) + 2 * c1 * pow(c3, 2) * d4 * d6 +
        2 * c0 * pow(c3, 2) * d5 * d6 - 4 * c1 * c2 * c3 * d4 * d7 - 2 * pow(c1, 2) * c3 * d5 * d7 - 4 * c0 * c2 * c3 * d5 * d7 - 4 * c0 * c1 * c3 * d6 * d7 + pow(c1, 3) * pow(d7, 2) +
        6 * c0 * c1 * c2 * pow(d7, 2);
    b4 = pow(c3, 3) * pow(d2, 2) + 2 * pow(c3, 3) * d1 * d3 + c2 * pow(c3, 2) * pow(d5, 2) + 2 * c2 * pow(c3, 2) * d4 * d6 + 2 * c1 * pow(c3, 2) * d5 * d6 +
        c0 * pow(c3, 2) * pow(d6, 2) - 2 * pow(c2, 2) * c3 * d4 * d7 - 4 * c1 * c2 * c3 * d5 * d7 - 2 * pow(c1, 2) * c3 * d6 * d7 - 4 * c0 * c2 * c3 * d6 * d7 +
        3 * pow(c1, 2) * c2 * pow(d7, 2) + 3 * c0 * pow(c2, 2) * pow(d7, 2);
    b5 = 2 * pow(c3, 3) * d2 * d3 + 2 * c2 * pow(c3, 2) * d5 * d6 + c1 * pow(c3, 2) * pow(d6, 2) - 2 * pow(c2, 2) * c3 * d5 * d7 - 4 * c1 * c2 * c3 * d6 * d7 +
        3 * c1 * pow(c2, 2) * pow(d7, 2);
    b6 = pow(c3, 3) * pow(d3, 2) + c2 * pow(c3, 2) * pow(d6, 2) - 2 * pow(c2, 2) * c3 * d6 * d7 + pow(c2, 3) * pow(d7, 2);

    Eigen::VectorXd coeff;
    if (b6 != 0)
    {
        vector<double> bb = { b0,b1,b2,b3,b4,b5,b6 };
        coeff.resize(7);
        coeff << b0, b1, b2, b3, b4, b5, b6;
    }
    else
    {
        if (b5 != 0)
        {
            vector<double> bb = { b0,b1,b2,b3,b4,b5 };
            coeff.resize(6);
            coeff << b0, b1, b2, b3, b4, b5;
        }
        else
        {
            if (b4 != 0)
            {
                vector<double> bb = { b0,b1,b2,b3,b4 };
                coeff.resize(5);
                coeff << b0, b1, b2, b3, b4;
            }
            else
            {
                if (b3 != 0)
                {
                    vector<double> bb = { b0,b1,b2,b3 };
                    coeff.resize(4);
                    coeff << b0, b1, b2, b3;
                }
                else
                {
                    if (b2 != 0)
                    {
                        vector<double> bb = { b0,b1,b2};
                        coeff.resize(3);
                        coeff << b0, b1, b2;
                    }
                    else
                    {
                        vector<double> bb = { b0,b1};
                        coeff.resize(2);
                        coeff << b0, b1;
                    }
                }
            }
        }

    }
    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeff);
    std::vector<double> realRoots;
    solver.realRoots(realRoots);
    vector<double> temp = isValid(realRoots);

    double denum = pow(c3, 5) * pow(d3, 2) * pow(d4, 3) - pow(c3, 5) * d2 * d3 * pow(d4, 2) * d5 + pow(c3, 5) * d1 * d3 * d4 * pow(d5, 2) - pow(c3, 5) * d0 * d3 * pow(d5, 3) +
        pow(c3, 5) * pow(d2, 2) * pow(d4, 2) * d6 - 2 * pow(c3, 5) * d1 * d3 * pow(d4, 2) * d6 - pow(c3, 5) * d1 * d2 * d4 * d5 * d6 + 3 * pow(c3, 5) * d0 * d3 * d4 * d5 * d6 +
        pow(c3, 5) * d0 * d2 * pow(d5, 2) * d6 + pow(c3, 5) * pow(d1, 2) * d4 * pow(d6, 2) - 2 * pow(c3, 5) * d0 * d2 * d4 * pow(d6, 2) - pow(c3, 5) * d0 * d1 * d5 * pow(d6, 2) +
        pow(c3, 5) * pow(d0, 2) * pow(d6, 3) - c2 * pow(c3, 4) * pow(d2, 2) * pow(d4, 2) * d7 + 2 * c2 * pow(c3, 4) * d1 * d3 * pow(d4, 2) * d7 +
        c1 * pow(c3, 4) * d2 * d3 * pow(d4, 2) * d7 - 3 * c0 * pow(c3, 4) * pow(d3, 2) * pow(d4, 2) * d7 + c2 * pow(c3, 4) * d1 * d2 * d4 * d5 * d7 - 3 * c2 * pow(c3, 4) * d0 * d3 * d4 * d5 * d7 -
        2 * c1 * pow(c3, 4) * d1 * d3 * d4 * d5 * d7 + 2 * c0 * pow(c3, 4) * d2 * d3 * d4 * d5 * d7 - c2 * pow(c3, 4) * d0 * d2 * pow(d5, 2) * d7 + 3 * c1 * pow(c3, 4) * d0 * d3 * pow(d5, 2) * d7 -
        c0 * pow(c3, 4) * d1 * d3 * pow(d5, 2) * d7 - 2 * c2 * pow(c3, 4) * pow(d1, 2) * d4 * d6 * d7 + 4 * c2 * pow(c3, 4) * d0 * d2 * d4 * d6 * d7 + c1 * pow(c3, 4) * d1 * d2 * d4 * d6 * d7 -
        2 * c0 * pow(c3, 4) * pow(d2, 2) * d4 * d6 * d7 - 3 * c1 * pow(c3, 4) * d0 * d3 * d4 * d6 * d7 + 4 * c0 * pow(c3, 4) * d1 * d3 * d4 * d6 * d7 + 2 * c2 * pow(c3, 4) * d0 * d1 * d5 * d6 * d7 -
        2 * c1 * pow(c3, 4) * d0 * d2 * d5 * d6 * d7 + c0 * pow(c3, 4) * d1 * d2 * d5 * d6 * d7 - 3 * c0 * pow(c3, 4) * d0 * d3 * d5 * d6 * d7 - 3 * c2 * pow(c3, 4) * pow(d0, 2) * pow(d6, 2) * d7 +
        c1 * pow(c3, 4) * d0 * d1 * pow(d6, 2) * d7 - c0 * pow(c3, 4) * pow(d1, 2) * pow(d6, 2) * d7 + 2 * c0 * pow(c3, 4) * d0 * d2 * pow(d6, 2) * d7 +
        pow(c2, 2) * pow(c3, 3) * pow(d1, 2) * d4 * pow(d7, 2) - 2 * pow(c2, 2) * pow(c3, 3) * d0 * d2 * d4 * pow(d7, 2) - c1 * c2 * pow(c3, 3) * d1 * d2 * d4 * pow(d7, 2) +
        2 * c0 * c2 * pow(c3, 3) * pow(d2, 2) * d4 * pow(d7, 2) + 3 * c1 * c2 * pow(c3, 3) * d0 * d3 * d4 * pow(d7, 2) + pow(c1, 2) * pow(c3, 3) * d1 * d3 * d4 * pow(d7, 2) -
        4 * c0 * c2 * pow(c3, 3) * d1 * d3 * d4 * pow(d7, 2) - 2 * c0 * c1 * pow(c3, 3) * d2 * d3 * d4 * pow(d7, 2) + 3 * pow(c0, 2) * pow(c3, 3) * pow(d3, 2) * d4 * pow(d7, 2) -
        pow(c2, 2) * pow(c3, 3) * d0 * d1 * d5 * pow(d7, 2) + 2 * c1 * c2 * pow(c3, 3) * d0 * d2 * d5 * pow(d7, 2) - c0 * c2 * pow(c3, 3) * d1 * d2 * d5 * pow(d7, 2) -
        3 * pow(c1, 2) * pow(c3, 3) * d0 * d3 * d5 * pow(d7, 2) + 3 * c0 * c2 * pow(c3, 3) * d0 * d3 * d5 * pow(d7, 2) + 2 * c0 * c1 * pow(c3, 3) * d1 * d3 * d5 * pow(d7, 2) -
        pow(c0, 2) * pow(c3, 3) * d2 * d3 * d5 * pow(d7, 2) + 3 * pow(c2, 2) * pow(c3, 3) * pow(d0, 2) * d6 * pow(d7, 2) - 2 * c1 * c2 * pow(c3, 3) * d0 * d1 * d6 * pow(d7, 2) +
        2 * c0 * c2 * pow(c3, 3) * pow(d1, 2) * d6 * pow(d7, 2) + pow(c1, 2) * pow(c3, 3) * d0 * d2 * d6 * pow(d7, 2) - 4 * c0 * c2 * pow(c3, 3) * d0 * d2 * d6 * pow(d7, 2) -
        c0 * c1 * pow(c3, 3) * d1 * d2 * d6 * pow(d7, 2) + pow(c0, 2) * pow(c3, 3) * pow(d2, 2) * d6 * pow(d7, 2) + 3 * c0 * c1 * pow(c3, 3) * d0 * d3 * d6 * pow(d7, 2) -
        2 * pow(c0, 2) * pow(c3, 3) * d1 * d3 * d6 * pow(d7, 2) - pow(c2, 3) * pow(c3, 2) * pow(d0, 2) * pow(d7, 3) + c1 * pow(c2, 2) * pow(c3, 2) * d0 * d1 * pow(d7, 3) -
        c0 * pow(c2, 2) * pow(c3, 2) * pow(d1, 2) * pow(d7, 3) - pow(c1, 2) * c2 * pow(c3, 2) * d0 * d2 * pow(d7, 3) + 2 * c0 * pow(c2, 2) * pow(c3, 2) * d0 * d2 * pow(d7, 3) +
        c0 * c1 * c2 * pow(c3, 2) * d1 * d2 * pow(d7, 3) - pow(c0, 2) * c2 * pow(c3, 2) * pow(d2, 2) * pow(d7, 3) + pow(c1, 3) * pow(c3, 2) * d0 * d3 * pow(d7, 3) -
        3 * c0 * c1 * c2 * pow(c3, 2) * d0 * d3 * pow(d7, 3) - c0 * pow(c1, 2) * pow(c3, 2) * d1 * d3 * pow(d7, 3) + 2 * pow(c0, 2) * c2 * pow(c3, 2) * d1 * d3 * pow(d7, 3) +
        pow(c0, 2) * c1 * pow(c3, 2) * d2 * d3 * pow(d7, 3) - pow(c0, 3) * pow(c3, 2) * pow(d3, 2) * pow(d7, 3);

    double tc1, ta1,tb1,tb2,ta2,tc2;
    double t4 = 0;

    if (abs(denum) < eps)
    {
        ans.push_back({0,0});
    }
    else
    {
        for (double ta1ANS : temp)
        {
            ta1 = ta1ANS;
            tc1 = (-(pow(c3, 5) * d0 * pow(d3, 2) * pow(d4, 2)) + pow(c3, 5) * d0 * d2 * d3 * d4 * d5 - pow(c3, 5) * d0 * d1 * d3 * pow(d5, 2) - c0 * pow(c3, 4) * d3 * d4 * pow(d5, 3) -
                pow(c3, 5) * d0 * pow(d2, 2) * d4 * d6 + 2 * pow(c3, 5) * d0 * d1 * d3 * d4 * d6 + pow(c3, 5) * d0 * d1 * d2 * d5 * d6 - pow(c3, 5) * pow(d0, 2) * d3 * d5 * d6 +
                2 * c0 * pow(c3, 4) * d3 * pow(d4, 2) * d5 * d6 + c0 * pow(c3, 4) * d2 * d4 * pow(d5, 2) * d6 - pow(c3, 5) * d0 * pow(d1, 2) * pow(d6, 2) +
                pow(c3, 5) * pow(d0, 2) * d2 * pow(d6, 2) - c0 * pow(c3, 4) * d2 * pow(d4, 2) * pow(d6, 2) - c0 * pow(c3, 4) * d1 * d4 * d5 * pow(d6, 2) +
                c0 * pow(c3, 4) * d0 * d4 * pow(d6, 3) + c2 * pow(c3, 4) * d0 * pow(d2, 2) * d4 * d7 - 2 * c2 * pow(c3, 4) * d0 * d1 * d3 * d4 * d7 - c1 * pow(c3, 4) * d0 * d2 * d3 * d4 * d7 +
                2 * c0 * pow(c3, 4) * d0 * pow(d3, 2) * d4 * d7 - c2 * pow(c3, 4) * d0 * d1 * d2 * d5 * d7 + c2 * pow(c3, 4) * pow(d0, 2) * d3 * d5 * d7 + 2 * c1 * pow(c3, 4) * d0 * d1 * d3 * d5 * d7 -
                c0 * pow(c3, 4) * d0 * d2 * d3 * d5 * d7 - 2 * c0 * c2 * pow(c3, 3) * d3 * pow(d4, 2) * d5 * d7 - c0 * c2 * pow(c3, 3) * d2 * d4 * pow(d5, 2) * d7 +
                3 * c0 * c1 * pow(c3, 3) * d3 * d4 * pow(d5, 2) * d7 + pow(c0, 2) * pow(c3, 3) * d3 * pow(d5, 3) * d7 + 2 * c2 * pow(c3, 4) * d0 * pow(d1, 2) * d6 * d7 -
                2 * c2 * pow(c3, 4) * pow(d0, 2) * d2 * d6 * d7 - c1 * pow(c3, 4) * d0 * d1 * d2 * d6 * d7 + c0 * pow(c3, 4) * d0 * pow(d2, 2) * d6 * d7 + c1 * pow(c3, 4) * pow(d0, 2) * d3 * d6 * d7 -
                2 * c0 * pow(c3, 4) * d0 * d1 * d3 * d6 * d7 + 2 * c0 * c2 * pow(c3, 3) * d2 * pow(d4, 2) * d6 * d7 - 2 * c0 * c1 * pow(c3, 3) * d3 * pow(d4, 2) * d6 * d7 +
                2 * c0 * c2 * pow(c3, 3) * d1 * d4 * d5 * d6 * d7 - 2 * c0 * c1 * pow(c3, 3) * d2 * d4 * d5 * d6 * d7 - 4 * pow(c0, 2) * pow(c3, 3) * d3 * d4 * d5 * d6 * d7 -
                pow(c0, 2) * pow(c3, 3) * d2 * pow(d5, 2) * d6 * d7 - 3 * c0 * c2 * pow(c3, 3) * d0 * d4 * pow(d6, 2) * d7 + c0 * c1 * pow(c3, 3) * d1 * d4 * pow(d6, 2) * d7 +
                2 * pow(c0, 2) * pow(c3, 3) * d2 * d4 * pow(d6, 2) * d7 + pow(c0, 2) * pow(c3, 3) * d1 * d5 * pow(d6, 2) * d7 - pow(c0, 2) * pow(c3, 3) * d0 * pow(d6, 3) * d7 -
                pow(c2, 2) * pow(c3, 3) * d0 * pow(d1, 2) * pow(d7, 2) + pow(c2, 2) * pow(c3, 3) * pow(d0, 2) * d2 * pow(d7, 2) + c1 * c2 * pow(c3, 3) * d0 * d1 * d2 * pow(d7, 2) -
                c0 * c2 * pow(c3, 3) * d0 * pow(d2, 2) * pow(d7, 2) - c1 * c2 * pow(c3, 3) * pow(d0, 2) * d3 * pow(d7, 2) - pow(c1, 2) * pow(c3, 3) * d0 * d1 * d3 * pow(d7, 2) +
                2 * c0 * c2 * pow(c3, 3) * d0 * d1 * d3 * pow(d7, 2) + c0 * c1 * pow(c3, 3) * d0 * d2 * d3 * pow(d7, 2) - pow(c0, 2) * pow(c3, 3) * d0 * pow(d3, 2) * pow(d7, 2) -
                c0 * pow(c2, 2) * pow(c3, 2) * d2 * pow(d4, 2) * pow(d7, 2) + 2 * c0 * c1 * c2 * pow(c3, 2) * d3 * pow(d4, 2) * pow(d7, 2) -
                c0 * pow(c2, 2) * pow(c3, 2) * d1 * d4 * d5 * pow(d7, 2) + 2 * c0 * c1 * c2 * pow(c3, 2) * d2 * d4 * d5 * pow(d7, 2) - 3 * c0 * pow(c1, 2) * pow(c3, 2) * d3 * d4 * d5 * pow(d7, 2) +
                4 * pow(c0, 2) * c2 * pow(c3, 2) * d3 * d4 * d5 * pow(d7, 2) + pow(c0, 2) * c2 * pow(c3, 2) * d2 * pow(d5, 2) * pow(d7, 2) -
                3 * pow(c0, 2) * c1 * pow(c3, 2) * d3 * pow(d5, 2) * pow(d7, 2) + 3 * c0 * pow(c2, 2) * pow(c3, 2) * d0 * d4 * d6 * pow(d7, 2) -
                2 * c0 * c1 * c2 * pow(c3, 2) * d1 * d4 * d6 * pow(d7, 2) + c0 * pow(c1, 2) * pow(c3, 2) * d2 * d4 * d6 * pow(d7, 2) - 4 * pow(c0, 2) * c2 * pow(c3, 2) * d2 * d4 * d6 * pow(d7, 2) +
                4 * pow(c0, 2) * c1 * pow(c3, 2) * d3 * d4 * d6 * pow(d7, 2) - 2 * pow(c0, 2) * c2 * pow(c3, 2) * d1 * d5 * d6 * pow(d7, 2) +
                2 * pow(c0, 2) * c1 * pow(c3, 2) * d2 * d5 * d6 * pow(d7, 2) + 2 * pow(c0, 3) * pow(c3, 2) * d3 * d5 * d6 * pow(d7, 2) +
                3 * pow(c0, 2) * c2 * pow(c3, 2) * d0 * pow(d6, 2) * pow(d7, 2) - pow(c0, 2) * c1 * pow(c3, 2) * d1 * pow(d6, 2) * pow(d7, 2) -
                pow(c0, 3) * pow(c3, 2) * d2 * pow(d6, 2) * pow(d7, 2) - c0 * pow(c2, 3) * c3 * d0 * d4 * pow(d7, 3) + c0 * c1 * pow(c2, 2) * c3 * d1 * d4 * pow(d7, 3) -
                c0 * pow(c1, 2) * c2 * c3 * d2 * d4 * pow(d7, 3) + 2 * pow(c0, 2) * pow(c2, 2) * c3 * d2 * d4 * pow(d7, 3) + c0 * pow(c1, 3) * c3 * d3 * d4 * pow(d7, 3) -
                4 * pow(c0, 2) * c1 * c2 * c3 * d3 * d4 * pow(d7, 3) + pow(c0, 2) * pow(c2, 2) * c3 * d1 * d5 * pow(d7, 3) - 2 * pow(c0, 2) * c1 * c2 * c3 * d2 * d5 * pow(d7, 3) +
                3 * pow(c0, 2) * pow(c1, 2) * c3 * d3 * d5 * pow(d7, 3) - 2 * pow(c0, 3) * c2 * c3 * d3 * d5 * pow(d7, 3) - 3 * pow(c0, 2) * pow(c2, 2) * c3 * d0 * d6 * pow(d7, 3) +
                2 * pow(c0, 2) * c1 * c2 * c3 * d1 * d6 * pow(d7, 3) - pow(c0, 2) * pow(c1, 2) * c3 * d2 * d6 * pow(d7, 3) + 2 * pow(c0, 3) * c2 * c3 * d2 * d6 * pow(d7, 3) -
                2 * pow(c0, 3) * c1 * c3 * d3 * d6 * pow(d7, 3) + pow(c0, 2) * pow(c2, 3) * d0 * pow(d7, 4) - pow(c0, 2) * c1 * pow(c2, 2) * d1 * pow(d7, 4) +
                pow(c0, 2) * pow(c1, 2) * c2 * d2 * pow(d7, 4) - pow(c0, 3) * pow(c2, 2) * d2 * pow(d7, 4) - pow(c0, 2) * pow(c1, 3) * d3 * pow(d7, 4) +
                2 * pow(c0, 3) * c1 * c2 * d3 * pow(d7, 4) - pow(c3, 5) * d1 * pow(d3, 2) * pow(d4, 2) * ta1ANS + pow(c3, 5) * d1 * d2 * d3 * d4 * d5 * ta1ANS +
                pow(c3, 5) * d0 * pow(d3, 2) * d4 * d5 * ta1ANS - pow(c3, 5) * pow(d1, 2) * d3 * pow(d5, 2) * ta1ANS - pow(c3, 5) * d0 * d2 * d3 * pow(d5, 2) * ta1ANS -
                c1 * pow(c3, 4) * d3 * d4 * pow(d5, 3) * ta1ANS - c0 * pow(c3, 4) * d3 * pow(d5, 4) * ta1ANS - pow(c3, 5) * d1 * pow(d2, 2) * d4 * d6 * ta1ANS +
                2 * pow(c3, 5) * pow(d1, 2) * d3 * d4 * d6 * ta1ANS + pow(c3, 5) * pow(d1, 2) * d2 * d5 * d6 * ta1ANS + pow(c3, 5) * d0 * pow(d2, 2) * d5 * d6 * ta1ANS -
                pow(c3, 5) * d0 * d1 * d3 * d5 * d6 * ta1ANS + 2 * c1 * pow(c3, 4) * d3 * pow(d4, 2) * d5 * d6 * ta1ANS + c1 * pow(c3, 4) * d2 * d4 * pow(d5, 2) * d6 * ta1ANS +
                c0 * pow(c3, 4) * d3 * d4 * pow(d5, 2) * d6 * ta1ANS + c0 * pow(c3, 4) * d2 * pow(d5, 3) * d6 * ta1ANS - pow(c3, 5) * pow(d1, 3) * pow(d6, 2) * ta1ANS +
                pow(c3, 5) * pow(d0, 2) * d3 * pow(d6, 2) * ta1ANS - c1 * pow(c3, 4) * d2 * pow(d4, 2) * pow(d6, 2) * ta1ANS +
                c0 * pow(c3, 4) * d3 * pow(d4, 2) * pow(d6, 2) * ta1ANS - c1 * pow(c3, 4) * d1 * d4 * d5 * pow(d6, 2) * ta1ANS - c0 * pow(c3, 4) * d1 * pow(d5, 2) * pow(d6, 2) * ta1ANS +
                c1 * pow(c3, 4) * d0 * d4 * pow(d6, 3) * ta1ANS - c0 * pow(c3, 4) * d1 * d4 * pow(d6, 3) * ta1ANS + c0 * pow(c3, 4) * d0 * d5 * pow(d6, 3) * ta1ANS +
                c2 * pow(c3, 4) * d1 * pow(d2, 2) * d4 * d7 * ta1ANS - 2 * c2 * pow(c3, 4) * pow(d1, 2) * d3 * d4 * d7 * ta1ANS - c1 * pow(c3, 4) * d1 * d2 * d3 * d4 * d7 * ta1ANS -
                c1 * pow(c3, 4) * d0 * pow(d3, 2) * d4 * d7 * ta1ANS + 2 * c0 * pow(c3, 4) * d1 * pow(d3, 2) * d4 * d7 * ta1ANS - c2 * pow(c3, 4) * pow(d1, 2) * d2 * d5 * d7 * ta1ANS -
                c2 * pow(c3, 4) * d0 * pow(d2, 2) * d5 * d7 * ta1ANS + c2 * pow(c3, 4) * d0 * d1 * d3 * d5 * d7 * ta1ANS + 2 * c1 * pow(c3, 4) * pow(d1, 2) * d3 * d5 * d7 * ta1ANS +
                2 * c1 * pow(c3, 4) * d0 * d2 * d3 * d5 * d7 * ta1ANS - c0 * pow(c3, 4) * d1 * d2 * d3 * d5 * d7 * ta1ANS - c0 * pow(c3, 4) * d0 * pow(d3, 2) * d5 * d7 * ta1ANS -
                2 * c1 * c2 * pow(c3, 3) * d3 * pow(d4, 2) * d5 * d7 * ta1ANS - c1 * c2 * pow(c3, 3) * d2 * d4 * pow(d5, 2) * d7 * ta1ANS +
                3 * pow(c1, 2) * pow(c3, 3) * d3 * d4 * pow(d5, 2) * d7 * ta1ANS - c0 * c2 * pow(c3, 3) * d3 * d4 * pow(d5, 2) * d7 * ta1ANS -
                c0 * c2 * pow(c3, 3) * d2 * pow(d5, 3) * d7 * ta1ANS + 5 * c0 * c1 * pow(c3, 3) * d3 * pow(d5, 3) * d7 * ta1ANS + 2 * c2 * pow(c3, 4) * pow(d1, 3) * d6 * d7 * ta1ANS -
                c1 * pow(c3, 4) * pow(d1, 2) * d2 * d6 * d7 * ta1ANS - c1 * pow(c3, 4) * d0 * pow(d2, 2) * d6 * d7 * ta1ANS + c0 * pow(c3, 4) * d1 * pow(d2, 2) * d6 * d7 * ta1ANS -
                2 * c2 * pow(c3, 4) * pow(d0, 2) * d3 * d6 * d7 * ta1ANS + c1 * pow(c3, 4) * d0 * d1 * d3 * d6 * d7 * ta1ANS - 2 * c0 * pow(c3, 4) * pow(d1, 2) * d3 * d6 * d7 * ta1ANS +
                2 * c1 * c2 * pow(c3, 3) * d2 * pow(d4, 2) * d6 * d7 * ta1ANS - 2 * pow(c1, 2) * pow(c3, 3) * d3 * pow(d4, 2) * d6 * d7 * ta1ANS -
                2 * c0 * c2 * pow(c3, 3) * d3 * pow(d4, 2) * d6 * d7 * ta1ANS + 2 * c1 * c2 * pow(c3, 3) * d1 * d4 * d5 * d6 * d7 * ta1ANS - 2 * pow(c1, 2) * pow(c3, 3) * d2 * d4 * d5 * d6 * d7 * ta1ANS -
                6 * c0 * c1 * pow(c3, 3) * d3 * d4 * d5 * d6 * d7 * ta1ANS + 2 * c0 * c2 * pow(c3, 3) * d1 * pow(d5, 2) * d6 * d7 * ta1ANS - 4 * c0 * c1 * pow(c3, 3) * d2 * pow(d5, 2) * d6 * d7 * ta1ANS -
                pow(c0, 2) * pow(c3, 3) * d3 * pow(d5, 2) * d6 * d7 * ta1ANS - 3 * c1 * c2 * pow(c3, 3) * d0 * d4 * pow(d6, 2) * d7 * ta1ANS +
                pow(c1, 2) * pow(c3, 3) * d1 * d4 * pow(d6, 2) * d7 * ta1ANS + 3 * c0 * c2 * pow(c3, 3) * d1 * d4 * pow(d6, 2) * d7 * ta1ANS +
                2 * c0 * c1 * pow(c3, 3) * d2 * d4 * pow(d6, 2) * d7 * ta1ANS - 2 * pow(c0, 2) * pow(c3, 3) * d3 * d4 * pow(d6, 2) * d7 * ta1ANS -
                3 * c0 * c2 * pow(c3, 3) * d0 * d5 * pow(d6, 2) * d7 * ta1ANS + 3 * c0 * c1 * pow(c3, 3) * d1 * d5 * pow(d6, 2) * d7 * ta1ANS - 2 * c0 * c1 * pow(c3, 3) * d0 * pow(d6, 3) * d7 * ta1ANS +
                pow(c0, 2) * pow(c3, 3) * d1 * pow(d6, 3) * d7 * ta1ANS - pow(c2, 2) * pow(c3, 3) * pow(d1, 3) * pow(d7, 2) * ta1ANS +
                c1 * c2 * pow(c3, 3) * pow(d1, 2) * d2 * pow(d7, 2) * ta1ANS + c1 * c2 * pow(c3, 3) * d0 * pow(d2, 2) * pow(d7, 2) * ta1ANS -
                c0 * c2 * pow(c3, 3) * d1 * pow(d2, 2) * pow(d7, 2) * ta1ANS + pow(c2, 2) * pow(c3, 3) * pow(d0, 2) * d3 * pow(d7, 2) * ta1ANS -
                c1 * c2 * pow(c3, 3) * d0 * d1 * d3 * pow(d7, 2) * ta1ANS - pow(c1, 2) * pow(c3, 3) * pow(d1, 2) * d3 * pow(d7, 2) * ta1ANS +
                2 * c0 * c2 * pow(c3, 3) * pow(d1, 2) * d3 * pow(d7, 2) * ta1ANS - pow(c1, 2) * pow(c3, 3) * d0 * d2 * d3 * pow(d7, 2) * ta1ANS +
                c0 * c1 * pow(c3, 3) * d1 * d2 * d3 * pow(d7, 2) * ta1ANS + c0 * c1 * pow(c3, 3) * d0 * pow(d3, 2) * pow(d7, 2) * ta1ANS -
                pow(c0, 2) * pow(c3, 3) * d1 * pow(d3, 2) * pow(d7, 2) * ta1ANS - c1 * pow(c2, 2) * pow(c3, 2) * d2 * pow(d4, 2) * pow(d7, 2) * ta1ANS +
                2 * pow(c1, 2) * c2 * pow(c3, 2) * d3 * pow(d4, 2) * pow(d7, 2) * ta1ANS + c0 * pow(c2, 2) * pow(c3, 2) * d3 * pow(d4, 2) * pow(d7, 2) * ta1ANS -
                c1 * pow(c2, 2) * pow(c3, 2) * d1 * d4 * d5 * pow(d7, 2) * ta1ANS + 2 * pow(c1, 2) * c2 * pow(c3, 2) * d2 * d4 * d5 * pow(d7, 2) * ta1ANS -
                3 * pow(c1, 3) * pow(c3, 2) * d3 * d4 * d5 * pow(d7, 2) * ta1ANS + 6 * c0 * c1 * c2 * pow(c3, 2) * d3 * d4 * d5 * pow(d7, 2) * ta1ANS -
                c0 * pow(c2, 2) * pow(c3, 2) * d1 * pow(d5, 2) * pow(d7, 2) * ta1ANS + 4 * c0 * c1 * c2 * pow(c3, 2) * d2 * pow(d5, 2) * pow(d7, 2) * ta1ANS -
                9 * c0 * pow(c1, 2) * pow(c3, 2) * d3 * pow(d5, 2) * pow(d7, 2) * ta1ANS + pow(c0, 2) * c2 * pow(c3, 2) * d3 * pow(d5, 2) * pow(d7, 2) * ta1ANS +
                3 * c1 * pow(c2, 2) * pow(c3, 2) * d0 * d4 * d6 * pow(d7, 2) * ta1ANS - 2 * pow(c1, 2) * c2 * pow(c3, 2) * d1 * d4 * d6 * pow(d7, 2) * ta1ANS -
                3 * c0 * pow(c2, 2) * pow(c3, 2) * d1 * d4 * d6 * pow(d7, 2) * ta1ANS + pow(c1, 3) * pow(c3, 2) * d2 * d4 * d6 * pow(d7, 2) * ta1ANS -
                4 * c0 * c1 * c2 * pow(c3, 2) * d2 * d4 * d6 * pow(d7, 2) * ta1ANS + 5 * c0 * pow(c1, 2) * pow(c3, 2) * d3 * d4 * d6 * pow(d7, 2) * ta1ANS +
                4 * pow(c0, 2) * c2 * pow(c3, 2) * d3 * d4 * d6 * pow(d7, 2) * ta1ANS + 3 * c0 * pow(c2, 2) * pow(c3, 2) * d0 * d5 * d6 * pow(d7, 2) * ta1ANS -
                6 * c0 * c1 * c2 * pow(c3, 2) * d1 * d5 * d6 * pow(d7, 2) * ta1ANS + 5 * c0 * pow(c1, 2) * pow(c3, 2) * d2 * d5 * d6 * pow(d7, 2) * ta1ANS +
                4 * pow(c0, 2) * c1 * pow(c3, 2) * d3 * d5 * d6 * pow(d7, 2) * ta1ANS + 6 * c0 * c1 * c2 * pow(c3, 2) * d0 * pow(d6, 2) * pow(d7, 2) * ta1ANS -
                2 * c0 * pow(c1, 2) * pow(c3, 2) * d1 * pow(d6, 2) * pow(d7, 2) * ta1ANS - 3 * pow(c0, 2) * c2 * pow(c3, 2) * d1 * pow(d6, 2) * pow(d7, 2) * ta1ANS -
                pow(c0, 2) * c1 * pow(c3, 2) * d2 * pow(d6, 2) * pow(d7, 2) * ta1ANS + pow(c0, 3) * pow(c3, 2) * d3 * pow(d6, 2) * pow(d7, 2) * ta1ANS -
                c1 * pow(c2, 3) * c3 * d0 * d4 * pow(d7, 3) * ta1ANS + pow(c1, 2) * pow(c2, 2) * c3 * d1 * d4 * pow(d7, 3) * ta1ANS + c0 * pow(c2, 3) * c3 * d1 * d4 * pow(d7, 3) * ta1ANS -
                pow(c1, 3) * c2 * c3 * d2 * d4 * pow(d7, 3) * ta1ANS + 2 * c0 * c1 * pow(c2, 2) * c3 * d2 * d4 * pow(d7, 3) * ta1ANS + pow(c1, 4) * c3 * d3 * d4 * pow(d7, 3) * ta1ANS -
                5 * c0 * pow(c1, 2) * c2 * c3 * d3 * d4 * pow(d7, 3) * ta1ANS - 2 * pow(c0, 2) * pow(c2, 2) * c3 * d3 * d4 * pow(d7, 3) * ta1ANS -
                c0 * pow(c2, 3) * c3 * d0 * d5 * pow(d7, 3) * ta1ANS + 3 * c0 * c1 * pow(c2, 2) * c3 * d1 * d5 * pow(d7, 3) * ta1ANS - 5 * c0 * pow(c1, 2) * c2 * c3 * d2 * d5 * pow(d7, 3) * ta1ANS +
                7 * c0 * pow(c1, 3) * c3 * d3 * d5 * pow(d7, 3) * ta1ANS - 4 * pow(c0, 2) * c1 * c2 * c3 * d3 * d5 * pow(d7, 3) * ta1ANS - 6 * c0 * c1 * pow(c2, 2) * c3 * d0 * d6 * pow(d7, 3) * ta1ANS +
                4 * c0 * pow(c1, 2) * c2 * c3 * d1 * d6 * pow(d7, 3) * ta1ANS + 3 * pow(c0, 2) * pow(c2, 2) * c3 * d1 * d6 * pow(d7, 3) * ta1ANS -
                2 * c0 * pow(c1, 3) * c3 * d2 * d6 * pow(d7, 3) * ta1ANS + 2 * pow(c0, 2) * c1 * c2 * c3 * d2 * d6 * pow(d7, 3) * ta1ANS -
                3 * pow(c0, 2) * pow(c1, 2) * c3 * d3 * d6 * pow(d7, 3) * ta1ANS - 2 * pow(c0, 3) * c2 * c3 * d3 * d6 * pow(d7, 3) * ta1ANS + 2 * c0 * c1 * pow(c2, 3) * d0 * pow(d7, 4) * ta1ANS -
                2 * c0 * pow(c1, 2) * pow(c2, 2) * d1 * pow(d7, 4) * ta1ANS - pow(c0, 2) * pow(c2, 3) * d1 * pow(d7, 4) * ta1ANS + 2 * c0 * pow(c1, 3) * c2 * d2 * pow(d7, 4) * ta1ANS -
                pow(c0, 2) * c1 * pow(c2, 2) * d2 * pow(d7, 4) * ta1ANS - 2 * c0 * pow(c1, 4) * d3 * pow(d7, 4) * ta1ANS + 3 * pow(c0, 2) * pow(c1, 2) * c2 * d3 * pow(d7, 4) * ta1ANS +
                pow(c0, 3) * pow(c2, 2) * d3 * pow(d7, 4) * ta1ANS - pow(c3, 5) * d2 * pow(d3, 2) * pow(d4, 2) * pow(ta1ANS, 2) +
                pow(c3, 5) * pow(d2, 2) * d3 * d4 * d5 * pow(ta1ANS, 2) + pow(c3, 5) * d1 * pow(d3, 2) * d4 * d5 * pow(ta1ANS, 2) -
                2 * pow(c3, 5) * d1 * d2 * d3 * pow(d5, 2) * pow(ta1ANS, 2) - pow(c3, 5) * d0 * pow(d3, 2) * pow(d5, 2) * pow(ta1ANS, 2) -
                c2 * pow(c3, 4) * d3 * d4 * pow(d5, 3) * pow(ta1ANS, 2) - c1 * pow(c3, 4) * d3 * pow(d5, 4) * pow(ta1ANS, 2) - pow(c3, 5) * pow(d2, 3) * d4 * d6 * pow(ta1ANS, 2) +
                2 * pow(c3, 5) * d1 * d2 * d3 * d4 * d6 * pow(ta1ANS, 2) + pow(c3, 5) * d0 * pow(d3, 2) * d4 * d6 * pow(ta1ANS, 2) +
                2 * pow(c3, 5) * d1 * pow(d2, 2) * d5 * d6 * pow(ta1ANS, 2) + 2 * c2 * pow(c3, 4) * d3 * pow(d4, 2) * d5 * d6 * pow(ta1ANS, 2) +
                c2 * pow(c3, 4) * d2 * d4 * pow(d5, 2) * d6 * pow(ta1ANS, 2) + c1 * pow(c3, 4) * d3 * d4 * pow(d5, 2) * d6 * pow(ta1ANS, 2) +
                c1 * pow(c3, 4) * d2 * pow(d5, 3) * d6 * pow(ta1ANS, 2) - 2 * c0 * pow(c3, 4) * d3 * pow(d5, 3) * d6 * pow(ta1ANS, 2) -
                2 * pow(c3, 5) * pow(d1, 2) * d2 * pow(d6, 2) * pow(ta1ANS, 2) + pow(c3, 5) * d0 * pow(d2, 2) * pow(d6, 2) * pow(ta1ANS, 2) -
                c2 * pow(c3, 4) * d2 * pow(d4, 2) * pow(d6, 2) * pow(ta1ANS, 2) + c1 * pow(c3, 4) * d3 * pow(d4, 2) * pow(d6, 2) * pow(ta1ANS, 2) -
                c2 * pow(c3, 4) * d1 * d4 * d5 * pow(d6, 2) * pow(ta1ANS, 2) + 3 * c0 * pow(c3, 4) * d3 * d4 * d5 * pow(d6, 2) * pow(ta1ANS, 2) -
                c1 * pow(c3, 4) * d1 * pow(d5, 2) * pow(d6, 2) * pow(ta1ANS, 2) + 2 * c0 * pow(c3, 4) * d2 * pow(d5, 2) * pow(d6, 2) * pow(ta1ANS, 2) +
                c2 * pow(c3, 4) * d0 * d4 * pow(d6, 3) * pow(ta1ANS, 2) - c1 * pow(c3, 4) * d1 * d4 * pow(d6, 3) * pow(ta1ANS, 2) -
                c0 * pow(c3, 4) * d2 * d4 * pow(d6, 3) * pow(ta1ANS, 2) + c1 * pow(c3, 4) * d0 * d5 * pow(d6, 3) * pow(ta1ANS, 2) -
                2 * c0 * pow(c3, 4) * d1 * d5 * pow(d6, 3) * pow(ta1ANS, 2) + c0 * pow(c3, 4) * d0 * pow(d6, 4) * pow(ta1ANS, 2) +
                c2 * pow(c3, 4) * pow(d2, 3) * d4 * d7 * pow(ta1ANS, 2) - 2 * c2 * pow(c3, 4) * d1 * d2 * d3 * d4 * d7 * pow(ta1ANS, 2) -
                c1 * pow(c3, 4) * pow(d2, 2) * d3 * d4 * d7 * pow(ta1ANS, 2) - c2 * pow(c3, 4) * d0 * pow(d3, 2) * d4 * d7 * pow(ta1ANS, 2) -
                c1 * pow(c3, 4) * d1 * pow(d3, 2) * d4 * d7 * pow(ta1ANS, 2) + 2 * c0 * pow(c3, 4) * d2 * pow(d3, 2) * d4 * d7 * pow(ta1ANS, 2) -
                2 * c2 * pow(c3, 4) * d1 * pow(d2, 2) * d5 * d7 * pow(ta1ANS, 2) + 4 * c1 * pow(c3, 4) * d1 * d2 * d3 * d5 * d7 * pow(ta1ANS, 2) -
                c0 * pow(c3, 4) * pow(d2, 2) * d3 * d5 * d7 * pow(ta1ANS, 2) + 2 * c1 * pow(c3, 4) * d0 * pow(d3, 2) * d5 * d7 * pow(ta1ANS, 2) -
                c0 * pow(c3, 4) * d1 * pow(d3, 2) * d5 * d7 * pow(ta1ANS, 2) - 2 * pow(c2, 2) * pow(c3, 3) * d3 * pow(d4, 2) * d5 * d7 * pow(ta1ANS, 2) -
                pow(c2, 2) * pow(c3, 3) * d2 * d4 * pow(d5, 2) * d7 * pow(ta1ANS, 2) + 2 * c1 * c2 * pow(c3, 3) * d3 * d4 * pow(d5, 2) * d7 * pow(ta1ANS, 2) -
                c1 * c2 * pow(c3, 3) * d2 * pow(d5, 3) * d7 * pow(ta1ANS, 2) + 4 * pow(c1, 2) * pow(c3, 3) * d3 * pow(d5, 3) * d7 * pow(ta1ANS, 2) +
                3 * c0 * c2 * pow(c3, 3) * d3 * pow(d5, 3) * d7 * pow(ta1ANS, 2) + 4 * c2 * pow(c3, 4) * pow(d1, 2) * d2 * d6 * d7 * pow(ta1ANS, 2) -
                2 * c2 * pow(c3, 4) * d0 * pow(d2, 2) * d6 * d7 * pow(ta1ANS, 2) - 2 * c1 * pow(c3, 4) * d1 * pow(d2, 2) * d6 * d7 * pow(ta1ANS, 2) +
                c0 * pow(c3, 4) * pow(d2, 3) * d6 * d7 * pow(ta1ANS, 2) - 2 * c0 * pow(c3, 4) * d1 * d2 * d3 * d6 * d7 * pow(ta1ANS, 2) -
                c0 * pow(c3, 4) * d0 * pow(d3, 2) * d6 * d7 * pow(ta1ANS, 2) + 2 * pow(c2, 2) * pow(c3, 3) * d2 * pow(d4, 2) * d6 * d7 * pow(ta1ANS, 2) -
                4 * c1 * c2 * pow(c3, 3) * d3 * pow(d4, 2) * d6 * d7 * pow(ta1ANS, 2) + 2 * pow(c2, 2) * pow(c3, 3) * d1 * d4 * d5 * d6 * d7 * pow(ta1ANS, 2) -
                2 * c1 * c2 * pow(c3, 3) * d2 * d4 * d5 * d6 * d7 * pow(ta1ANS, 2) - 2 * pow(c1, 2) * pow(c3, 3) * d3 * d4 * d5 * d6 * d7 * pow(ta1ANS, 2) -
                10 * c0 * c2 * pow(c3, 3) * d3 * d4 * d5 * d6 * d7 * pow(ta1ANS, 2) + 2 * c1 * c2 * pow(c3, 3) * d1 * pow(d5, 2) * d6 * d7 * pow(ta1ANS, 2) -
                3 * pow(c1, 2) * pow(c3, 3) * d2 * pow(d5, 2) * d6 * d7 * pow(ta1ANS, 2) - 5 * c0 * c2 * pow(c3, 3) * d2 * pow(d5, 2) * d6 * d7 * pow(ta1ANS, 2) +
                5 * c0 * c1 * pow(c3, 3) * d3 * pow(d5, 2) * d6 * d7 * pow(ta1ANS, 2) - 3 * pow(c2, 2) * pow(c3, 3) * d0 * d4 * pow(d6, 2) * d7 * pow(ta1ANS, 2) +
                4 * c1 * c2 * pow(c3, 3) * d1 * d4 * pow(d6, 2) * d7 * pow(ta1ANS, 2) + 5 * c0 * c2 * pow(c3, 3) * d2 * d4 * pow(d6, 2) * d7 * pow(ta1ANS, 2) -
                5 * c0 * c1 * pow(c3, 3) * d3 * d4 * pow(d6, 2) * d7 * pow(ta1ANS, 2) - 3 * c1 * c2 * pow(c3, 3) * d0 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 2) +
                2 * pow(c1, 2) * pow(c3, 3) * d1 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 2) + 7 * c0 * c2 * pow(c3, 3) * d1 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 2) -
                4 * c0 * c1 * pow(c3, 3) * d2 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 2) - 3 * pow(c0, 2) * pow(c3, 3) * d3 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 2) -
                pow(c1, 2) * pow(c3, 3) * d0 * pow(d6, 3) * d7 * pow(ta1ANS, 2) - 5 * c0 * c2 * pow(c3, 3) * d0 * pow(d6, 3) * d7 * pow(ta1ANS, 2) +
                3 * c0 * c1 * pow(c3, 3) * d1 * pow(d6, 3) * d7 * pow(ta1ANS, 2) + pow(c0, 2) * pow(c3, 3) * d2 * pow(d6, 3) * d7 * pow(ta1ANS, 2) -
                2 * pow(c2, 2) * pow(c3, 3) * pow(d1, 2) * d2 * pow(d7, 2) * pow(ta1ANS, 2) + pow(c2, 2) * pow(c3, 3) * d0 * pow(d2, 2) * pow(d7, 2) * pow(ta1ANS, 2) +
                2 * c1 * c2 * pow(c3, 3) * d1 * pow(d2, 2) * pow(d7, 2) * pow(ta1ANS, 2) - c0 * c2 * pow(c3, 3) * pow(d2, 3) * pow(d7, 2) * pow(ta1ANS, 2) -
                2 * pow(c1, 2) * pow(c3, 3) * d1 * d2 * d3 * pow(d7, 2) * pow(ta1ANS, 2) + 2 * c0 * c2 * pow(c3, 3) * d1 * d2 * d3 * pow(d7, 2) * pow(ta1ANS, 2) +
                c0 * c1 * pow(c3, 3) * pow(d2, 2) * d3 * pow(d7, 2) * pow(ta1ANS, 2) - pow(c1, 2) * pow(c3, 3) * d0 * pow(d3, 2) * pow(d7, 2) * pow(ta1ANS, 2) +
                c0 * c2 * pow(c3, 3) * d0 * pow(d3, 2) * pow(d7, 2) * pow(ta1ANS, 2) + c0 * c1 * pow(c3, 3) * d1 * pow(d3, 2) * pow(d7, 2) * pow(ta1ANS, 2) -
                pow(c0, 2) * pow(c3, 3) * d2 * pow(d3, 2) * pow(d7, 2) * pow(ta1ANS, 2) - pow(c2, 3) * pow(c3, 2) * d2 * pow(d4, 2) * pow(d7, 2) * pow(ta1ANS, 2) +
                3 * c1 * pow(c2, 2) * pow(c3, 2) * d3 * pow(d4, 2) * pow(d7, 2) * pow(ta1ANS, 2) - pow(c2, 3) * pow(c3, 2) * d1 * d4 * d5 * pow(d7, 2) * pow(ta1ANS, 2) +
                2 * c1 * pow(c2, 2) * pow(c3, 2) * d2 * d4 * d5 * pow(d7, 2) * pow(ta1ANS, 2) - pow(c1, 2) * c2 * pow(c3, 2) * d3 * d4 * d5 * pow(d7, 2) * pow(ta1ANS, 2) +
                7 * c0 * pow(c2, 2) * pow(c3, 2) * d3 * d4 * d5 * pow(d7, 2) * pow(ta1ANS, 2) - c1 * pow(c2, 2) * pow(c3, 2) * d1 * pow(d5, 2) * pow(d7, 2) * pow(ta1ANS, 2) +
                3 * pow(c1, 2) * c2 * pow(c3, 2) * d2 * pow(d5, 2) * pow(d7, 2) * pow(ta1ANS, 2) + 3 * c0 * pow(c2, 2) * pow(c3, 2) * d2 * pow(d5, 2) * pow(d7, 2) * pow(ta1ANS, 2) -
                6 * pow(c1, 3) * pow(c3, 2) * d3 * pow(d5, 2) * pow(d7, 2) * pow(ta1ANS, 2) - 8 * c0 * c1 * c2 * pow(c3, 2) * d3 * pow(d5, 2) * pow(d7, 2) * pow(ta1ANS, 2) +
                3 * pow(c2, 3) * pow(c3, 2) * d0 * d4 * d6 * pow(d7, 2) * pow(ta1ANS, 2) - 5 * c1 * pow(c2, 2) * pow(c3, 2) * d1 * d4 * d6 * pow(d7, 2) * pow(ta1ANS, 2) +
                pow(c1, 2) * c2 * pow(c3, 2) * d2 * d4 * d6 * pow(d7, 2) * pow(ta1ANS, 2) - 7 * c0 * pow(c2, 2) * pow(c3, 2) * d2 * d4 * d6 * pow(d7, 2) * pow(ta1ANS, 2) +
                pow(c1, 3) * pow(c3, 2) * d3 * d4 * d6 * pow(d7, 2) * pow(ta1ANS, 2) + 14 * c0 * c1 * c2 * pow(c3, 2) * d3 * d4 * d6 * pow(d7, 2) * pow(ta1ANS, 2) +
                3 * c1 * pow(c2, 2) * pow(c3, 2) * d0 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 2) - 4 * pow(c1, 2) * c2 * pow(c3, 2) * d1 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 2) -
                8 * c0 * pow(c2, 2) * pow(c3, 2) * d1 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 2) + 3 * pow(c1, 3) * pow(c3, 2) * d2 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 2) +
                10 * c0 * c1 * c2 * pow(c3, 2) * d2 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 2) - 4 * c0 * pow(c1, 2) * pow(c3, 2) * d3 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 2) +
                8 * pow(c0, 2) * c2 * pow(c3, 2) * d3 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 2) + 3 * pow(c1, 2) * c2 * pow(c3, 2) * d0 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 2) +
                9 * c0 * pow(c2, 2) * pow(c3, 2) * d0 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 2) - pow(c1, 3) * pow(c3, 2) * d1 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 2) -
                10 * c0 * c1 * c2 * pow(c3, 2) * d1 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 2) + 2 * c0 * pow(c1, 2) * pow(c3, 2) * d2 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 2) -
                4 * pow(c0, 2) * c2 * pow(c3, 2) * d2 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 2) + 4 * pow(c0, 2) * c1 * pow(c3, 2) * d3 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 2) -
                pow(c2, 4) * c3 * d0 * d4 * pow(d7, 3) * pow(ta1ANS, 2) + 2 * c1 * pow(c2, 3) * c3 * d1 * d4 * pow(d7, 3) * pow(ta1ANS, 2) -
                pow(c1, 2) * pow(c2, 2) * c3 * d2 * d4 * pow(d7, 3) * pow(ta1ANS, 2) + 3 * c0 * pow(c2, 3) * c3 * d2 * d4 * pow(d7, 3) * pow(ta1ANS, 2) -
                9 * c0 * c1 * pow(c2, 2) * c3 * d3 * d4 * pow(d7, 3) * pow(ta1ANS, 2) - c1 * pow(c2, 3) * c3 * d0 * d5 * pow(d7, 3) * pow(ta1ANS, 2) +
                2 * pow(c1, 2) * pow(c2, 2) * c3 * d1 * d5 * pow(d7, 3) * pow(ta1ANS, 2) + 3 * c0 * pow(c2, 3) * c3 * d1 * d5 * pow(d7, 3) * pow(ta1ANS, 2) -
                3 * pow(c1, 3) * c2 * c3 * d2 * d5 * pow(d7, 3) * pow(ta1ANS, 2) - 6 * c0 * c1 * pow(c2, 2) * c3 * d2 * d5 * pow(d7, 3) * pow(ta1ANS, 2) +
                4 * pow(c1, 4) * c3 * d3 * d5 * pow(d7, 3) * pow(ta1ANS, 2) + 7 * c0 * pow(c1, 2) * c2 * c3 * d3 * d5 * pow(d7, 3) * pow(ta1ANS, 2) -
                5 * pow(c0, 2) * pow(c2, 2) * c3 * d3 * d5 * pow(d7, 3) * pow(ta1ANS, 2) - 3 * pow(c1, 2) * pow(c2, 2) * c3 * d0 * d6 * pow(d7, 3) * pow(ta1ANS, 2) -
                7 * c0 * pow(c2, 3) * c3 * d0 * d6 * pow(d7, 3) * pow(ta1ANS, 2) + 2 * pow(c1, 3) * c2 * c3 * d1 * d6 * pow(d7, 3) * pow(ta1ANS, 2) +
                11 * c0 * c1 * pow(c2, 2) * c3 * d1 * d6 * pow(d7, 3) * pow(ta1ANS, 2) - pow(c1, 4) * c3 * d2 * d6 * pow(d7, 3) * pow(ta1ANS, 2) -
                5 * c0 * pow(c1, 2) * c2 * c3 * d2 * d6 * pow(d7, 3) * pow(ta1ANS, 2) + 5 * pow(c0, 2) * pow(c2, 2) * c3 * d2 * d6 * pow(d7, 3) * pow(ta1ANS, 2) +
                c0 * pow(c1, 3) * c3 * d3 * d6 * pow(d7, 3) * pow(ta1ANS, 2) - 10 * pow(c0, 2) * c1 * c2 * c3 * d3 * d6 * pow(d7, 3) * pow(ta1ANS, 2) +
                pow(c1, 2) * pow(c2, 3) * d0 * pow(d7, 4) * pow(ta1ANS, 2) + 2 * c0 * pow(c2, 4) * d0 * pow(d7, 4) * pow(ta1ANS, 2) -
                pow(c1, 3) * pow(c2, 2) * d1 * pow(d7, 4) * pow(ta1ANS, 2) - 4 * c0 * c1 * pow(c2, 3) * d1 * pow(d7, 4) * pow(ta1ANS, 2) +
                pow(c1, 4) * c2 * d2 * pow(d7, 4) * pow(ta1ANS, 2) + 3 * c0 * pow(c1, 2) * pow(c2, 2) * d2 * pow(d7, 4) * pow(ta1ANS, 2) -
                2 * pow(c0, 2) * pow(c2, 3) * d2 * pow(d7, 4) * pow(ta1ANS, 2) - pow(c1, 5) * d3 * pow(d7, 4) * pow(ta1ANS, 2) -
                2 * c0 * pow(c1, 3) * c2 * d3 * pow(d7, 4) * pow(ta1ANS, 2) + 6 * pow(c0, 2) * c1 * pow(c2, 2) * d3 * pow(d7, 4) * pow(ta1ANS, 2) -
                pow(c3, 5) * pow(d3, 3) * pow(d4, 2) * pow(ta1ANS, 3) + 2 * pow(c3, 5) * d2 * pow(d3, 2) * d4 * d5 * pow(ta1ANS, 3) -
                pow(c3, 5) * pow(d2, 2) * d3 * pow(d5, 2) * pow(ta1ANS, 3) - 2 * pow(c3, 5) * d1 * pow(d3, 2) * pow(d5, 2) * pow(ta1ANS, 3) -
                c2 * pow(c3, 4) * d3 * pow(d5, 4) * pow(ta1ANS, 3) - pow(c3, 5) * pow(d2, 2) * d3 * d4 * d6 * pow(ta1ANS, 3) +
                3 * pow(c3, 5) * d1 * pow(d3, 2) * d4 * d6 * pow(ta1ANS, 3) + pow(c3, 5) * pow(d2, 3) * d5 * d6 * pow(ta1ANS, 3) + 2 * pow(c3, 5) * d1 * d2 * d3 * d5 * d6 * pow(ta1ANS, 3) -
                pow(c3, 5) * d0 * pow(d3, 2) * d5 * d6 * pow(ta1ANS, 3) + c2 * pow(c3, 4) * d3 * d4 * pow(d5, 2) * d6 * pow(ta1ANS, 3) +
                c2 * pow(c3, 4) * d2 * pow(d5, 3) * d6 * pow(ta1ANS, 3) - 2 * c1 * pow(c3, 4) * d3 * pow(d5, 3) * d6 * pow(ta1ANS, 3) -
                pow(c3, 5) * d1 * pow(d2, 2) * pow(d6, 2) * pow(ta1ANS, 3) - 2 * pow(c3, 5) * pow(d1, 2) * d3 * pow(d6, 2) * pow(ta1ANS, 3) +
                2 * pow(c3, 5) * d0 * d2 * d3 * pow(d6, 2) * pow(ta1ANS, 3) + c2 * pow(c3, 4) * d3 * pow(d4, 2) * pow(d6, 2) * pow(ta1ANS, 3) +
                3 * c1 * pow(c3, 4) * d3 * d4 * d5 * pow(d6, 2) * pow(ta1ANS, 3) - c2 * pow(c3, 4) * d1 * pow(d5, 2) * pow(d6, 2) * pow(ta1ANS, 3) +
                2 * c1 * pow(c3, 4) * d2 * pow(d5, 2) * pow(d6, 2) * pow(ta1ANS, 3) - c0 * pow(c3, 4) * d3 * pow(d5, 2) * pow(d6, 2) * pow(ta1ANS, 3) -
                c2 * pow(c3, 4) * d1 * d4 * pow(d6, 3) * pow(ta1ANS, 3) - c1 * pow(c3, 4) * d2 * d4 * pow(d6, 3) * pow(ta1ANS, 3) +
                c0 * pow(c3, 4) * d3 * d4 * pow(d6, 3) * pow(ta1ANS, 3) + c2 * pow(c3, 4) * d0 * d5 * pow(d6, 3) * pow(ta1ANS, 3) -
                2 * c1 * pow(c3, 4) * d1 * d5 * pow(d6, 3) * pow(ta1ANS, 3) + c0 * pow(c3, 4) * d2 * d5 * pow(d6, 3) * pow(ta1ANS, 3) +
                c1 * pow(c3, 4) * d0 * pow(d6, 4) * pow(ta1ANS, 3) - c0 * pow(c3, 4) * d1 * pow(d6, 4) * pow(ta1ANS, 3) +
                c2 * pow(c3, 4) * pow(d2, 2) * d3 * d4 * d7 * pow(ta1ANS, 3) - 3 * c2 * pow(c3, 4) * d1 * pow(d3, 2) * d4 * d7 * pow(ta1ANS, 3) -
                2 * c1 * pow(c3, 4) * d2 * pow(d3, 2) * d4 * d7 * pow(ta1ANS, 3) + 2 * c0 * pow(c3, 4) * pow(d3, 3) * d4 * d7 * pow(ta1ANS, 3) -
                c2 * pow(c3, 4) * pow(d2, 3) * d5 * d7 * pow(ta1ANS, 3) - 2 * c2 * pow(c3, 4) * d1 * d2 * d3 * d5 * d7 * pow(ta1ANS, 3) +
                2 * c1 * pow(c3, 4) * pow(d2, 2) * d3 * d5 * d7 * pow(ta1ANS, 3) + c2 * pow(c3, 4) * d0 * pow(d3, 2) * d5 * d7 * pow(ta1ANS, 3) +
                4 * c1 * pow(c3, 4) * d1 * pow(d3, 2) * d5 * d7 * pow(ta1ANS, 3) - 2 * c0 * pow(c3, 4) * d2 * pow(d3, 2) * d5 * d7 * pow(ta1ANS, 3) -
                pow(c2, 2) * pow(c3, 3) * d3 * d4 * pow(d5, 2) * d7 * pow(ta1ANS, 3) - pow(c2, 2) * pow(c3, 3) * d2 * pow(d5, 3) * d7 * pow(ta1ANS, 3) +
                6 * c1 * c2 * pow(c3, 3) * d3 * pow(d5, 3) * d7 * pow(ta1ANS, 3) + 2 * c2 * pow(c3, 4) * d1 * pow(d2, 2) * d6 * d7 * pow(ta1ANS, 3) -
                c1 * pow(c3, 4) * pow(d2, 3) * d6 * d7 * pow(ta1ANS, 3) + 4 * c2 * pow(c3, 4) * pow(d1, 2) * d3 * d6 * d7 * pow(ta1ANS, 3) -
                4 * c2 * pow(c3, 4) * d0 * d2 * d3 * d6 * d7 * pow(ta1ANS, 3) - 2 * c1 * pow(c3, 4) * d1 * d2 * d3 * d6 * d7 * pow(ta1ANS, 3) +
                c0 * pow(c3, 4) * pow(d2, 2) * d3 * d6 * d7 * pow(ta1ANS, 3) + c1 * pow(c3, 4) * d0 * pow(d3, 2) * d6 * d7 * pow(ta1ANS, 3) -
                3 * c0 * pow(c3, 4) * d1 * pow(d3, 2) * d6 * d7 * pow(ta1ANS, 3) - 2 * pow(c2, 2) * pow(c3, 3) * d3 * pow(d4, 2) * d6 * d7 * pow(ta1ANS, 3) -
                8 * c1 * c2 * pow(c3, 3) * d3 * d4 * d5 * d6 * d7 * pow(ta1ANS, 3) + 2 * pow(c2, 2) * pow(c3, 3) * d1 * pow(d5, 2) * d6 * d7 * pow(ta1ANS, 3) -
                7 * c1 * c2 * pow(c3, 3) * d2 * pow(d5, 2) * d6 * d7 * pow(ta1ANS, 3) + 6 * pow(c1, 2) * pow(c3, 3) * d3 * pow(d5, 2) * d6 * d7 * pow(ta1ANS, 3) +
                c0 * c2 * pow(c3, 3) * d3 * pow(d5, 2) * d6 * d7 * pow(ta1ANS, 3) + 3 * pow(c2, 2) * pow(c3, 3) * d1 * d4 * pow(d6, 2) * d7 * pow(ta1ANS, 3) +
                3 * c1 * c2 * pow(c3, 3) * d2 * d4 * pow(d6, 2) * d7 * pow(ta1ANS, 3) - 3 * pow(c1, 2) * pow(c3, 3) * d3 * d4 * pow(d6, 2) * d7 * pow(ta1ANS, 3) -
                5 * c0 * c2 * pow(c3, 3) * d3 * d4 * pow(d6, 2) * d7 * pow(ta1ANS, 3) - 3 * pow(c2, 2) * pow(c3, 3) * d0 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 3) +
                8 * c1 * c2 * pow(c3, 3) * d1 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 3) - 4 * pow(c1, 2) * pow(c3, 3) * d2 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 3) -
                3 * c0 * c2 * pow(c3, 3) * d2 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 3) - c0 * c1 * pow(c3, 3) * d3 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 3) -
                5 * c1 * c2 * pow(c3, 3) * d0 * pow(d6, 3) * d7 * pow(ta1ANS, 3) + 2 * pow(c1, 2) * pow(c3, 3) * d1 * pow(d6, 3) * d7 * pow(ta1ANS, 3) +
                5 * c0 * c2 * pow(c3, 3) * d1 * pow(d6, 3) * d7 * pow(ta1ANS, 3) - pow(c0, 2) * pow(c3, 3) * d3 * pow(d6, 3) * d7 * pow(ta1ANS, 3) -
                pow(c2, 2) * pow(c3, 3) * d1 * pow(d2, 2) * pow(d7, 2) * pow(ta1ANS, 3) + c1 * c2 * pow(c3, 3) * pow(d2, 3) * pow(d7, 2) * pow(ta1ANS, 3) -
                2 * pow(c2, 2) * pow(c3, 3) * pow(d1, 2) * d3 * pow(d7, 2) * pow(ta1ANS, 3) + 2 * pow(c2, 2) * pow(c3, 3) * d0 * d2 * d3 * pow(d7, 2) * pow(ta1ANS, 3) +
                2 * c1 * c2 * pow(c3, 3) * d1 * d2 * d3 * pow(d7, 2) * pow(ta1ANS, 3) - pow(c1, 2) * pow(c3, 3) * pow(d2, 2) * d3 * pow(d7, 2) * pow(ta1ANS, 3) -
                c0 * c2 * pow(c3, 3) * pow(d2, 2) * d3 * pow(d7, 2) * pow(ta1ANS, 3) - c1 * c2 * pow(c3, 3) * d0 * pow(d3, 2) * pow(d7, 2) * pow(ta1ANS, 3) -
                2 * pow(c1, 2) * pow(c3, 3) * d1 * pow(d3, 2) * pow(d7, 2) * pow(ta1ANS, 3) + 3 * c0 * c2 * pow(c3, 3) * d1 * pow(d3, 2) * pow(d7, 2) * pow(ta1ANS, 3) +
                2 * c0 * c1 * pow(c3, 3) * d2 * pow(d3, 2) * pow(d7, 2) * pow(ta1ANS, 3) - pow(c0, 2) * pow(c3, 3) * pow(d3, 3) * pow(d7, 2) * pow(ta1ANS, 3) +
                pow(c2, 3) * pow(c3, 2) * d3 * pow(d4, 2) * pow(d7, 2) * pow(ta1ANS, 3) + 5 * c1 * pow(c2, 2) * pow(c3, 2) * d3 * d4 * d5 * pow(d7, 2) * pow(ta1ANS, 3) -
                pow(c2, 3) * pow(c3, 2) * d1 * pow(d5, 2) * pow(d7, 2) * pow(ta1ANS, 3) + 5 * c1 * pow(c2, 2) * pow(c3, 2) * d2 * pow(d5, 2) * pow(d7, 2) * pow(ta1ANS, 3) -
                12 * pow(c1, 2) * c2 * pow(c3, 2) * d3 * pow(d5, 2) * pow(d7, 2) * pow(ta1ANS, 3) - 3 * pow(c2, 3) * pow(c3, 2) * d1 * d4 * d6 * pow(d7, 2) * pow(ta1ANS, 3) -
                3 * c1 * pow(c2, 2) * pow(c3, 2) * d2 * d4 * d6 * pow(d7, 2) * pow(ta1ANS, 3) + 7 * pow(c1, 2) * c2 * pow(c3, 2) * d3 * d4 * d6 * pow(d7, 2) * pow(ta1ANS, 3) +
                7 * c0 * pow(c2, 2) * pow(c3, 2) * d3 * d4 * d6 * pow(d7, 2) * pow(ta1ANS, 3) + 3 * pow(c2, 3) * pow(c3, 2) * d0 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 3) -
                10 * c1 * pow(c2, 2) * pow(c3, 2) * d1 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 3) + 11 * pow(c1, 2) * c2 * pow(c3, 2) * d2 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 3) +
                3 * c0 * pow(c2, 2) * pow(c3, 2) * d2 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 3) - 6 * pow(c1, 3) * pow(c3, 2) * d3 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 3) +
                4 * c0 * c1 * c2 * pow(c3, 2) * d3 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 3) + 9 * c1 * pow(c2, 2) * pow(c3, 2) * d0 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 3) -
                7 * pow(c1, 2) * c2 * pow(c3, 2) * d1 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 3) - 9 * c0 * pow(c2, 2) * pow(c3, 2) * d1 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 3) +
                2 * pow(c1, 3) * pow(c3, 2) * d2 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 3) + 2 * c0 * pow(c1, 2) * pow(c3, 2) * d3 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 3) +
                4 * pow(c0, 2) * c2 * pow(c3, 2) * d3 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 3) + pow(c2, 4) * c3 * d1 * d4 * pow(d7, 3) * pow(ta1ANS, 3) +
                c1 * pow(c2, 3) * c3 * d2 * d4 * pow(d7, 3) * pow(ta1ANS, 3) - 4 * pow(c1, 2) * pow(c2, 2) * c3 * d3 * d4 * pow(d7, 3) * pow(ta1ANS, 3) -
                3 * c0 * pow(c2, 3) * c3 * d3 * d4 * pow(d7, 3) * pow(ta1ANS, 3) - pow(c2, 4) * c3 * d0 * d5 * pow(d7, 3) * pow(ta1ANS, 3) +
                4 * c1 * pow(c2, 3) * c3 * d1 * d5 * pow(d7, 3) * pow(ta1ANS, 3) - 7 * pow(c1, 2) * pow(c2, 2) * c3 * d2 * d5 * pow(d7, 3) * pow(ta1ANS, 3) -
                c0 * pow(c2, 3) * c3 * d2 * d5 * pow(d7, 3) * pow(ta1ANS, 3) + 10 * pow(c1, 3) * c2 * c3 * d3 * d5 * pow(d7, 3) * pow(ta1ANS, 3) -
                3 * c0 * c1 * pow(c2, 2) * c3 * d3 * d5 * pow(d7, 3) * pow(ta1ANS, 3) - 7 * c1 * pow(c2, 3) * c3 * d0 * d6 * pow(d7, 3) * pow(ta1ANS, 3) +
                8 * pow(c1, 2) * pow(c2, 2) * c3 * d1 * d6 * pow(d7, 3) * pow(ta1ANS, 3) + 7 * c0 * pow(c2, 3) * c3 * d1 * d6 * pow(d7, 3) * pow(ta1ANS, 3) -
                5 * pow(c1, 3) * c2 * c3 * d2 * d6 * pow(d7, 3) * pow(ta1ANS, 3) + 2 * pow(c1, 4) * c3 * d3 * d6 * pow(d7, 3) * pow(ta1ANS, 3) -
                5 * c0 * pow(c1, 2) * c2 * c3 * d3 * d6 * pow(d7, 3) * pow(ta1ANS, 3) - 5 * pow(c0, 2) * pow(c2, 2) * c3 * d3 * d6 * pow(d7, 3) * pow(ta1ANS, 3) +
                2 * c1 * pow(c2, 4) * d0 * pow(d7, 4) * pow(ta1ANS, 3) - 3 * pow(c1, 2) * pow(c2, 3) * d1 * pow(d7, 4) * pow(ta1ANS, 3) -
                2 * c0 * pow(c2, 4) * d1 * pow(d7, 4) * pow(ta1ANS, 3) + 3 * pow(c1, 3) * pow(c2, 2) * d2 * pow(d7, 4) * pow(ta1ANS, 3) -
                3 * pow(c1, 4) * c2 * d3 * pow(d7, 4) * pow(ta1ANS, 3) + 3 * c0 * pow(c1, 2) * pow(c2, 2) * d3 * pow(d7, 4) * pow(ta1ANS, 3) +
                2 * pow(c0, 2) * pow(c2, 3) * d3 * pow(d7, 4) * pow(ta1ANS, 3) + pow(c3, 5) * pow(d3, 3) * d4 * d5 * pow(ta1ANS, 4) -
                2 * pow(c3, 5) * d2 * pow(d3, 2) * pow(d5, 2) * pow(ta1ANS, 4) + pow(c3, 5) * d2 * pow(d3, 2) * d4 * d6 * pow(ta1ANS, 4) +
                2 * pow(c3, 5) * pow(d2, 2) * d3 * d5 * d6 * pow(ta1ANS, 4) - 2 * c2 * pow(c3, 4) * d3 * pow(d5, 3) * d6 * pow(ta1ANS, 4) -
                2 * pow(c3, 5) * d1 * d2 * d3 * pow(d6, 2) * pow(ta1ANS, 4) + pow(c3, 5) * d0 * pow(d3, 2) * pow(d6, 2) * pow(ta1ANS, 4) +
                3 * c2 * pow(c3, 4) * d3 * d4 * d5 * pow(d6, 2) * pow(ta1ANS, 4) + 2 * c2 * pow(c3, 4) * d2 * pow(d5, 2) * pow(d6, 2) * pow(ta1ANS, 4) -
                c1 * pow(c3, 4) * d3 * pow(d5, 2) * pow(d6, 2) * pow(ta1ANS, 4) - c2 * pow(c3, 4) * d2 * d4 * pow(d6, 3) * pow(ta1ANS, 4) +
                c1 * pow(c3, 4) * d3 * d4 * pow(d6, 3) * pow(ta1ANS, 4) - 2 * c2 * pow(c3, 4) * d1 * d5 * pow(d6, 3) * pow(ta1ANS, 4) +
                c1 * pow(c3, 4) * d2 * d5 * pow(d6, 3) * pow(ta1ANS, 4) + c2 * pow(c3, 4) * d0 * pow(d6, 4) * pow(ta1ANS, 4) - c1 * pow(c3, 4) * d1 * pow(d6, 4) * pow(ta1ANS, 4) -
                c2 * pow(c3, 4) * d2 * pow(d3, 2) * d4 * d7 * pow(ta1ANS, 4) - c1 * pow(c3, 4) * pow(d3, 3) * d4 * d7 * pow(ta1ANS, 4) -
                2 * c2 * pow(c3, 4) * pow(d2, 2) * d3 * d5 * d7 * pow(ta1ANS, 4) + 4 * c1 * pow(c3, 4) * d2 * pow(d3, 2) * d5 * d7 * pow(ta1ANS, 4) -
                c0 * pow(c3, 4) * pow(d3, 3) * d5 * d7 * pow(ta1ANS, 4) + 2 * pow(c2, 2) * pow(c3, 3) * d3 * pow(d5, 3) * d7 * pow(ta1ANS, 4) +
                4 * c2 * pow(c3, 4) * d1 * d2 * d3 * d6 * d7 * pow(ta1ANS, 4) - 2 * c1 * pow(c3, 4) * pow(d2, 2) * d3 * d6 * d7 * pow(ta1ANS, 4) -
                2 * c2 * pow(c3, 4) * d0 * pow(d3, 2) * d6 * d7 * pow(ta1ANS, 4) - c0 * pow(c3, 4) * d2 * pow(d3, 2) * d6 * d7 * pow(ta1ANS, 4) -
                6 * pow(c2, 2) * pow(c3, 3) * d3 * d4 * d5 * d6 * d7 * pow(ta1ANS, 4) - 4 * pow(c2, 2) * pow(c3, 3) * d2 * pow(d5, 2) * d6 * d7 * pow(ta1ANS, 4) +
                8 * c1 * c2 * pow(c3, 3) * d3 * pow(d5, 2) * d6 * d7 * pow(ta1ANS, 4) + 3 * pow(c2, 2) * pow(c3, 3) * d2 * d4 * pow(d6, 2) * d7 * pow(ta1ANS, 4) -
                6 * c1 * c2 * pow(c3, 3) * d3 * d4 * pow(d6, 2) * d7 * pow(ta1ANS, 4) + 6 * pow(c2, 2) * pow(c3, 3) * d1 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 4) -
                7 * c1 * c2 * pow(c3, 3) * d2 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 4) + 2 * pow(c1, 2) * pow(c3, 3) * d3 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 4) -
                3 * c0 * c2 * pow(c3, 3) * d3 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 4) - 4 * pow(c2, 2) * pow(c3, 3) * d0 * pow(d6, 3) * d7 * pow(ta1ANS, 4) +
                6 * c1 * c2 * pow(c3, 3) * d1 * pow(d6, 3) * d7 * pow(ta1ANS, 4) - pow(c1, 2) * pow(c3, 3) * d2 * pow(d6, 3) * d7 * pow(ta1ANS, 4) +
                c0 * c2 * pow(c3, 3) * d2 * pow(d6, 3) * d7 * pow(ta1ANS, 4) - c0 * c1 * pow(c3, 3) * d3 * pow(d6, 3) * d7 * pow(ta1ANS, 4) -
                2 * pow(c2, 2) * pow(c3, 3) * d1 * d2 * d3 * pow(d7, 2) * pow(ta1ANS, 4) + 2 * c1 * c2 * pow(c3, 3) * pow(d2, 2) * d3 * pow(d7, 2) * pow(ta1ANS, 4) +
                pow(c2, 2) * pow(c3, 3) * d0 * pow(d3, 2) * pow(d7, 2) * pow(ta1ANS, 4) - 2 * pow(c1, 2) * pow(c3, 3) * d2 * pow(d3, 2) * pow(d7, 2) * pow(ta1ANS, 4) +
                c0 * c2 * pow(c3, 3) * d2 * pow(d3, 2) * pow(d7, 2) * pow(ta1ANS, 4) + c0 * c1 * pow(c3, 3) * pow(d3, 3) * pow(d7, 2) * pow(ta1ANS, 4) +
                3 * pow(c2, 3) * pow(c3, 2) * d3 * d4 * d5 * pow(d7, 2) * pow(ta1ANS, 4) + 2 * pow(c2, 3) * pow(c3, 2) * d2 * pow(d5, 2) * pow(d7, 2) * pow(ta1ANS, 4) -
                7 * c1 * pow(c2, 2) * pow(c3, 2) * d3 * pow(d5, 2) * pow(d7, 2) * pow(ta1ANS, 4) - 3 * pow(c2, 3) * pow(c3, 2) * d2 * d4 * d6 * pow(d7, 2) * pow(ta1ANS, 4) +
                9 * c1 * pow(c2, 2) * pow(c3, 2) * d3 * d4 * d6 * pow(d7, 2) * pow(ta1ANS, 4) - 6 * pow(c2, 3) * pow(c3, 2) * d1 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 4) +
                11 * c1 * pow(c2, 2) * pow(c3, 2) * d2 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 4) - 10 * pow(c1, 2) * c2 * pow(c3, 2) * d3 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 4) +
                6 * c0 * pow(c2, 2) * pow(c3, 2) * d3 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 4) + 6 * pow(c2, 3) * pow(c3, 2) * d0 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 4) -
                12 * c1 * pow(c2, 2) * pow(c3, 2) * d1 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 4) +
                5 * pow(c1, 2) * c2 * pow(c3, 2) * d2 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 4) - 3 * c0 * pow(c2, 2) * pow(c3, 2) * d2 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 4) -
                pow(c1, 3) * pow(c3, 2) * d3 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 4) + 6 * c0 * c1 * c2 * pow(c3, 2) * d3 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 4) +
                pow(c2, 4) * c3 * d2 * d4 * pow(d7, 3) * pow(ta1ANS, 4) - 4 * c1 * pow(c2, 3) * c3 * d3 * d4 * pow(d7, 3) * pow(ta1ANS, 4) +
                2 * pow(c2, 4) * c3 * d1 * d5 * pow(d7, 3) * pow(ta1ANS, 4) - 5 * c1 * pow(c2, 3) * c3 * d2 * d5 * pow(d7, 3) * pow(ta1ANS, 4) +
                8 * pow(c1, 2) * pow(c2, 2) * c3 * d3 * d5 * pow(d7, 3) * pow(ta1ANS, 4) - 3 * c0 * pow(c2, 3) * c3 * d3 * d5 * pow(d7, 3) * pow(ta1ANS, 4) -
                4 * pow(c2, 4) * c3 * d0 * d6 * pow(d7, 3) * pow(ta1ANS, 4) + 10 * c1 * pow(c2, 3) * c3 * d1 * d6 * pow(d7, 3) * pow(ta1ANS, 4) -
                7 * pow(c1, 2) * pow(c2, 2) * c3 * d2 * d6 * pow(d7, 3) * pow(ta1ANS, 4) + 3 * c0 * pow(c2, 3) * c3 * d2 * d6 * pow(d7, 3) * pow(ta1ANS, 4) +
                4 * pow(c1, 3) * c2 * c3 * d3 * d6 * pow(d7, 3) * pow(ta1ANS, 4) - 9 * c0 * c1 * pow(c2, 2) * c3 * d3 * d6 * pow(d7, 3) * pow(ta1ANS, 4) +
                pow(c2, 5) * d0 * pow(d7, 4) * pow(ta1ANS, 4) - 3 * c1 * pow(c2, 4) * d1 * pow(d7, 4) * pow(ta1ANS, 4) +
                3 * pow(c1, 2) * pow(c2, 3) * d2 * pow(d7, 4) * pow(ta1ANS, 4) - c0 * pow(c2, 4) * d2 * pow(d7, 4) * pow(ta1ANS, 4) -
                3 * pow(c1, 3) * pow(c2, 2) * d3 * pow(d7, 4) * pow(ta1ANS, 4) + 4 * c0 * c1 * pow(c2, 3) * d3 * pow(d7, 4) * pow(ta1ANS, 4) -
                pow(c3, 5) * pow(d3, 3) * pow(d5, 2) * pow(ta1ANS, 5) + pow(c3, 5) * pow(d3, 3) * d4 * d6 * pow(ta1ANS, 5) +
                pow(c3, 5) * d2 * pow(d3, 2) * d5 * d6 * pow(ta1ANS, 5) - pow(c3, 5) * d1 * pow(d3, 2) * pow(d6, 2) * pow(ta1ANS, 5) -
                c2 * pow(c3, 4) * d3 * pow(d5, 2) * pow(d6, 2) * pow(ta1ANS, 5) + c2 * pow(c3, 4) * d3 * d4 * pow(d6, 3) * pow(ta1ANS, 5) +
                c2 * pow(c3, 4) * d2 * d5 * pow(d6, 3) * pow(ta1ANS, 5) - c2 * pow(c3, 4) * d1 * pow(d6, 4) * pow(ta1ANS, 5) -
                c2 * pow(c3, 4) * pow(d3, 3) * d4 * d7 * pow(ta1ANS, 5) - c2 * pow(c3, 4) * d2 * pow(d3, 2) * d5 * d7 * pow(ta1ANS, 5) +
                2 * c1 * pow(c3, 4) * pow(d3, 3) * d5 * d7 * pow(ta1ANS, 5) + 2 * c2 * pow(c3, 4) * d1 * pow(d3, 2) * d6 * d7 * pow(ta1ANS, 5) -
                c1 * pow(c3, 4) * d2 * pow(d3, 2) * d6 * d7 * pow(ta1ANS, 5) - c0 * pow(c3, 4) * pow(d3, 3) * d6 * d7 * pow(ta1ANS, 5) +
                2 * pow(c2, 2) * pow(c3, 3) * d3 * pow(d5, 2) * d6 * d7 * pow(ta1ANS, 5) - 3 * pow(c2, 2) * pow(c3, 3) * d3 * d4 * pow(d6, 2) * d7 * pow(ta1ANS, 5) -
                3 * pow(c2, 2) * pow(c3, 3) * d2 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 5) + 2 * c1 * c2 * pow(c3, 3) * d3 * d5 * pow(d6, 2) * d7 * pow(ta1ANS, 5) +
                4 * pow(c2, 2) * pow(c3, 3) * d1 * pow(d6, 3) * d7 * pow(ta1ANS, 5) - c1 * c2 * pow(c3, 3) * d2 * pow(d6, 3) * d7 * pow(ta1ANS, 5) -
                c0 * c2 * pow(c3, 3) * d3 * pow(d6, 3) * d7 * pow(ta1ANS, 5) - pow(c2, 2) * pow(c3, 3) * d1 * pow(d3, 2) * pow(d7, 2) * pow(ta1ANS, 5) +
                c1 * c2 * pow(c3, 3) * d2 * pow(d3, 2) * pow(d7, 2) * pow(ta1ANS, 5) - pow(c1, 2) * pow(c3, 3) * pow(d3, 3) * pow(d7, 2) * pow(ta1ANS, 5) +
                c0 * c2 * pow(c3, 3) * pow(d3, 3) * pow(d7, 2) * pow(ta1ANS, 5) - pow(c2, 3) * pow(c3, 2) * d3 * pow(d5, 2) * pow(d7, 2) * pow(ta1ANS, 5) +
                3 * pow(c2, 3) * pow(c3, 2) * d3 * d4 * d6 * pow(d7, 2) * pow(ta1ANS, 5) + 3 * pow(c2, 3) * pow(c3, 2) * d2 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 5) -
                4 * c1 * pow(c2, 2) * pow(c3, 2) * d3 * d5 * d6 * pow(d7, 2) * pow(ta1ANS, 5) - 6 * pow(c2, 3) * pow(c3, 2) * d1 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 5) +
                3 * c1 * pow(c2, 2) * pow(c3, 2) * d2 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 5) - pow(c1, 2) * c2 * pow(c3, 2) * d3 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 5) +
                3 * c0 * pow(c2, 2) * pow(c3, 2) * d3 * pow(d6, 2) * pow(d7, 2) * pow(ta1ANS, 5) - pow(c2, 4) * c3 * d3 * d4 * pow(d7, 3) * pow(ta1ANS, 5) -
                pow(c2, 4) * c3 * d2 * d5 * pow(d7, 3) * pow(ta1ANS, 5) + 2 * c1 * pow(c2, 3) * c3 * d3 * d5 * pow(d7, 3) * pow(ta1ANS, 5) +
                4 * pow(c2, 4) * c3 * d1 * d6 * pow(d7, 3) * pow(ta1ANS, 5) - 3 * c1 * pow(c2, 3) * c3 * d2 * d6 * pow(d7, 3) * pow(ta1ANS, 5) +
                2 * pow(c1, 2) * pow(c2, 2) * c3 * d3 * d6 * pow(d7, 3) * pow(ta1ANS, 5) - 3 * c0 * pow(c2, 3) * c3 * d3 * d6 * pow(d7, 3) * pow(ta1ANS, 5) -
                pow(c2, 5) * d1 * pow(d7, 4) * pow(ta1ANS, 5) + c1 * pow(c2, 4) * d2 * pow(d7, 4) * pow(ta1ANS, 5) -
                pow(c1, 2) * pow(c2, 3) * d3 * pow(d7, 4) * pow(ta1ANS, 5) + c0 * pow(c2, 4) * d3 * pow(d7, 4) * pow(ta1ANS, 5)) /
                (pow(c3, 5) * pow(d3, 2) * pow(d4, 3) - pow(c3, 5) * d2 * d3 * pow(d4, 2) * d5 + pow(c3, 5) * d1 * d3 * d4 * pow(d5, 2) - pow(c3, 5) * d0 * d3 * pow(d5, 3) +
                pow(c3, 5) * pow(d2, 2) * pow(d4, 2) * d6 - 2 * pow(c3, 5) * d1 * d3 * pow(d4, 2) * d6 - pow(c3, 5) * d1 * d2 * d4 * d5 * d6 + 3 * pow(c3, 5) * d0 * d3 * d4 * d5 * d6 +
                pow(c3, 5) * d0 * d2 * pow(d5, 2) * d6 + pow(c3, 5) * pow(d1, 2) * d4 * pow(d6, 2) - 2 * pow(c3, 5) * d0 * d2 * d4 * pow(d6, 2) - pow(c3, 5) * d0 * d1 * d5 * pow(d6, 2) +
                pow(c3, 5) * pow(d0, 2) * pow(d6, 3) - c2 * pow(c3, 4) * pow(d2, 2) * pow(d4, 2) * d7 + 2 * c2 * pow(c3, 4) * d1 * d3 * pow(d4, 2) * d7 +
                c1 * pow(c3, 4) * d2 * d3 * pow(d4, 2) * d7 - 3 * c0 * pow(c3, 4) * pow(d3, 2) * pow(d4, 2) * d7 + c2 * pow(c3, 4) * d1 * d2 * d4 * d5 * d7 -
                3 * c2 * pow(c3, 4) * d0 * d3 * d4 * d5 * d7 - 2 * c1 * pow(c3, 4) * d1 * d3 * d4 * d5 * d7 + 2 * c0 * pow(c3, 4) * d2 * d3 * d4 * d5 * d7 - c2 * pow(c3, 4) * d0 * d2 * pow(d5, 2) * d7 +
                3 * c1 * pow(c3, 4) * d0 * d3 * pow(d5, 2) * d7 - c0 * pow(c3, 4) * d1 * d3 * pow(d5, 2) * d7 - 2 * c2 * pow(c3, 4) * pow(d1, 2) * d4 * d6 * d7 +
                4 * c2 * pow(c3, 4) * d0 * d2 * d4 * d6 * d7 + c1 * pow(c3, 4) * d1 * d2 * d4 * d6 * d7 - 2 * c0 * pow(c3, 4) * pow(d2, 2) * d4 * d6 * d7 - 3 * c1 * pow(c3, 4) * d0 * d3 * d4 * d6 * d7 +
                4 * c0 * pow(c3, 4) * d1 * d3 * d4 * d6 * d7 + 2 * c2 * pow(c3, 4) * d0 * d1 * d5 * d6 * d7 - 2 * c1 * pow(c3, 4) * d0 * d2 * d5 * d6 * d7 + c0 * pow(c3, 4) * d1 * d2 * d5 * d6 * d7 -
                3 * c0 * pow(c3, 4) * d0 * d3 * d5 * d6 * d7 - 3 * c2 * pow(c3, 4) * pow(d0, 2) * pow(d6, 2) * d7 + c1 * pow(c3, 4) * d0 * d1 * pow(d6, 2) * d7 -
                c0 * pow(c3, 4) * pow(d1, 2) * pow(d6, 2) * d7 + 2 * c0 * pow(c3, 4) * d0 * d2 * pow(d6, 2) * d7 + pow(c2, 2) * pow(c3, 3) * pow(d1, 2) * d4 * pow(d7, 2) -
                2 * pow(c2, 2) * pow(c3, 3) * d0 * d2 * d4 * pow(d7, 2) - c1 * c2 * pow(c3, 3) * d1 * d2 * d4 * pow(d7, 2) + 2 * c0 * c2 * pow(c3, 3) * pow(d2, 2) * d4 * pow(d7, 2) +
                3 * c1 * c2 * pow(c3, 3) * d0 * d3 * d4 * pow(d7, 2) + pow(c1, 2) * pow(c3, 3) * d1 * d3 * d4 * pow(d7, 2) - 4 * c0 * c2 * pow(c3, 3) * d1 * d3 * d4 * pow(d7, 2) -
                2 * c0 * c1 * pow(c3, 3) * d2 * d3 * d4 * pow(d7, 2) + 3 * pow(c0, 2) * pow(c3, 3) * pow(d3, 2) * d4 * pow(d7, 2) - pow(c2, 2) * pow(c3, 3) * d0 * d1 * d5 * pow(d7, 2) +
                2 * c1 * c2 * pow(c3, 3) * d0 * d2 * d5 * pow(d7, 2) - c0 * c2 * pow(c3, 3) * d1 * d2 * d5 * pow(d7, 2) - 3 * pow(c1, 2) * pow(c3, 3) * d0 * d3 * d5 * pow(d7, 2) +
                3 * c0 * c2 * pow(c3, 3) * d0 * d3 * d5 * pow(d7, 2) + 2 * c0 * c1 * pow(c3, 3) * d1 * d3 * d5 * pow(d7, 2) - pow(c0, 2) * pow(c3, 3) * d2 * d3 * d5 * pow(d7, 2) +
                3 * pow(c2, 2) * pow(c3, 3) * pow(d0, 2) * d6 * pow(d7, 2) - 2 * c1 * c2 * pow(c3, 3) * d0 * d1 * d6 * pow(d7, 2) + 2 * c0 * c2 * pow(c3, 3) * pow(d1, 2) * d6 * pow(d7, 2) +
                pow(c1, 2) * pow(c3, 3) * d0 * d2 * d6 * pow(d7, 2) - 4 * c0 * c2 * pow(c3, 3) * d0 * d2 * d6 * pow(d7, 2) - c0 * c1 * pow(c3, 3) * d1 * d2 * d6 * pow(d7, 2) +
                pow(c0, 2) * pow(c3, 3) * pow(d2, 2) * d6 * pow(d7, 2) + 3 * c0 * c1 * pow(c3, 3) * d0 * d3 * d6 * pow(d7, 2) - 2 * pow(c0, 2) * pow(c3, 3) * d1 * d3 * d6 * pow(d7, 2) -
                pow(c2, 3) * pow(c3, 2) * pow(d0, 2) * pow(d7, 3) + c1 * pow(c2, 2) * pow(c3, 2) * d0 * d1 * pow(d7, 3) -
                c0 * pow(c2, 2) * pow(c3, 2) * pow(d1, 2) * pow(d7, 3) - pow(c1, 2) * c2 * pow(c3, 2) * d0 * d2 * pow(d7, 3) +
                2 * c0 * pow(c2, 2) * pow(c3, 2) * d0 * d2 * pow(d7, 3) + c0 * c1 * c2 * pow(c3, 2) * d1 * d2 * pow(d7, 3) - pow(c0, 2) * c2 * pow(c3, 2) * pow(d2, 2) * pow(d7, 3) +
                pow(c1, 3) * pow(c3, 2) * d0 * d3 * pow(d7, 3) - 3 * c0 * c1 * c2 * pow(c3, 2) * d0 * d3 * pow(d7, 3) - c0 * pow(c1, 2) * pow(c3, 2) * d1 * d3 * pow(d7, 3) +
                2 * pow(c0, 2) * c2 * pow(c3, 2) * d1 * d3 * pow(d7, 3) + pow(c0, 2) * c1 * pow(c3, 2) * d2 * d3 * pow(d7, 3) - pow(c0, 3) * pow(c3, 2) * pow(d3, 2) * pow(d7, 3));
                tb1 = ta1 + (a0 * pi * alpha) / (Jpeak * pi * sa + 4 * Jpeak * sa * alpha - Jpeak * pi * sa * alpha);
                ta2 = (1 - alpha) * ta1 / alpha;
                tb2 = (1 - alpha) * tb1 / alpha;
                tc2 = (1 - alpha) * tc1 / alpha;
                vector<double> res = { ta1,ta2,ta3,tb1,tb2,t4,tc1,tc2,tc3 };
                res = isValid(res);
                if (res.size() == 9)
                {
                    res.push_back(sa);
                    res.push_back(sv);
                    ans.push_back(res);
                }
        }
    }
    return ans;
};
vector<vector<double>> TrigonometricOTG::computeTypeII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv) {
   /* cout << "II" << endl;*/
    double Jpeak = mJ;
    double Apeak = sv * mA;
    vector<vector<double>> ans = {};
    double c0, c1, c2, c3, c4;
    c4 = -0.5 * (pow(Jpeak, 2) * pow(sa, 2) * pow(1 + alpha, 2) * pow(pi + 4 * alpha - pi * alpha, 2)) / (Apeak * pow(pi, 2) * pow(alpha, 4));
    c3 = ((2 * a0 - Apeak) * Jpeak * sa * (pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2)) / (Apeak * pi * pow(alpha, 3));
    c2 = ((Apeak * sa * pow(1 + alpha, 2)) / sv + (a0 * (24 * pi * (-1 + alpha) * pow(alpha, 2) - 96 * pow(alpha, 3) - 22 * pow(pi, 2) * alpha * pow(1 + alpha, 2) +
        pow(pi, 3) * (-1 + alpha) * (3 + 2 * alpha) * (2 + 3 * alpha))) / (pow(pi, 2) * (pi * (-1 + alpha) - 4 * alpha)) -
        ((1 + alpha) * (5 * pow(a0, 2) * pi * (1 + alpha) + 2 * Jpeak * sa * v0 * (pi + 4 * alpha - pi * alpha))) / (Apeak * pi)) / (2. * pow(alpha, 2));
    c1 = (2 * pow(a0, 3) * pow(pi, 2) * sv * (pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2) + 4 * Apeak * Jpeak * pi * sa * sv * v0 * (1 + alpha) * pow(pi + 4 * alpha - pi * alpha, 2) -
        2 * a0 * pi * sa * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha) * (pow(Apeak, 2) * pi * (1 + alpha) + 2 * Jpeak * sv * v0 * (-pi + (-4 + pi) * alpha)) +
        pow(a0, 2) * Apeak * sv * (-24 * pi * (-1 + alpha) * pow(alpha, 2) + 96 * pow(alpha, 3) + 14 * pow(pi, 2) * alpha * pow(1 + alpha, 2) -
            pow(pi, 3) * (-1 + alpha) * (4 + alpha * (9 + 4 * alpha)))) / (2. * Apeak * Jpeak * pi * sa * sv * pow(pi * (-1 + alpha) - 4 * alpha, 2) * alpha);
    c0 = (-3 * pow(a0, 4) * pow(pi, 2) * sv * (pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2) - 24 * a0 * Apeak * Jpeak * pi * sa * sv * v0 * (1 + alpha) * pow(pi + 4 * alpha - pi * alpha, 2) +
        4 * pow(a0, 3) * Apeak * sv * (24 * pi * (-1 + alpha) * pow(alpha, 2) - 96 * pow(alpha, 3) - 6 * pow(pi, 2) * alpha * pow(1 + alpha, 2) +
            pow(pi, 3) * (-1 + alpha) * (2 + alpha) * (1 + 2 * alpha)) + 6 * pow(a0, 2) * pi * sa * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha) *
        (pow(Apeak, 2) * pi * (1 + alpha) + 2 * Jpeak * sv * v0 * (-pi + (-4 + pi) * alpha)) +
        12 * Jpeak * pow(pi * sa * (-1 + alpha) - 4 * sa * alpha, 2) * (-(pow(Apeak, 2) * pi * v0 * (1 + alpha)) + 2 * Apeak * Jpeak * (p0 - pG) * sv * (-pi + (-4 + pi) * alpha) +
            Jpeak * sv * pow(v0, 2) * (pi + 4 * alpha - pi * alpha))) / (24. * Apeak * pow(Jpeak, 2) * pow(sa, 2) * sv * pow(pi * (-1 + alpha) - 4 * alpha, 3));

    Eigen::VectorXd coeff(5);
    coeff << c0,c1,c2,c3,c4;
    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeff);
    std::vector<double> realRoots;
    solver.realRoots(realRoots);
    vector<double> temp = isValid(realRoots);
    double ta3 = 0;
    double tc1 = (Apeak * pi * alpha) / (Jpeak * pi * sv + 4 * Jpeak * sv * alpha - Jpeak * pi * sv * alpha);
    double tc2 = (1 - alpha) * tc1 / alpha;
    double ta1, tc3, tb1, tb2, ta2;
    double t4 = 0;
    for (double d : temp)
    {
        ta1 = d;
        tc3 = -((v0 + (2 * a0 * ta1 * (1 + alpha)) / alpha + (pow(a0, 2) * pi * (1 + alpha)) / (2. * Jpeak * sa * (pi + 4 * alpha - pi * alpha)) +
            (pow(Apeak, 2) * pi * (1 + alpha)) / (Jpeak * sv * (pi + 4 * alpha - pi * alpha)) + (Jpeak * sa * pow(ta1, 2) * (pi + 4 * alpha - (-4 + pi) * pow(alpha, 2))) / (pi * pow(alpha, 2))) /
            Apeak);
        tb1 = ta1 + (a0 * pi * alpha) / (Jpeak * pi * sa + 4 * Jpeak * sa * alpha - Jpeak * pi * sa * alpha);
        ta2 = (1 - alpha) * ta1 / alpha;
        tb2 = (1 - alpha) * tb1 / alpha;
        vector<double> res = { ta1,ta2,ta3,tb1,tb2,t4,tc1,tc2,tc3 };
        res = isValid(res);
        if (res.size() == 9)
        {
            res.push_back(sa);
            res.push_back(sv);
            ans.push_back(res);
        }
    }
    return ans;
};
vector<vector<double>> TrigonometricOTG::computeTypeIII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv) {
    /*cout << "III" << endl;*/
    double Jpeak = mJ;
    double Apeak = sa * mA;
    vector<vector<double>> ans = {};
    double c0, c1, c2, c3, c4;
    c4 = (pow(Jpeak, 2) * pow(sv, 2) * pow(1 + alpha, 2) * pow(pi + 4 * alpha - pi * alpha, 2)) / (2. * Apeak * pow(pi, 2) * pow(alpha, 4));
    c3 = (Jpeak * sv * (pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2)) / (pi * pow(alpha, 3));
    c2 = -0.5 * (Apeak * sv * pow(1 + alpha, 2)) / (sa * pow(alpha, 2));
    c1 = 0;
    c0 = (-3 * pow(a0, 4) * pow(pi, 2) * (pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2) -
        12 * Jpeak * sa * pow(pi * (-1 + alpha) - 4 * alpha, 2) * (-2 * Apeak * Jpeak * (p0 - pG) * sa * (pi * (-1 + alpha) - 4 * alpha) + Jpeak * sa * pow(v0, 2) * (pi * (-1 + alpha) - 4 * alpha) +
            pow(Apeak, 2) * pi * v0 * (1 + alpha)) + 4 * pow(a0, 3) * Apeak *
        (24 * pi * (-1 + alpha) * pow(alpha, 2) - 96 * pow(alpha, 3) - 6 * pow(pi, 2) * alpha * pow(1 + alpha, 2) + pow(pi, 3) * (-1 + alpha) * (2 + alpha) * (1 + 2 * alpha)) +
        12 * a0 * (2 * Apeak * Jpeak * pi * sa * v0 * (1 + alpha) * pow(pi + 4 * alpha - pi * alpha, 2) +
            pow(Apeak, 3) * alpha * (pow(pi, 3) * (-1 + alpha) + 24 * pi * (-1 + alpha) * alpha - 96 * pow(alpha, 2) + 2 * pow(pi, 2) * pow(1 + alpha, 2))) -
        6 * pow(a0, 2) * (2 * Jpeak * pi * sa * v0 * (1 + alpha) * pow(pi + 4 * alpha - pi * alpha, 2) +
            pow(Apeak, 2) * (48 * pi * (-1 + alpha) * pow(alpha, 2) - 192 * pow(alpha, 3) + pow(pi, 3) * (-1 + alpha) * (1 + alpha * (4 + alpha))))) /
        (24. * Apeak * pow(Jpeak, 2) * pow(sa, 2) * pow(pi * (-1 + alpha) - 4 * alpha, 3));

    Eigen::VectorXd coeff(5);
    coeff << c0, c1, c2, c3, c4;
    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeff);
    std::vector<double> realRoots;
    solver.realRoots(realRoots);
    vector<double> temp = isValid(realRoots);
    double tc3 = 0;
    double t4 = 0;
    double ta1 = ((a0 - Apeak) * pi * alpha) / (Jpeak * sa * (pi * (-1 + alpha) - 4 * alpha));
    double tb1 = (Apeak * pi * alpha) / (Jpeak * pi * sa + 4 * Jpeak * sa * alpha - Jpeak * pi * sa * alpha);
    double ta2 = (1 - alpha) * ta1 / alpha;
    double tb2 = (1 - alpha) * tb1 / alpha;
    double tc1, tc2, ta3;
    for (double d : temp)
    {
        tc1 = d;
        tc2 = (1 - alpha) * tc1 / alpha;
        ta3 = (-((pow(a0, 2) - 2 * pow(Apeak, 2)) * pow(pi, 2) * pow(alpha, 2) * (1 + alpha)) + 2 * Jpeak * pi * sa * v0 * pow(alpha, 2) * (pi + 4 * alpha - pi * alpha) +
            2 * pow(Jpeak, 2) * sa * sv * pow(tc1, 2) * (1 + alpha) * pow(pi + 4 * alpha - pi * alpha, 2)) / (2. * Apeak * Jpeak * pi * sa * (pi * (-1 + alpha) - 4 * alpha) * pow(alpha, 2));
        vector<double> res = { ta1,ta2,ta3,tb1,tb2,t4,tc1,tc2,tc3 };
        res = isValid(res);
        if (res.size() == 9)
        {
            res.push_back(sa);
            res.push_back(sv);
            ans.push_back(res);
        }
    }
    return ans;
};
vector<vector<double>> TrigonometricOTG::computeTypeIV(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv) {
   /* cout << "IV" << endl;*/
    double Jpeak = mJ;
    double Apeak1 = sa * mA;
    double Apeak2 = sv * mA;
    vector<vector<double>> ans = {};
    double c2, c1, c0;
    c2 = (Apeak1 * (-Apeak1 + Apeak2)) / (2. * Apeak2);
    c1 = (2 * pow(Apeak1, 3) * pi * sv * (1 + alpha) - 3 * pow(Apeak1, 2) * Apeak2 * pi * sv * (1 + alpha) -
        Apeak1 * (2 * Jpeak * sa * sv * v0 * (pi * (-1 + alpha) - 4 * alpha) + pow(Apeak2, 2) * pi * sa * (1 + alpha) + pow(a0, 2) * pi * sv * (1 + alpha)) +
        Apeak2 * sv * (pow(a0, 2) * pi * (1 + alpha) + 2 * Jpeak * sa * v0 * (-pi + (-4 + pi) * alpha))) / (2. * Apeak2 * Jpeak * sa * sv * (pi * (-1 + alpha) - 4 * alpha));
    c0 = (-3 * pow(a0, 4) * pow(pi, 2) * sv * (pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2) +
        4 * pow(a0, 3) * Apeak2 * sv * (24 * pi * (-1 + alpha) * pow(alpha, 2) - 96 * pow(alpha, 3) - 6 * pow(pi, 2) * alpha * pow(1 + alpha, 2) +
            pow(pi, 3) * (-1 + alpha) * (2 + alpha) * (1 + 2 * alpha)) + 12 * a0 * Apeak2 * sv *
        (2 * Jpeak * pi * sa * v0 * (1 + alpha) * pow(pi + 4 * alpha - pi * alpha, 2) +
            pow(Apeak1, 2) * alpha * (pow(pi, 3) * (-1 + alpha) + 24 * pi * (-1 + alpha) * alpha - 96 * pow(alpha, 2) + 2 * pow(pi, 2) * pow(1 + alpha, 2))) -
        12 * (pi * (-1 + alpha) - 4 * alpha) * (4 * Apeak1 * Apeak2 * Jpeak * pi * sa * sv * v0 * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha) + pow(Apeak1, 4) * pow(pi, 2) * sv * pow(1 + alpha, 2) -
            2 * pow(Apeak1, 3) * Apeak2 * pow(pi, 2) * sv * pow(1 + alpha, 2) -
            pow(Apeak1, 2) * pi * sa * (1 + alpha) * (pow(Apeak2, 2) * pi * (1 + alpha) + 2 * Jpeak * sv * v0 * (-pi + (-4 + pi) * alpha)) +
            Jpeak * pow(sa, 2) * (pi * (-1 + alpha) - 4 * alpha) * (pow(Apeak2, 2) * pi * v0 * (1 + alpha) - 2 * Apeak2 * Jpeak * (p0 - pG) * sv * (-pi + (-4 + pi) * alpha) +
                Jpeak * sv * pow(v0, 2) * (-pi + (-4 + pi) * alpha))) - 6 * pow(a0, 2) *
        (pow(Apeak2, 2) * pow(pi, 2) * sa * (pi * (-1 + alpha) - 4 * alpha) * pow(1 + alpha, 2) +
            2 * Apeak1 * Apeak2 * sv * (24 * pi * (-1 + alpha) * pow(alpha, 2) - 96 * pow(alpha, 3) - 6 * pow(pi, 2) * alpha * pow(1 + alpha, 2) +
                pow(pi, 3) * (-1 + alpha) * (2 + alpha) * (1 + 2 * alpha)) - 2 * pi * sv * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha) *
            (pow(Apeak1, 2) * pi * (1 + alpha) + Jpeak * sa * v0 * (pi + 4 * alpha - pi * alpha)))) / (24. * Apeak2 * pow(Jpeak, 2) * pow(sa, 2) * sv * pow(pi * (-1 + alpha) - 4 * alpha, 3));

    Eigen::VectorXd coeff(3);
    coeff << c0, c1, c2;
    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeff);
    std::vector<double> realRoots;
    solver.realRoots(realRoots);
    vector<double> temp = isValid(realRoots);

    double tc1 = (Apeak2 * pi * alpha) / (Jpeak * pi * sv + 4 * Jpeak * sv * alpha - Jpeak * pi * sv * alpha);
    double ta1 = ((a0 - Apeak1) * pi * alpha) / (Jpeak * sa * (pi * (-1 + alpha) - 4 * alpha));
    double tb1 = (Apeak1 * pi * alpha) / (Jpeak * pi * sa + 4 * Jpeak * sa * alpha - Jpeak * pi * sa * alpha);
    double ta2 = (1 - alpha) * ta1 / alpha;
    double tb2 = (1 - alpha) * tb1 / alpha;
    double tc2 = (1 - alpha) * tc1 / alpha;
    double t4 = 0;
    double ta3,tc3;
    for (double d : temp)
    {
        ta3 = d;
        tc3 = (2 * pow(Apeak2, 2) * pi * sa * (1 + alpha) - pow(a0, 2) * pi * sv * (1 + alpha) +
            2 * sv * (pow(Apeak1, 2) * pi * (1 + alpha) + Apeak1 * Jpeak * sa * ta3 * (pi + 4 * alpha - pi * alpha) + Jpeak * sa * v0 * (pi + 4 * alpha - pi * alpha))) /
            (2. * Apeak2 * Jpeak * sa * sv * (pi * (-1 + alpha) - 4 * alpha));
        vector<double> res = { ta1,ta2,ta3,tb1,tb2,t4,tc1,tc2,tc3 };
        res = isValid(res);
        if (res.size() == 9)
        {
            res.push_back(sa);
            res.push_back(sv);
            ans.push_back(res);
        }
    }
    return ans;
};
vector<vector<double>> TrigonometricOTG::computeTypeV(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv) {
    /*cout << "V" << endl;*/
    vector<vector<double>> ans = {};
    double Vpeak = mV * sa;
    double Jpeak = mJ;
    vector<double> temp = { (alpha * (2 * a0 * pi * (1 + alpha) + sqrt(2 * pi) * sqrt((1 + alpha) * (pow(a0,2) * pi * (1 + alpha) + 2 * Jpeak * sa * (v0 - Vpeak) * (-pi + (-4 + pi) * alpha))))) /
    (2. * Jpeak * sa * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha)), (alpha * (2 * a0 * pi * (1 + alpha) - sqrt(2 * pi) * sqrt((1 + alpha) * (pow(a0,2) * pi * (1 + alpha) + 2 * Jpeak * sa * (v0 - Vpeak) * (-pi + (-4 + pi) * alpha))))) /
    (2. * Jpeak * sa * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha)) };
    temp = isValid(temp);
    double ta1 = temp.empty() ? -1 : *max_element(temp.begin(), temp.end());
    double tb1 = ta1 + (a0 * pi * alpha) / (Jpeak * pi * sa + 4 * Jpeak * sa * alpha - Jpeak * pi * sa * alpha);
    temp = { -(sqrt(pi) * sqrt((Vpeak * pow(alpha,2)) / (Jpeak * sv * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha)))), sqrt(pi) * sqrt((Vpeak * pow(alpha,2)) / (Jpeak * sv * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha))) };
    temp = isValid(temp);
    double tc1 = temp.empty() ? -1 : *max_element(temp.begin(), temp.end());
    double ta3 = 0;
    double tc3 = 0;
    double ta2 = (1 - alpha) * ta1 / alpha;
    double tb2 = (1 - alpha) * tb1 / alpha;
    double tc2 = (1 - alpha) * tc1 / alpha;
    double t4 = 0;
    double dd = p0 + (3 * (a0 + Jpeak * sa * ((4 * ta1) / pi + ta2)) * pow(ta3 + 2 * tb1 + tb2, 2) + 3 * a0 * (2 * ta1 + ta2) * (2 * ta1 + ta2 + 2 * (ta3 + 2 * tb1 + tb2)) +
        (Jpeak * sa * (24 * (-4 + pow(pi, 2)) * pow(ta1, 3) + 96 * pow(tb1, 3) + 24 * pi * pow(tb1, 2) * tb2 - 6 * pow(pi, 2) * tb1 * pow(2 * tb1 + tb2, 2) +
            3 * pow(pi, 2) * (2 + pi) * ta1 * ta2 * (ta2 + 2 * (ta3 + 2 * tb1 + tb2)) + 3 * pi * pow(ta1, 2) * ((-8 + pi * (8 + pi)) * ta2 + 8 * pi * (ta3 + 2 * tb1 + tb2)) +
            pow(pi, 3) * (pow(ta2, 3) + 3 * pow(ta2, 2) * (ta3 + 2 * tb1 + tb2) - tb2 * (3 * pow(tb1, 2) + 3 * tb1 * tb2 + pow(tb2, 2))))) / pow(pi, 3) +
        (3 * Jpeak * sv * (4 * tc1 + pi * tc2) * (2 * tc1 + tc2 + tc3) * (4 * tc1 + 2 * tc2 + tc3)) / pi + 6 * (2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2) * v0 +
        6 * (t4 + 4 * tc1 + 2 * tc2 + tc3) * (v0 + (2 * a0 * ta1 * (1 + alpha)) / alpha + (pow(a0, 2) * pi * (1 + alpha)) / (2. * Jpeak * sa * (pi + 4 * alpha - pi * alpha)) +
            (Jpeak * sa * pow(ta1, 2) * (pi + 4 * alpha - (-4 + pi) * pow(alpha, 2))) / (pi * pow(alpha, 2)))) / 6.;
    t4 = (pG - dd) / Vpeak;
    vector<double> res = { ta1,ta2,ta3,tb1,tb2,t4,tc1,tc2,tc3 };
    res = isValid(res);
    if (res.size() == 9) {
        res.push_back(sa);
        res.push_back(sv);
        ans.push_back(res);
    }
    return ans;
};
vector<vector<double>> TrigonometricOTG::computeTypeVI(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv) {
    /*cout << "VI" << endl;*/
    vector<vector<double>> ans = {};
    double Vpeak = mV * sa;
    double Apeak = mA * sv;
    double Jpeak = mJ;
    vector<double> temp = { (alpha * (2 * a0 * pi * (1 + alpha) + sqrt(2 * pi) * sqrt((1 + alpha) * (pow(a0,2) * pi * (1 + alpha) + 2 * Jpeak * sa * (v0 - Vpeak) * (-pi + (-4 + pi) * alpha))))) /
    (2. * Jpeak * sa * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha)),(alpha * (2 * a0 * pi * (1 + alpha) -
        sqrt(2 * pi) * sqrt((1 + alpha) * (pow(a0,2) * pi * (1 + alpha) + 2 * Jpeak * sa * (v0 - Vpeak) * (-pi + (-4 + pi) * alpha))))) /
    (2. * Jpeak * sa * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha)) };
    temp = isValid(temp);
    double ta1 = temp.empty() ? -1 : *max_element(temp.begin(), temp.end());
    double tb1 = ta1 + (a0 * pi * alpha) / (Jpeak * pi * sa + 4 * Jpeak * sa * alpha - Jpeak * pi * sa * alpha);
    double tc1 = (Apeak * pi * alpha) / (Jpeak * pi * sv + 4 * Jpeak * sv * alpha - Jpeak * pi * sv * alpha);
    double ta3 = 0;
    double tc3 = -(Vpeak / Apeak) - (tc1 * (1 + alpha)) / alpha;
    double ta2 = (1 - alpha) * ta1 / alpha;
    double tb2 = (1 - alpha) * tb1 / alpha;
    double tc2 = (1 - alpha) * tc1 / alpha;
    double t4 = 0;
    double dd = p0 + (3 * (a0 + Jpeak * sa * ((4 * ta1) / pi + ta2)) * pow(ta3 + 2 * tb1 + tb2, 2) + 3 * a0 * (2 * ta1 + ta2) * (2 * ta1 + ta2 + 2 * (ta3 + 2 * tb1 + tb2)) +
        (Jpeak * sa * (24 * (-4 + pow(pi, 2)) * pow(ta1, 3) + 96 * pow(tb1, 3) + 24 * pi * pow(tb1, 2) * tb2 - 6 * pow(pi, 2) * tb1 * pow(2 * tb1 + tb2, 2) +
            3 * pow(pi, 2) * (2 + pi) * ta1 * ta2 * (ta2 + 2 * (ta3 + 2 * tb1 + tb2)) + 3 * pi * pow(ta1, 2) * ((-8 + pi * (8 + pi)) * ta2 + 8 * pi * (ta3 + 2 * tb1 + tb2)) +
            pow(pi, 3) * (pow(ta2, 3) + 3 * pow(ta2, 2) * (ta3 + 2 * tb1 + tb2) - tb2 * (3 * pow(tb1, 2) + 3 * tb1 * tb2 + pow(tb2, 2))))) / pow(pi, 3) +
        (3 * Jpeak * sv * (4 * tc1 + pi * tc2) * (2 * tc1 + tc2 + tc3) * (4 * tc1 + 2 * tc2 + tc3)) / pi + 6 * (2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2) * v0 +
        6 * (t4 + 4 * tc1 + 2 * tc2 + tc3) * (v0 + (2 * a0 * ta1 * (1 + alpha)) / alpha + (pow(a0, 2) * pi * (1 + alpha)) / (2. * Jpeak * sa * (pi + 4 * alpha - pi * alpha)) +
            (Jpeak * sa * pow(ta1, 2) * (pi + 4 * alpha - (-4 + pi) * pow(alpha, 2))) / (pi * pow(alpha, 2)))) / 6.;
    t4 = (pG - dd) / Vpeak;
    vector<double> res = { ta1,ta2,ta3,tb1,tb2,t4,tc1,tc2,tc3 };
    res = isValid(res);
    if (res.size() == 9) {
        res.push_back(sa);
        res.push_back(sv);
        ans.push_back(res);
    }
    return ans;
};
vector<vector<double>> TrigonometricOTG::computeTypeVII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv) {
    /*cout << "VII" << endl;*/
    vector<vector<double>> ans = {};
    double Vpeak = mV * sa;
    double Apeak = mA * sa;
    double Jpeak = mJ;
    double ta1 = ((a0 - Apeak) * pi * alpha) / (Jpeak * sa * (pi * (-1 + alpha) - 4 * alpha));
    double tb1 = (Apeak * pi * alpha) / (Jpeak * pi * sa + 4 * Jpeak * sa * alpha - Jpeak * pi * sa * alpha);
    vector<double> temp = { -(sqrt(pi) * sqrt((Vpeak * pow(alpha,2)) / (Jpeak * sv * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha)))),
   sqrt(pi) * sqrt((Vpeak * pow(alpha,2)) / (Jpeak * sv * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha))) };
    temp = isValid(temp);
    double tc1 = temp.empty() ? -1 : *max_element(temp.begin(), temp.end());
    double ta3 = (-(pow(a0, 2) * pi * (1 + alpha)) + 2 * pow(Apeak, 2) * pi * (1 + alpha) + 2 * Jpeak * sa * (v0 - Vpeak) * (pi + 4 * alpha - pi * alpha)) / (2. * Apeak * Jpeak * sa * (pi * (-1 + alpha) - 4 * alpha));
    double tc3 = 0;
    double ta2 = (1 - alpha) * ta1 / alpha;
    double tb2 = (1 - alpha) * tb1 / alpha;
    double tc2 = (1 - alpha) * tc1 / alpha;
    double t4 = 0;
    double dd = p0 + (3 * (a0 + Jpeak * sa * ((4 * ta1) / pi + ta2)) * pow(ta3 + 2 * tb1 + tb2, 2) + 3 * a0 * (2 * ta1 + ta2) * (2 * ta1 + ta2 + 2 * (ta3 + 2 * tb1 + tb2)) +
        (Jpeak * sa * (24 * (-4 + pow(pi, 2)) * pow(ta1, 3) + 96 * pow(tb1, 3) + 24 * pi * pow(tb1, 2) * tb2 - 6 * pow(pi, 2) * tb1 * pow(2 * tb1 + tb2, 2) +
            3 * pow(pi, 2) * (2 + pi) * ta1 * ta2 * (ta2 + 2 * (ta3 + 2 * tb1 + tb2)) + 3 * pi * pow(ta1, 2) * ((-8 + pi * (8 + pi)) * ta2 + 8 * pi * (ta3 + 2 * tb1 + tb2)) +
            pow(pi, 3) * (pow(ta2, 3) + 3 * pow(ta2, 2) * (ta3 + 2 * tb1 + tb2) - tb2 * (3 * pow(tb1, 2) + 3 * tb1 * tb2 + pow(tb2, 2))))) / pow(pi, 3) +
        (3 * Jpeak * sv * (4 * tc1 + pi * tc2) * (2 * tc1 + tc2 + tc3) * (4 * tc1 + 2 * tc2 + tc3)) / pi + 6 * (2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2) * v0 +
        (3 * (t4 + 4 * tc1 + 2 * tc2 + tc3) * (pow(a0, 2) * pi * (1 + alpha) - 2 * (pow(Apeak, 2) * pi * (1 + alpha) + Apeak * Jpeak * sa * ta3 * (pi + 4 * alpha - pi * alpha) + Jpeak * sa * v0 * (pi + 4 * alpha - pi * alpha)))) /
        (Jpeak * sa * (pi * (-1 + alpha) - 4 * alpha))) / 6.;
    t4 = (pG - dd) / Vpeak;
    vector<double> res = { ta1,ta2,ta3,tb1,tb2,t4,tc1,tc2,tc3 };
    res = isValid(res);
    if (res.size() == 9) {
        res.push_back(sa);
        res.push_back(sv);
        ans.push_back(res);
    }
    return ans;
};
vector<vector<double>> TrigonometricOTG::computeTypeVIII(double a0, double v0, double p0, double pG, double alpha, double mJ, double mA, double mV, double sa, double sv) {
    /*cout << "VIII" << endl;*/
    vector<vector<double>> ans = {};
    double Vpeak = mV * sa;
    double Apeak1 = mA * sa;
    double Apeak2 = mA * sv;
    double Jpeak = mJ;
    double ta1 = ((a0 - Apeak1) * pi * alpha) / (Jpeak * sa * (pi * (-1 + alpha) - 4 * alpha));
    double tb1 = (Apeak1 * pi * alpha) / (Jpeak * pi * sa + 4 * Jpeak * sa * alpha - Jpeak * pi * sa * alpha);
    double tc1 = (Apeak2 * pi * alpha) / (Jpeak * pi * sv + 4 * Jpeak * sv * alpha - Jpeak * pi * sv * alpha);
    double ta3 = (-(pow(a0, 2) * pi * (1 + alpha)) + 2 * pow(Apeak1, 2) * pi * (1 + alpha) + 2 * Jpeak * sa * (v0 - Vpeak) * (pi + 4 * alpha - pi * alpha)) / (2. * Apeak1 * Jpeak * sa * (pi * (-1 + alpha) - 4 * alpha));
    double tc3 = (pow(Apeak2, 2) * pi * (1 + alpha) + Jpeak * sv * Vpeak * (pi + 4 * alpha - pi * alpha)) / (Apeak2 * Jpeak * sv * (pi * (-1 + alpha) - 4 * alpha));
    double ta2 = (1 - alpha) * ta1 / alpha;
    double tb2 = (1 - alpha) * tb1 / alpha;
    double tc2 = (1 - alpha) * tc1 / alpha;
    double t4 = 0;
    double dd = p0 + (3 * (a0 + Jpeak * sa * ((4 * ta1) / pi + ta2)) * pow(ta3 + 2 * tb1 + tb2, 2) + 3 * a0 * (2 * ta1 + ta2) * (2 * ta1 + ta2 + 2 * (ta3 + 2 * tb1 + tb2)) +
        (Jpeak * sa * (24 * (-4 + pow(pi, 2)) * pow(ta1, 3) + 96 * pow(tb1, 3) + 24 * pi * pow(tb1, 2) * tb2 - 6 * pow(pi, 2) * tb1 * pow(2 * tb1 + tb2, 2) +
            3 * pow(pi, 2) * (2 + pi) * ta1 * ta2 * (ta2 + 2 * (ta3 + 2 * tb1 + tb2)) + 3 * pi * pow(ta1, 2) * ((-8 + pi * (8 + pi)) * ta2 + 8 * pi * (ta3 + 2 * tb1 + tb2)) +
            pow(pi, 3) * (pow(ta2, 3) + 3 * pow(ta2, 2) * (ta3 + 2 * tb1 + tb2) - tb2 * (3 * pow(tb1, 2) + 3 * tb1 * tb2 + pow(tb2, 2))))) / pow(pi, 3) +
        (3 * Jpeak * sv * (4 * tc1 + pi * tc2) * (2 * tc1 + tc2 + tc3) * (4 * tc1 + 2 * tc2 + tc3)) / pi + 6 * (2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2) * v0 +
        (3 * (t4 + 4 * tc1 + 2 * tc2 + tc3) * (pow(a0, 2) * pi * (1 + alpha) - 2 * (pow(Apeak1, 2) * pi * (1 + alpha) + Apeak1 * Jpeak * sa * ta3 * (pi + 4 * alpha - pi * alpha) + Jpeak * sa * v0 * (pi + 4 * alpha - pi * alpha)))) /
        (Jpeak * sa * (pi * (-1 + alpha) - 4 * alpha))) / 6.;
    t4 = (pG - dd) / Vpeak;
    vector<double> res = { ta1,ta2,ta3,tb1,tb2,t4,tc1,tc2,tc3 };
    res = isValid(res);
    if (res.size() == 9) {
        res.push_back(sa);
        res.push_back(sv);
        ans.push_back(res);
    }
    return ans;
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
vector<double> TrigonometricOTG::filterResults(vector<vector<double>>& candidates, double a0, double v0, double p0, double pG, double alpha, double Jpeak, double Apeak, double Vpeak) {
    int n = candidates.size();
    double ta1, ta2, ta3, tb1, tb2, t4, tc1, tc2, tc3;
    double sa, sv;
    double dur = INT16_MAX;
    vector<double> ans;
    for (int i = 0; i < n; i++)
    {
        vector<double> candidate = candidates[i];
        /*printf("cand: %f %f %f %f %f %f %f\n", candidate[0], candidate[1], candidate[2], candidate[3], candidate[4], candidate[5], candidate[6]);*/
        ta1 = candidate[0];
        ta2 = candidate[1];
        ta3 = candidate[2];
        tb1 = candidate[3];
        tb2 = candidate[4];
        t4 = candidate[5];
        tc1 = candidate[6];
        tc2 = candidate[7];
        tc3 = candidate[8];
        sa = candidate[9];
        sv = candidate[10];
        /*printf("%f %f %f %f %f %f %f\n", t1, t2, t3, t4, t11, t22, t33);*/

        double mA1 = a0 + sa * 4 * Jpeak * ta1 / pi + sa * Jpeak * ta2;
        double mA2 = sa * 4 * Jpeak * tb1 / pi + sa * Jpeak * tb2;
        double mA3 = sv * 4 * Jpeak * tc1 / pi + sv * Jpeak * tc2;
        if (abs(mA1-mA2) > eps|| abs(mA3)-Apeak>eps || abs(mA1)-Apeak>eps) continue;

        double mA = mA1;
        double mV = (Jpeak * sa * (-ta1 + tb1) * (ta1 + tb1) * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha) + 2 * pi * alpha * (a0 * ta1 + mA1 * tb1 + (a0 * ta1 + mA1 * (ta3 + tb1) + v0) * alpha)) / (2. * pi * pow(alpha, 2));
        if (abs(mV) - abs(Vpeak)>eps) continue;

        double df = p0 + (3 * mA1 * pow(ta3 + 2 * tb1 + tb2, 2) + 3 * a0 * (2 * ta1 + ta2) * (2 * ta1 + ta2 + 2 * (ta3 + 2 * tb1 + tb2)) +
            (Jpeak * sa * (24 * (-4 + pow(pi, 2)) * pow(ta1, 3) + 96 * pow(tb1, 3) + 24 * pi * pow(tb1, 2) * tb2 - 6 * pow(pi, 2) * tb1 * pow(2 * tb1 + tb2, 2) +
                3 * pow(pi, 2) * (2 + pi) * ta1 * ta2 * (ta2 + 2 * (ta3 + 2 * tb1 + tb2)) + 3 * pi * pow(ta1, 2) * ((-8 + pi * (8 + pi)) * ta2 + 8 * pi * (ta3 + 2 * tb1 + tb2)) +
                pow(pi, 3) * (pow(ta2, 3) + 3 * pow(ta2, 2) * (ta3 + 2 * tb1 + tb2) - tb2 * (3 * pow(tb1, 2) + 3 * tb1 * tb2 + pow(tb2, 2))))) / pow(pi, 3) +
            6 * mV * (t4 + 4 * tc1 + 2 * tc2 + tc3) + (3 * (2 * tc1 + tc2 + tc3) * (Jpeak * sv * (2 * tc1 + tc2) * (4 * tc1 + pi * tc2) + mA3 * pi * (2 * tc1 + tc2 + tc3))) / pi +
            6 * (2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2) * v0) / 6.;
        if (abs(df - pG) > eps) continue;

        double total = 2*ta1+ta2+ta3+2*tb1+tb2+4*tc1+2*tc2+tc3+t4;
        if (dur > total)
        {
            dur = total;
            ans = candidate;
        }
    }
    return ans;
};
vector<double> TrigonometricOTG::profileGeneratorH(double a0, double v0, double p0, double alpha, double mJ, double mA, double mV, double t11, double t22, double t33, double t1, double t2, double sa, double t) {
    double temp;
    double dur1 = t11 + t22 + t11 + t33;
    double dur2 = t1 + t2 + t1;
    vector<double> ans;
    profileSeg prof(v0, a0, p0, alpha, t11, t22, t33, t1, t2, sa, mJ, mA, mV);
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
    else
    {
        temp = t - dur1 - dur2;
        ans = prof.Seg8(temp);
    }

    return ans;
};
vector<vector<double>> TrigonometricOTG::trajGeneratorT(double a0, double v0, double p0, double pG, double alpha, double maxJ, double maxA, double maxV, double duration) {
    vector<vector<double>> ans = {};
    vector<double> timefactors = trajTimeT(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    double t11, t22, t33, t1, t2, s, ta1, ta2, ta3, tb1, tb2, t4, tc1, tc2, tc3, sa, sv;
    //cout << res.size()<<endl;
    if (timefactors.size()==12)
    {
        ta1 = timefactors[0];
        ta2 = timefactors[1];
        ta3 = timefactors[2];
        tb1 = timefactors[3];
        tb2 = timefactors[4];
        t4 = timefactors[5];
        tc1 = timefactors[6];
        tc2 = timefactors[7];
        tc3 = timefactors[8];
        sa = timefactors[9];
        sv = timefactors[10];
        //printf("no brake: %f %f %f %f %f %f %f\n", t1, t2, t3, t4, t11, t22, t33);
        double dura = 2 * ta1 + ta2 + ta3 + 2*tb1+tb2;
        double durb = 4*tc1 + 2*tc2 + tc3;
        double total = dura + durb + t4;
        double t = 0;
        vector<double> pre = profileGeneratorH(a0, v0, p0, alpha, maxJ, maxA, maxV, ta1, ta2, ta3, tb1, tb2, sa, dura);
        double vf = pre[2];
        double pta = pre[3];
        double pt = pta + vf *t4;
        while (t <= total)
        {
            if (t < dura)
            {
                vector<double> temp = profileGeneratorH(a0, v0, p0, alpha, maxJ, maxA, maxV, ta1, ta2, ta3, tb1, tb2, sa, t);
                temp.insert(temp.begin(), t);
                ans.push_back(temp);
            }
            else if (t >= dura && t < dura + t4)
            {
                vector<double> temp = { 0, 0, vf, pta + vf * (t - dura) };
                temp.insert(temp.begin(), t);
                ans.push_back(temp);
            }
            else
            {
                vector<double> temp = profileGeneratorH(0, vf, pt, alpha, maxJ, maxA, maxV, tc1, tc2, tc3, tc1, tc2, sv, t-dura-t4);
                temp.insert(temp.begin(), t);
                ans.push_back(temp);
            }
            t += rate_;
        }
    }
    else
    {
        t11 = timefactors[0];
        t22 = timefactors[1];
        t33 = timefactors[2];
        t1 = timefactors[3];
        t2 = timefactors[4];
        s = timefactors[5];
        double pf = timefactors[6];
        ta1 = timefactors[7];
        ta2 = timefactors[8];
        ta3 = timefactors[9];
        tb1 = timefactors[10];
        tb2 = timefactors[11];
        t4 = timefactors[12];
        tc1 = timefactors[13];
        tc2 = timefactors[14];
        tc3 = timefactors[15];
        sa = timefactors[16];
        sv = timefactors[17];
        double dura = 2 * t11 + t22+t33 + 2*t1+t2;
        double durb = 2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2;
        double durc = 4 * tc1 + 2 * tc2 + tc3;
        vector<double> pre = profileGeneratorH(0, 0, pf, alpha, maxJ, maxA, maxV, ta1, ta2, ta3, tb1, tb2, sa, durb);
        double vf = pre[2];
        double ptb = pre[3];
        double pt = ptb + vf * t4;
        double t = 0;
        double total = dura + durb + durc + t4;
        while (t < total)
        {
            if (t < dura)
            {
                vector<double> temp = profileGeneratorH(a0, v0, p0, alpha, maxJ, maxA, maxV, t11, t22, t33, t1, t2, s, t);
                temp.insert(temp.begin(), t);
                ans.push_back(temp);
            }
            else if (t>=dura && t < dura+durb)
            {
                vector<double> temp = profileGeneratorH(0, 0, pf, alpha, maxJ, maxA, maxV, ta1, ta2, ta3, tb1, tb2, sa, t-dura);
                temp.insert(temp.begin(), t);
                ans.push_back(temp);
            }
            else if (t >= dura+durb && t < dura +durb+ t4)
            {
                vector<double> temp = { 0, 0, vf, ptb + vf * (t - dura-durb) };
                temp.insert(temp.begin(), t);
                ans.push_back(temp);
            }
            else
            {
                vector<double> temp = profileGeneratorH(0, vf, pt, alpha, maxJ, maxA, maxV, tc1, tc2, tc3, tc1, tc2, sv, t-dura-durb-t4);
                temp.insert(temp.begin(), t);
                ans.push_back(temp);
            }

            t += rate_;
        }
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

      vector<double> temp = trajTimeT(a0, v0, p0, pG, alpha, Jpeak, Apeak, Vpeak);

      if(temp.size()==12)
      {
        /*printf("TypeI\n");*/
        trajTimes_[i].onePiece = vector<double>(temp.begin(),temp.end()-2);
        trajTimes_[i].brake = false;
        trajTimes_[i].sa = temp[9];
        trajTimes_[i].sv = temp[10];
        /*
        for (double tt : trajTimes_[i].twoPiece) printf("%f ", tt);
        printf("\n");*/
      }
      else
      {
        /*printf("TypeII\n");*/
        double t11 = temp[0];
        double t22 = temp[1];
        double t33 = temp[2];
        double t1 = temp[3];
        double t2 = temp[4];
        double s = temp[5];
        double pf = temp[6];
        double ta1 = temp[7];
        double ta2 = temp[8];
        double ta3 = temp[9];
        double tb1 = temp[10];
        double tb2 = temp[11];
        double t4 = temp[12];
        double tc1 = temp[13];
        double tc2 = temp[14];
        double tc3 = temp[15];
        double sa = temp[16];
        double sv = temp[17];
        trajTimes_[i].twoPiece = vector<double>({t11,t22,t33,t1,t2,ta1,ta2,ta3,tb1,tb2,t4,tc1,tc2,tc3});
        trajTimes_[i].brake = true;
        trajTimes_[i].sa = sa;
        trajTimes_[i].sv = sv;
        trajTimes_[i].s = s;
        trajTimes_[i].dist = pf;
        /*for (double tt : trajTimes_[i].twoPiece) printf("%f ", tt);
        printf("\n");*/
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
            double ta1 = trajTimes_[i].onePiece[0];
            double ta2 = trajTimes_[i].onePiece[1];
            double ta3 = trajTimes_[i].onePiece[2];
            double tb1 = trajTimes_[i].onePiece[3];
            double tb2 = trajTimes_[i].onePiece[4];
            double t4 = trajTimes_[i].onePiece[5];
            double tc1 = trajTimes_[i].onePiece[6];
            double tc2 = trajTimes_[i].onePiece[7];
            double tc3 = trajTimes_[i].onePiece[8];
            double sa = trajTimes_[i].sa;
            double sv = trajTimes_[i].sv;
            //printf("All no brake: %f %f %f %f %f %f %f %f %f %f %f\n", ta1, ta2, ta3, tb1,tb2,t4, tc1, tc2, tc3,sa,sv);
            double dura = 2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2;
            double durb = 4 * tc1 + 2 * tc2 + tc3;
            double total = dura + durb + t4;
            double t = 0;
            double maxJ = Jpeak;
            double maxA = Apeak;
            double maxV = Vpeak;
            vector<double> pre = profileGeneratorH(a0, v0, p0, alpha, maxJ, maxA, maxV, ta1, ta2, ta3, tb1, tb2, sa, dura);
            double vf = pre[2];
            double pta = pre[3];
            double pt = pta + vf * t4;
            while (t <= minT_)
            {
                if (t < dura)
                {
                    vector<double> temp = profileGeneratorH(a0, v0, p0, alpha, maxJ, maxA, maxV, ta1, ta2, ta3, tb1, tb2, sa, t);
                    temp.insert(temp.begin(), t);
                    res.push_back(temp);
                }
                else if (t >= dura && t < dura + t4)
                {
                    vector<double> temp = { 0, 0, vf, pta + vf * (t - dura) };
                    temp.insert(temp.begin(), t);
                    res.push_back(temp);
                }
                else
                {
                    vector<double> temp = profileGeneratorH(0, vf, pt, alpha, maxJ, maxA, maxV, tc1, tc2, tc3, tc1, tc2, sv, t - dura - t4);
                    temp.insert(temp.begin(), t);
                    res.push_back(temp);
                }
                t += rate_;
            }
        }
        else
        {
            double t11 = trajTimes_[i].twoPiece[0];
            double t22 = trajTimes_[i].twoPiece[1];
            double t33 = trajTimes_[i].twoPiece[2];
            double t1 = trajTimes_[i].twoPiece[3];
            double t2 = trajTimes_[i].twoPiece[4];
            double ta1 = trajTimes_[i].twoPiece[5];
            double ta2 = trajTimes_[i].twoPiece[6];
            double ta3 = trajTimes_[i].twoPiece[7];
            double tb1 = trajTimes_[i].twoPiece[8];
            double tb2 = trajTimes_[i].twoPiece[9];
            double t4 = trajTimes_[i].twoPiece[10];
            double tc1 = trajTimes_[i].twoPiece[11];
            double tc2 = trajTimes_[i].twoPiece[12];
            double tc3 = trajTimes_[i].twoPiece[13];
            double s = trajTimes_[i].s;
            double pf = trajTimes_[i].dist;
            double sa = trajTimes_[i].sa;
            double sv = trajTimes_[i].sv;
            double dura = 2 * t11 + t22 + t33 + 2 * t1 + t2;
            double durb = 2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2;
            double durc = 4 * tc1 + 2 * tc2 + tc3;
            double maxJ = Jpeak;
            double maxA = Apeak;
            double maxV = Vpeak;
            vector<double> pre = profileGeneratorH(0, 0, pf, alpha, maxJ, maxA, maxV, ta1, ta2, ta3, tb1, tb2, sa, durb);
            double vf = pre[2];
            double ptb = pre[3];
            double pt = ptb + vf * t4;
            double t = 0;
            while (t < minT_)
            {
                if (t < dura)
                {
                    vector<double> temp = profileGeneratorH(a0, v0, p0, alpha, maxJ, maxA, maxV, t11, t22, t33, t1, t2, s, t);
                    temp.insert(temp.begin(), t);
                    res.push_back(temp);
                }
                else if (t >= dura && t < dura + durb)
                {
                    vector<double> temp = profileGeneratorH(0, 0, pf, alpha, maxJ, maxA, maxV, ta1, ta2, ta3, tb1, tb2, sa, t - dura);
                    temp.insert(temp.begin(), t);
                    res.push_back(temp);
                }
                else if (t >= dura + durb && t < dura + durb + t4)
                {
                    vector<double> temp = { 0, 0, vf, ptb + vf * (t - dura - durb) };
                    temp.insert(temp.begin(), t);
                    res.push_back(temp);
                }
                else
                {
                    vector<double> temp = profileGeneratorH(0, vf, pt, alpha, maxJ, maxA, maxV, tc1, tc2, tc3, tc1, tc2, sv, t - dura - durb - t4);
                    temp.insert(temp.begin(), t);
                    res.push_back(temp);
                }

                t += rate_;
            }
        }
        ans.push_back(res);
    }

    return ans;
}

vector<vector<double>> TrigonometricOTG::trajGeneratorS(){
    vector<vector<double>> ans;
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
        vector<double> res;
        if(!trajTimes_[i].brake)
        {
            double ta1 = trajTimes_[i].onePiece[0];
            double ta2 = trajTimes_[i].onePiece[1];
            double ta3 = trajTimes_[i].onePiece[2];
            double tb1 = trajTimes_[i].onePiece[3];
            double tb2 = trajTimes_[i].onePiece[4];
            double t4 = trajTimes_[i].onePiece[5];
            double tc1 = trajTimes_[i].onePiece[6];
            double tc2 = trajTimes_[i].onePiece[7];
            double tc3 = trajTimes_[i].onePiece[8];
            double sa = trajTimes_[i].sa;
            double sv = trajTimes_[i].sv;
            //printf("ptr no brake: %f %f %f %f %f %f %f %f %f %f %f\n", ta1, ta2, ta3, tb1, tb2, t4, tc1, tc2, tc3, sa, sv);
            double dura = 2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2;
            double durb = 4 * tc1 + 2 * tc2 + tc3;
            double total = dura + durb + t4;
            double maxJ = Jpeak;
            double maxA = Apeak;
            double maxV = Vpeak;
            vector<double> pre = profileGeneratorH(a0, v0, p0, alpha, maxJ, maxA, maxV, ta1, ta2, ta3, tb1, tb2, sa, dura);
            double vf = pre[2];
            double pta = pre[3];
            double pt = pta + vf * t4;
            t = idx_ * rate_;
            if (t < dura)
            {
                res = profileGeneratorH(a0, v0, p0, alpha, maxJ, maxA, maxV, ta1, ta2, ta3, tb1, tb2, sa, t);
                res.insert(res.begin(), t);
            }
            else if (t >= dura && t < dura + t4)
            {
                res = { 0, 0, vf, pta + vf * (t - dura) };
                res.insert(res.begin(), t);
            }
            else
            {
                res = profileGeneratorH(0, vf, pt, alpha, maxJ, maxA, maxV, tc1, tc2, tc3, tc1, tc2, sv, t - dura - t4);
                res.insert(res.begin(), t);
            }
        }
        else
        {
            double t11 = trajTimes_[i].twoPiece[0];
            double t22 = trajTimes_[i].twoPiece[1];
            double t33 = trajTimes_[i].twoPiece[2];
            double t1 = trajTimes_[i].twoPiece[3];
            double t2 = trajTimes_[i].twoPiece[4];
            double ta1 = trajTimes_[i].twoPiece[5];
            double ta2 = trajTimes_[i].twoPiece[6];
            double ta3 = trajTimes_[i].twoPiece[7];
            double tb1 = trajTimes_[i].twoPiece[8];
            double tb2 = trajTimes_[i].twoPiece[9];
            double t4 = trajTimes_[i].twoPiece[10];
            double tc1 = trajTimes_[i].twoPiece[11];
            double tc2 = trajTimes_[i].twoPiece[12];
            double tc3 = trajTimes_[i].twoPiece[13];
            double s = trajTimes_[i].s;
            double pf = trajTimes_[i].dist;
            double sa = trajTimes_[i].sa;
            double sv = trajTimes_[i].sv;
            double dura = 2 * t11 + t22 + t33 + 2 * t1 + t2;
            double durb = 2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2;
            double durc = 4 * tc1 + 2 * tc2 + tc3;
            double maxJ = Jpeak;
            double maxA = Apeak;
            double maxV = Vpeak;
            vector<double> pre = profileGeneratorH(0, 0, pf, alpha, maxJ, maxA, maxV, ta1, ta2, ta3, tb1, tb2, sa, durb);
            double vf = pre[2];
            double ptb = pre[3];
            double pt = ptb + vf * t4;
            t = idx_ * rate_;

            if (t < dura)
            {
                res = profileGeneratorH(a0, v0, p0, alpha, maxJ, maxA, maxV, t11, t22, t33, t1, t2, s, t);
                res.insert(res.begin(), t);
            }
            else if (t >= dura && t < dura + durb)
            {
                res = profileGeneratorH(0, 0, pf, alpha, maxJ, maxA, maxV, ta1, ta2, ta3, tb1, tb2, sa, t - dura);
                res.insert(res.begin(), t);
            }
            else if (t >= dura + durb && t < dura + durb + t4)
            {
                res = { 0, 0, vf, ptb + vf * (t - dura - durb) };
                res.insert(res.begin(), t);
            }
            else
            {
                res = profileGeneratorH(0, vf, pt, alpha, maxJ, maxA, maxV, tc1, tc2, tc3, tc1, tc2, sv, t - dura - durb - t4);
                res.insert(res.begin(), t);
            }
        }
        ans.push_back(res);
    }

    return ans;
}

vector<double> TrigonometricOTG::trajTimeT(double a0, double v0, double p0, double pG, double alpha, double maxJ, double maxA, double maxV) {
    vector<double> ans = {};
    double vf = pG - p0 >= 0 ? maxV : -maxV;
    double ta1, ta2, ta3, tb1, tb2, t4, tc1, tc2, tc3, sa, sv;
    sa = 1;
    vector<double> accToVmax = trajTimeCall1(a0, v0, p0, vf, alpha, maxJ, maxA, maxV, sa);
    ta1 = accToVmax[0];
    ta2 = accToVmax[1];
    ta3 = accToVmax[2];
    tb1 = accToVmax[3];
    tb2 = accToVmax[4];
    vector<double> res = { ta1,ta2,ta3,tb1,tb2 };
    res = isValid(res);
    accToVmax = res.size() == 5 ? accToVmax : trajTimeCall1(a0, v0, p0, vf, alpha, maxJ, maxA, maxV, -sa);
    /*cout << brakeresult.size()<<endl;*/
    ta1 = accToVmax[0];
    ta2 = accToVmax[1];
    ta3 = accToVmax[2];
    tb1 = accToVmax[3];
    tb2 = accToVmax[4];
    sa = accToVmax[5];
    double pta = accToVmax[6];
    res = { ta1,ta2,ta3,tb1,tb2,sa };
    vector<double> brakeToZero = trajTimeCall2(0, vf, 0, alpha, maxJ, maxA, maxV);
    tc1 = brakeToZero[0];
    tc2 = brakeToZero[1];
    tc3 = brakeToZero[2];
    sv = brakeToZero[3];
    double ptb = brakeToZero[4];
    double pt = pta + ptb;
    if ((pt-p0)*(pG-p0)>=0 && abs(pt-p0)<abs(pG-p0))
    {
        t4 = (pG - pt) / vf;
        sv = -((vf < 0) ? -1 : ((vf > 0) ? 1 : 0));
        //printf("no brake: %f %f %f %f %f %f %f\n", t1, t2, t3, t4, t11, t22, t33);
        double dura = 2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2;
        double durb = 4 * tc1 + 2 * tc2 + tc3;
        double total = dura + durb + t4;
        ans = { ta1, ta2, ta3, tb1, tb2, t4, tc1, tc2, tc3, sa, sv, total };
    }
    else
    {
        ans = trajTimeN(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    }
    return ans;
};
vector<double> TrigonometricOTG::trajTimeN(double a0, double v0, double p0, double pG, double alpha, double maxJ, double maxA, double maxV) {
    vector<double> ans = {};
    vector<vector<double>> cand1 = typeI(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    vector<vector<double>> cand2 = typeII(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    vector<vector<double>> cand3 = typeIII(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    vector<vector<double>> cand4 = typeIV(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    vector<vector<double>> cand5 = typeV(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    vector<vector<double>> cand6 = typeVI(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    vector<vector<double>> cand7 = typeVII(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    vector<vector<double>> cand8 = typeVIII(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);

    vector<vector<double>> candidates;
    candidates.insert(candidates.end(), cand1.begin(), cand1.end());
    candidates.insert(candidates.end(), cand2.begin(), cand2.end());
    candidates.insert(candidates.end(), cand3.begin(), cand3.end());
    candidates.insert(candidates.end(), cand4.begin(), cand4.end());
    candidates.insert(candidates.end(), cand5.begin(), cand5.end());
    candidates.insert(candidates.end(), cand6.begin(), cand6.end());
    candidates.insert(candidates.end(), cand7.begin(), cand7.end());
    candidates.insert(candidates.end(), cand8.begin(), cand8.end());

    vector<double> res = filterResults(candidates, a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    double ta1, ta2, ta3, tb1, tb2, t4, tc1, tc2, tc3, sa, sv;
    //cout << res.size()<<endl;
    if (!res.empty())
    {
        ta1 = res[0];
        ta2 = res[1];
        ta3 = res[2];
        tb1 = res[3];
        tb2 = res[4];
        t4 = res[5];
        tc1 = res[6];
        tc2 = res[7];
        tc3 = res[8];
        sa = res[9];
        sv = res[10];
        //printf("no brake: %f %f %f %f %f %f %f\n", t1, t2, t3, t4, t11, t22, t33);
        double dura = 2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2;
        double durb = 4 * tc1 + 2 * tc2 + tc3;
        double total = dura + durb + t4;
        ans = { ta1, ta2, ta3, tb1, tb2, t4, tc1, tc2, tc3, sa, sv, total };
    }
    else
    {
        vector<double> brakeresult = trajTimeCall1(a0, v0, p0, 0, alpha, maxJ, maxA, maxV, 1);
        ta1 = brakeresult[0];
        ta2 = brakeresult[1];
        ta3 = brakeresult[2];
        tb1 = brakeresult[3];
        tb2 = brakeresult[4];
        vector<double> res = { ta1,ta2,ta3,tb1,tb2 };
        res = isValid(res);
        brakeresult = res.size() == 5 ? brakeresult : trajTimeCall1(a0, v0, p0, 0, alpha, maxJ, maxA, maxV, -1);
        /*cout << brakeresult.size()<<endl;*/
        ta1 = brakeresult[0];
        ta2 = brakeresult[1];
        ta3 = brakeresult[2];
        tb1 = brakeresult[3];
        tb2 = brakeresult[4];
        double s = brakeresult[5];
        double pt = brakeresult[6];
        res = { ta1,ta2,ta3,tb1,tb2,s,pt};
        double dura = 2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2;
        ans.insert(ans.end(), res.begin(), res.end());
        vector<double> second = trajTimeT1(0, 0, pt, pG, alpha, maxJ, maxA, maxV);
        ta1 = second[0];
        ta2 = second[1];
        ta3 = second[2];
        tb1 = second[3];
        tb2 = second[4];
        t4 = second[5];
        tc1 = second[6];
        tc2 = second[7];
        tc3 = second[8];
        double durb = 2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2 + t4 + 4*tc1 + 2*tc2 + tc3;
        double total = dura + durb;
        ans.insert(ans.end(), second.begin(), second.end() - 1);
        ans.push_back(total);
    }
    return ans;
};
vector<double> TrigonometricOTG::trajTimeT1(double a0, double v0, double p0, double pG, double alpha, double maxJ, double maxA, double maxV) {
    vector<double> ans = {};
    vector<vector<double>> cand1 = typeI(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    vector<vector<double>> cand2 = typeII(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    vector<vector<double>> cand3 = typeIII(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    vector<vector<double>> cand4 = typeIV(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    vector<vector<double>> cand5 = typeV(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    vector<vector<double>> cand6 = typeVI(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    vector<vector<double>> cand7 = typeVII(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    vector<vector<double>> cand8 = typeVIII(a0, v0, p0, pG, alpha, maxJ, maxA, maxV);

    vector<vector<double>> candidates;
    candidates.insert(candidates.end(), cand1.begin(), cand1.end());
    candidates.insert(candidates.end(), cand2.begin(), cand2.end());
    candidates.insert(candidates.end(), cand3.begin(), cand3.end());
    candidates.insert(candidates.end(), cand4.begin(), cand4.end());
    candidates.insert(candidates.end(), cand5.begin(), cand5.end());
    candidates.insert(candidates.end(), cand6.begin(), cand6.end());
    candidates.insert(candidates.end(), cand7.begin(), cand7.end());
    candidates.insert(candidates.end(), cand8.begin(), cand8.end());

    vector<double> res = filterResults(candidates, a0, v0, p0, pG, alpha, maxJ, maxA, maxV);
    double ta1, ta2, ta3, tb1, tb2, t4, tc1, tc2, tc3, sa, sv;
    //cout << res.size()<<endl;
    if (!res.empty())
    {
        ta1 = res[0];
        ta2 = res[1];
        ta3 = res[2];
        tb1 = res[3];
        tb2 = res[4];
        t4 = res[5];
        tc1 = res[6];
        tc2 = res[7];
        tc3 = res[8];
        sa = res[9];
        sv = res[10];
        //printf("no brake: %f %f %f %f %f %f %f\n", t1, t2, t3, t4, t11, t22, t33);
        double dura = 2 * ta1 + ta2 + ta3 + 2 * tb1 + tb2;
        double durb = 4 * tc1 + 2 * tc2 + tc3;
        double total = dura + durb + t4;
        ans = { ta1, ta2, ta3, tb1, tb2, t4, tc1, tc2, tc3, sa, sv, total };
    }
    else
    {
        ans = { -1,0,0,0,0,0,0,0,0,0,0,0 };
    }
    return ans;
};
vector<double> TrigonometricOTG::trajTimeCall1(double a0, double v0, double p0, double vf, double alpha, double mJ, double mA, double mV, double s)
{
    double Jpeak = mJ;
    vector<double> temp = { (alpha * (2 * a0 * pi * (1 + alpha) + sqrt(2 * pi) * sqrt((1 + alpha) * (pow(a0,2) * pi * (1 + alpha) + 2 * mJ * s * (v0 - vf) * (-pi + (-4 + pi) * alpha))))) /
    (2. * mJ * s * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha)), (alpha * (2 * a0 * pi * (1 + alpha) -
        sqrt(2 * pi) * sqrt((1 + alpha) * (pow(a0,2) * pi * (1 + alpha) + 2 * mJ * s * (v0 - vf) * (-pi + (-4 + pi) * alpha))))) / (2. * mJ * s * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha)) };
    temp = isValid(temp);
    double t11 = temp.empty() ? -1 : *max_element(temp.begin(), temp.end());
    double t1 = t11 + (a0 * pi * alpha) / (mJ * pi * s + 4 * mJ * s * alpha - mJ * pi * s * alpha);
    double t2 = ((1 - alpha) / alpha)* t1;
    double t22 = ((1 - alpha) / alpha)* t11;
    double t33 = 0;
    double Apeak = a0 + 4 * mJ * s * t11 / pi + mJ * s * t22;

    if (abs(Apeak) > mA)
    {
        Apeak = s * mA;
        t11 = ((a0 - Apeak) * pi * alpha) / (Jpeak * s * (pi * (-1 + alpha) - 4 * alpha));
        t1 = -((Apeak * pi * alpha) / (Jpeak * s * (-pi - 4 * alpha + pi * alpha)));
        t2 = ((1 - alpha) / alpha) * t1;
        t22 = ((1 - alpha) / alpha) * t11;
        t33 = (-(pow(a0, 2) * pi * (1 + alpha)) + 2 * pow(Apeak, 2) * pi * (1 + alpha) + 2 * Jpeak * s * (v0 - vf) * (pi + 4 * alpha - pi * alpha)) / (2. * Apeak * Jpeak * s * (pi * (-1 + alpha) - 4 * alpha));
    }
    double pf = p0 + (8 * Jpeak * s * pow(t1, 3)) / pow(pi, 3) + (3 * Apeak * pow(2 * t1 + t2 + t33, 2) + 3 * a0 * (2 * t11 + t22) * (4 * t1 + 2 * t11 + 2 * t2 + t22 + 2 * t33) +
        (Jpeak * s * (48 * (pow(t1, 3) - 2 * pow(t11, 3)) + 24 * pi * (pow(t1, 2) * t2 - pow(t11, 2) * t22) +
            pow(pi, 3) * (-3 * pow(t1, 2) * t2 - 3 * t1 * pow(t2, 2) - pow(t2, 3) + 3 * t11 * (2 * t1 + t11) * t22 + 3 * t1 * t22 * (2 * t11 + t22) +
                3 * t2 * t22 * (2 * t11 + t22) + pow(t22, 2) * (3 * (t1 + t11) + t22) + 3 * t22 * (2 * t11 + t22) * t33) -
            6 * pow(pi, 2) * (2 * pow(t1, 3) + 2 * pow(t1, 2) * t2 + t1 * pow(t2, 2) + 2 * pow(t1, 2) * (t1 + t2) - 4 * t1 * t11 * (2 * t11 + t22) -
                t11 * (2 * t11 + t22) * (2 * t11 + 2 * t2 + t22 + 2 * t33)))) / pow(pi, 3) + 6 * (2 * t1 + 2 * t11 + t2 + t22 + t33) * v0) / 6.;
    return {t11,t22,t33,t1,t2,s,pf};
};
vector<double> TrigonometricOTG::trajTimeCall2(double a0, double v0, double p0, double alpha, double mJ, double mA, double mV)
{
    double Jpeak = mJ;
    double s = -((v0 < 0) ? -1 : ((v0 > 0) ? 1 : 0));
    double t1, t2, t3;
    t1 = v0 == 0 ? 0 : sqrt(pi) * sqrt((v0 * pow(alpha, 2)) / (Jpeak * s * (pi * (-1 + alpha) - 4 * alpha) * (1 + alpha)));
    t2 = (1 - alpha) * t1 / alpha;
    t3 = 0;
    double Apeak = (4 * mJ * s * t1) / pi + mJ * s * t2;
    if (abs(Apeak)>mA)
    {
        Apeak = s * mA;
        t1 = -((Apeak * pi * alpha) / (Jpeak * s * (-pi - 4 * alpha + pi * alpha)));
        t2 = (1 - alpha) * t1 / alpha;
        t3 = (pow(Apeak, 2) * pi * (1 + alpha) + Jpeak * s * v0 * (pi + 4 * alpha - pi * alpha)) / (Apeak * Jpeak * s * (pi * (-1 + alpha) - 4 * alpha));
    }
    double dur = 4 * t1 + 2 * t2 + t3;
    double pf = p0 + (Apeak * pow(2 * t1 + t2 + t3, 2)) / 2. + (Jpeak * s * t2 * (4 * pow(t1, 2) + t2 * (t2 + t3) + 2 * t1 * (2 * t2 + t3))) / 2. +
        (Jpeak * s * t1 * (8 * pow(t1, 2) + 2 * t2 * (t2 + t3) + 4 * t1 * (2 * t2 + t3))) / pi + (4 * t1 + 2 * t2 + t3) * v0;
    return { t1,t2,t3,s,pf};
};
