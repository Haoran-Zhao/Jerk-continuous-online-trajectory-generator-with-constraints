// TrigonometricAlgorithm.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>
#include<TrigonometricOTG.h>
using namespace std;
int main()
{
    /*Read inputs from .txt*/
    string line;
    ifstream infile("dataset.txt");
    vector<vector<double> > dataset;

    while (getline(infile, line)) {
        std::vector<double> row;

        string s;
        for (char& c : line) {
            if (c != ',') {
                s += c;
            }
            else
            {
                double v = stod(s);
                row.push_back(v);
                s.clear();
            }
        }
        double v = stod(s);
        row.push_back(v);
        s.clear();
        dataset.push_back(row);
    }

    //for (vector<double>& row : dataset) {
    //    for (double c : row) {
    //        std::cout << c << ' ';
    //    }

    //    std::cout << '\n';
    //}

    /*Generate trajectory time profile with given input from .txt, finally save results as .txt*/
 /*   vector<vector<double>> res;
    for (vector<double>& row : dataset)
    {
        TrigonometricOTG trajOTG;
        trajOTG.mA_ = { row[6] };
        trajOTG.mJ_ = { row[5] };
        trajOTG.mV_ = { row[7] };
        trajOTG.num_dof_ = 1;
        trajOTG.rate_ = 0.008;
        double v0 = row[1];
        double a0 = row[0];
        double p0 = row[2];
        double pG = row[3];
        double alpha = row[4];
        double Jpeak = trajOTG.mJ_[0];
        double Apeak = trajOTG.mA_[0];
        double Vpeak = trajOTG.mV_[0];
        vector<double> ans = trajOTG.TrigonometricOTG::trajTimeT(v0, a0, p0, pG, alpha, Jpeak, Apeak, Vpeak);
        if (ans.size() == 7) ans.insert(ans.end(), {0,0});
        res.push_back(ans);
    }

    ofstream output_file("time_cpp.txt");
    ostream_iterator<double> output_iterator(output_file, "\t");
    for (int i = 0; i < res.size(); i++)
    {
        copy(res.at(i).begin(), res.at(i).end(), output_iterator);
        output_file << '\n';
    }*/

    /*compute trajectory time profile with manual input*/
   //TrigonometricOTG trajOTG;
   //trajOTG.mA_ = { 1 };
   //trajOTG.mJ_ = { 1 };
   //trajOTG.mV_ = { 1 };
   //trajOTG.num_dof_ = 1;
   //trajOTG.rate_ = 0.008;
   //double v0 = 0.38;
   //double a0 = -0.0756;
   //double p0 = 1.88;
   //double pG = -2.62;
   //double alpha = 0.774;
   //double Jpeak = trajOTG.mJ_[0];
   //double Apeak = trajOTG.mA_[0];
   //double Vpeak = trajOTG.mV_[0];
   //vector<double> ans = trajOTG.TrigonometricOTG::trajTimeT(v0, a0, p0,pG,alpha,Jpeak,Apeak,Vpeak);

   //for (int i = 0; i < ans.size(); i++)
   // {
   //    cout << ans[i] << ' ';
   // }
   //cout << endl;

    /*Generate trajectory profile and save as .txt*/
    TrigonometricOTG trajOTG;

    trajOTG.num_dof_ = 1;
    trajOTG.rate_ = 0.008;
    double a0 = 4.2686;
    double v0 = -1.3462;
    double p0 = -42.0478;
    double pG = -11.0910;
    double alpha = 0.9575;
    trajOTG.mJ_ = { 0.5046 };
    trajOTG.mA_ = { 0.3697 };
    trajOTG.mV_ = { 0.2570 };
    double Jpeak = trajOTG.mJ_[0];
    double Apeak = trajOTG.mA_[0];
    double Vpeak = trajOTG.mV_[0];
    vector<vector<double>> ans = trajOTG.TrigonometricOTG::trajGeneratorT(v0, a0, p0,pG,alpha,Jpeak,Apeak,Vpeak);

    ofstream output_file1("traj.txt");
    ostream_iterator<double> output_iterator1(output_file1, "\t");
    for (int i = 0; i < ans.size(); i++)
    {
        copy(ans.at(i).begin(), ans.at(i).end(), output_iterator1);
        output_file1 << '\n';
    }
        

    //for (int i = 0; i < ans.size(); i++)
    //{
    //    for (int j = 0; j < ans[0].size(); j++)
    //    {
    //        cout << ans[i][j] << ' ';
    //    }
    //    cout << endl;
    //}
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
