#ifndef ONLINESYSTEMIDENTIFICATION_H
#define ONLINESYSTEMIDENTIFICATION_H

#define rlms_z (1)                //Number of input past values including current input
#define rlms_p 2					//Number of output past values
//#define rlms_N (rlms_z+rlms_p)      //phi dimension
#define rlms_N (32)      //phi dimension. Higher orders will throw assertion on eigen

#include <vector>
#undef Success //as in http://eigen.tuxfamily.org/bz/show_bug.cgi?id=253
//#include <3rdparty/eigen/Eigen/Dense>
#include <Eigen/Dense>
#define Success 0 //as in http://eigen.tuxfamily.org/bz/show_bug.cgi?id=253
#include <iostream>
#include "SystemBlock.h"

using namespace std;
using namespace Eigen;

#include <Eigen/Eigenvalues>
//#include <3rdparty/eigen/Eigen/Eigenvalues>


class OnlineSystemIdentification
{
public:
    OnlineSystemIdentification();
    OnlineSystemIdentification(ulong new_numOrder, ulong new_denOrder, double new_ff = 0.95, double new_paramFilter = 0.8, double new_paramAvg = 20);
    OnlineSystemIdentification(ulong new_numOrder, ulong new_denOrder, SystemBlock new_filter);
    OnlineSystemIdentification(ulong new_numOrder, ulong new_denOrder, SystemBlock new_filter, double new_ff, double new_paramFilter, double new_paramAvg = 20);
    long SetFilter(SystemBlock filter);

    double UpdateSystem(double new_input, double new_output);
    double UpdateSystemDT1(double new_input, double new_output);
    double UpdateSystemPEff(double new_input, double new_output);


    double GetZTransferFunction(vector<double>& num,vector<double>& den);

    double PrintZTransferFunction(double dts);
    vector<double> GetParamsVector();

    double PrintParamsVector();

    long GetMagnitudeAndPhase(double dts, double w, double & magnitude, double & phase);

    long GetSystemBlock(SystemBlock &idsys);
    double GetAvgZTransferFunction(vector<double> &num, vector<double> &den);
    long GetAvgSystemBlock(SystemBlock &idsys);
    long SetParamsVector(vector<double> new_th);
private:

    ulong numOrder,denOrder;
    long order;
    long phiNumIndex,phiLastIndex;

    SystemBlock inFilter, outFilter;
    bool filterOn;


    complex<double> z,nz,dz;


//    Matrix<double,Dynamic,Dynamic> P;
//    Matrix<double,Dynamic,1> phi;
//    Matrix<double,Dynamic,1> L;
//    Matrix<double,Dynamic,1> th;

//    Matrix<double, 1, 1, 0, 1, 1> ff; //forgetting factor
    double ff, paramFilter, PEff=1, PEcorr=1, paramAvg;
    Matrix<double, 1, 1, 0, 1, 1> PEAux; //max order rlms_N


    Matrix<double, Dynamic, Dynamic, 0, rlms_N, rlms_N> P; //max order rlms_N #defined
    Matrix<double, Dynamic, 1, 0, rlms_N, 1> phi; //max order rlms_N
//    Matrix<double, 1, Dynamic, 0, 1, rlms_N> phit; //max order rlms_N

    Matrix<double, Dynamic, 1, 0, rlms_N, 1> R_ev; //max order rlms_N

    Matrix<double, Dynamic, 1, 0, rlms_N, 1>  L;//max order rlms_N
    Matrix<double, Dynamic, 1, 0, rlms_N, 1> th;//max order rlms_N
    Matrix<double, Dynamic, 1, 0, rlms_N, 1> thAvg; //max order rlms_N
    double converge;

    Matrix<double, Dynamic, Dynamic, 0, rlms_N, rlms_N> R; //max order rlms_N #defined
    Matrix<double, Dynamic, Dynamic, 0, rlms_N, rlms_N> F; //max order rlms_N #defined
    Matrix<double, Dynamic, Dynamic, 0, rlms_N, rlms_N> newR; //max order rlms_N #defined


    double err;
    double ti; //time index
    ulong oPEi=0; //persistent excitation order index
    double R_ev_sum=0; //persistent excitation order index


    double output, input;
    vector<double> oldOuts, oldIns;
    vector<double> idNum, idDen;


};

#endif // ONLINESYSTEMIDENTIFICATION_H
