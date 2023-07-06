#ifndef TOOLSFCONTROL_H
#define TOOLSFCONTROL_H

#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <fstream>      // std::ifstream
#include <sstream>      // std::istringstream

using namespace std;
class SamplingTime
{
public:
    SamplingTime(double new_dts = 0.01);
//    long ToolsFControlInit();
    long SetSamplingTime(double new_dts);
    double WaitSamplingTime();

private:
    std::chrono::system_clock::time_point actualTimeValue, lastTimeValue; //last time value
//    long timeAlreadyWaited; //in nanoseconds
    long dts; //sampling time nanoseconds.
    std::chrono::nanoseconds dtsWait, tWaited;
//    chrono::system_clock clock;

};

class TableInterpolation
{
public:
    TableInterpolation();
    TableInterpolation(string new_table);
    double GetTableValue(ulong row, ulong col);
private:
    long getData(string fileName);
    vector < vector<double> > lookupTable;
    ulong hsize,vsize;
    ulong row,col;


};

#endif // TOOLSFCONTROL_H
