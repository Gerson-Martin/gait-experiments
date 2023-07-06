#ifndef PLOT_H
#define PLOT_H


#include <vector>
#include <stdio.h>
#include <fstream>      // std::fstream
#include <algorithm>    // std::min_element, std::max_element

#include <iostream>
#include <sstream>
//#include <plplot/plplot.h>
//#include <plplot/plstream.h>
#define NSIZE    101
#define STRINGSIZE    255


// Patch for Windows by Damien Loison
#ifdef _WIN32
#    include <windows.h>
#    define GNUPLOT_PCLOSE _pclose
#    define GNUPLOT_POPEN  _popen
#    define GNUPLOT_FILENO _fileno
#else
#    define GNUPLOT_PCLOSE pclose
#    define GNUPLOT_POPEN  popen
#    define GNUPLOT_FILENO fileno
#endif

#ifdef _WIN32
#    define GNUPLOT_ISNAN _isnan
#else
// cppreference.com says std::isnan is only for C++11.  However, this seems to work on Linux
// and I am assuming that if isnan exists in math.h then std::isnan exists in cmath.
#    define GNUPLOT_ISNAN std::isnan
#endif

using namespace std;

class IPlot
{
public:
    IPlot(double sampleTime = 1, string new_title = "", string new_xLabel = "", string new_yLabel = "");
    IPlot(vector<double> &new_y);
    ~IPlot();

    long SetParameters(string new_parameters);

    long pushBack(double new_value);
    long Update(double new_value);

    long Plot();
    long Save(string filename);

    long Plot(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley);
    long PlotAndSave(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley, std::string filename);

//    long PlotAxis(double scalex, double scaley, double offset);


    long PlotAndSave(string filename);


private:



    double Ts;

    string xLabel;
    string yLabel;
    string title;

    std::vector<double> x,y;

    long N;

    FILE* figure;
//    FILE* figdata;
    stringstream figdata;
    string strdata;

    string parameters;
};

#endif // PLOT_H
