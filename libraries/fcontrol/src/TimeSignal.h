#ifndef TIMESIGNAL_H
#define TIMESIGNAL_H

#include <vector>
#include <valarray>

class TimeSignal
{
public:
    TimeSignal();
    TimeSignal(unsigned int init_size, double init_fs);
    TimeSignal(std::vector<double> new_data, double new_dts);
    TimeSignal(std::valarray<double> new_data, double new_dts);

    friend double operator > (double new_value, TimeSignal & ts)
    {
        return ts.Update(new_value);

    }

    bool Initialize(unsigned int new_size, double new_fs);
    bool GetParams(unsigned int &out_size, double &out_fs) const;

    long Update(double new_value);
    double GetValue(long index);

    //variables exposed
    //data must be exposed for better performance.
    //Think of this class as an improved std::valarray.
    std::valarray<double> data;

    double getFs() const;

    double getN() const;


    double getDts() const;

private:
    double fs;//frequency of sampling
    double dts;//time of sampling
    unsigned int N;//number of data

};

#endif // TIMESIGNAL_H
