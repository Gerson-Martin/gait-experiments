#include "TimeSignal.h"

TimeSignal::TimeSignal()
{
    //default signal is 256 points sampled at 1 Hz
    Initialize(256,1);
}

TimeSignal::TimeSignal( unsigned int init_size, double init_fs)
{
    Initialize(init_size,init_fs);
}

TimeSignal::TimeSignal(std::vector<double> new_data, double new_dts)
{
    dts = new_dts;
    data = std::valarray<double>(new_data.data(),new_data.size());
}

TimeSignal::TimeSignal(std::valarray<double> new_data, double new_dts)
{
    dts = new_dts;
    data = new_data;
}

bool TimeSignal::Initialize(unsigned int new_size, double new_fs)
{
    fs=new_fs;
    dts=1/fs;
    N=new_size;
    data.resize(0);
    data.resize(N);

    for (int i=0; i<N; i++)
    {
        data[i]=0;
    }

    return true;

}

bool TimeSignal::GetParams(unsigned int &out_size, double &out_fs) const
{
    out_fs = fs;
    out_size = N;
    return 0;

}

long TimeSignal::Update(double new_value)
{
    data=data.shift(-1);
    data[0]= new_value;

    return 0;
}

double TimeSignal::GetValue(long index)
{
    if (index > N)
    {
        return -1;
    }
    else
    {
        return data[index];
    }

}

double TimeSignal::getFs() const
{
    return fs;
}

double TimeSignal::getN() const
{
    return N;
}

double TimeSignal::getDts() const
{
    return dts;
}

