#ifndef BLOCK_H
#define BLOCK_H


#include <vector>
#include <deque>
#include <valarray>
#include <math.h>
#include <iostream>

#include "TimeSignal.h"

class BaseBlock
{
public:
    BaseBlock();

    //operator overloading
    friend double operator > (double input, BaseBlock& output)
    {
        return output.OutputUpdate(input);
    }

    friend double operator > (BaseBlock& from, double & to)
    {
        to=from.GetState();
        std::cout << to;
        return to;
    }

    long SetSaturation(double low, double high);



protected:
    double maxOut, minOut; // 0 values disables saturation
    bool saturation;


private:

//    double saturation;
    //saturation values

    virtual double OutputUpdate(double new_input)=0;
    virtual double GetState() const = 0;




};

#endif // BLOCK_H
