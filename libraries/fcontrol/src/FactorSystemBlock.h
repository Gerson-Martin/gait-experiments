#ifndef FACTORSYSTEMBLOCK_H
#define FACTORSYSTEMBLOCK_H

#include "fcontrol.h"

using namespace std;

class FactorSystemBlock : public BaseBlock
{
public:
    FactorSystemBlock();


    double OutputUpdate(double new_input);


    FactorSystemBlock(const std::vector<double> &new_zeros, const std::vector<double> &new_poles, double new_gain);
    double GetState() const;

private:

    double response;

    int order;

    double gain;

    //state values
    double state;
    std::vector<double> oldStates;
    std::vector<double> oldInputs;

    vector<double> poles,zeros;
    vector<SystemBlock> gs;

    long InitFactorSystemBlock(const std::vector<double> &new_zeros, const std::vector<double> &new_poles, double new_gain);
};

#endif // FACTORSYSTEMBLOCK_H
