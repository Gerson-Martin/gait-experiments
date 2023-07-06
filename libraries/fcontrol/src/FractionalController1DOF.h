#ifndef FRACTIONALCONTROLLER1DOF_H
#define FRACTIONALCONTROLLER1DOF_H

#include "SystemBlock.h"
#include "FractionalDerivative.h"


class FractionalController1DOF : public BaseBlock
{
public:
    FractionalController1DOF();
    FractionalController1DOF(double new_exp, double new_dts);
    double OutputUpdate(double new_input);
    long ParameterUpdate(double new_exp, double new_dts);

    long Reset(double new_state = 0);
    double GetState() const;

private:
    long Init(double new_exp, double new_dts);
    vector<SystemBlock> ipart;
    FractionalDerivative fpart;

    double iexp;
    double exp;
    double dts;
    double state;

};

#endif // FRACTIONALCONTROLLER1DOF_H
