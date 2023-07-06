#ifndef PIDBLOCK_H
#define PIDBLOCK_H

#include "fcontrol.h"


class PIDBlock : public BaseBlock
{
public:
    PIDBlock();
    PIDBlock(double kp, double ki, double kd, double new_Ts);
    double UpdateControl(double input);
    double UpdateControlILock(double input);

    double OutputUpdate(double input);

    long AntiWindup(double minPlantSaturation, double maxPlantSaturation);
//    void operator>>(double & output);
//    void operator>>(SystemBlock & output);
    friend double operator > (double input, PIDBlock& output)
    {
        return output.OutputUpdate(input);
    }

    double GetState() const;


private:
    SystemBlock iBlock;
    SystemBlock dBlock;
    double pBlock,kp,ki,kd,Ts;
    double state;
    SystemBlock pidBlock;
    long Initial(double new_Ts);
    double cp,ci,cd;

};

#endif // PIDBLOCK_H
