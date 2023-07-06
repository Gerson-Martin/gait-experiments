#ifndef STATEVARIABLEBLOCK_H
#define STATEVARIABLEBLOCK_H

#include "Block.h"
#include "StateVariable.h"

class StateVariableBlock : public BaseBlock
{
public:
    StateVariableBlock(double sampleTime = 0.01);//make sure dts has a value.

    double OutputUpdate(double new_input);


private:


    double dts; //sample time. Make sure dts has a value.
    StateVariable variable;

};

#endif // STATEVARIABLEBLOCK_H
