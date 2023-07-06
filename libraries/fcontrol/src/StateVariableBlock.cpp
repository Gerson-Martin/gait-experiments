#include "StateVariableBlock.h"


StateVariableBlock::StateVariableBlock(double sampleTime)
{

    variable = StateVariable(0,0,0);
    dts=sampleTime;

}


double StateVariableBlock::OutputUpdate(double new_input)
{
    variable.Update(new_input, dts);
    return variable.D(0);

}
