#include "StateVariable.h"

long StateVariable::Initialize(std::vector<double> newFormer, std::vector<double> newInitial)
{
    state = newInitial;
    order=state.size();
    former = newFormer;
    return 0;

}

std::vector<double> StateVariable::getFormer() const
{
    return former;
}

std::vector<double> StateVariable::getState() const
{
    return state;
}

StateVariable::StateVariable()
{
    Initialize(std::vector<double>(3,0), std::vector<double>(3,0));

}

StateVariable::StateVariable(std::vector<double> formerState, std::vector<double> initialState)
{
    Initialize(formerState, initialState);

}

StateVariable::StateVariable(double newValue, double D1, double D2)
{

    state = std::vector<double>{newValue,D1,D2};
    order = state.size();
    former.clear();
    former.resize(order);

}


long StateVariable::Update(double newValue, double dt)
{
    //first of all, actual state becomes former state
    former=state;

    //then update values for state
    state[0]=newValue;

    for (uint i=1; i<order; i++)
    {
        state[i]=(state[i-1]-former[i-1])/dt;
    }
    return 0;


}

double StateVariable::GetOrder()
{
    return state.size();

}

double StateVariable::D(uint dOrder)
{
    if (dOrder>order)
    {
        return 0;
    }
    else
    {
        return state[dOrder];
    }

}
