#ifndef STATEVARIABLE_H
#define STATEVARIABLE_H

#include <vector>

typedef unsigned int uint;



class StateVariable
{
public:
    StateVariable();
    StateVariable(std::vector<double> formerState, std::vector<double> initialState);
    StateVariable(double newValue, double D1, double D2);

    long Update(double newValue, double dt);

    double GetOrder();

    double D(uint dOrder);

    std::vector<double> getState() const;

    std::vector<double> getFormer() const;

private:

    long Initialize(std::vector<double> newFormer, std::vector<double> newInitial);


    long order;
    std::vector<double> state; //current value [0] and derivatives of order [1],[2]...[n]
    double dtFS; //Time difference between former and state
    std::vector<double> former; //former value [0] and derivatives of order [1],[2]...[n]

};


#endif // STATEVARIABLE_H
