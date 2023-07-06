#ifndef STATEOBSERVER_H
#define STATEOBSERVER_H

#include "StateSpace.h"

class StateObserver:public StateSpace
{
public:
    StateObserver(std::vector<vector<double>> G,std::vector<vector<double>> H,std::vector<vector<double>>C,std::vector<vector<double>>D,double dts);
    double output(double new_input,double y);

protected:
    double error;
    VectorXd K;
};
#endif //STATEOBSERVER_H
