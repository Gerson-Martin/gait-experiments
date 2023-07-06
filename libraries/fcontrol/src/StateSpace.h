#ifndef STATESPACE_H
#define STATESPACE_H

#include "fcontrol.h"
#include "LinearSystem.h"
#include <Eigen/Dense>
#include <iostream>
//#include <unsupported/Eigen/MatrixFunctions>

/**
 * @brief The StateSpace class:
 * This class encapsulates a system based in state space, defined by his matrix G, H, C y B or its transference function G.
 * .
 */

using namespace Eigen;

class StateSpace:public LinearSystem
{
public:
    StateSpace();
    StateSpace(MatrixXd G,MatrixXd H,MatrixXd C,MatrixXd D,double dts);
    StateSpace(std::vector<vector<double>> G,std::vector<vector<double>> H,std::vector<vector<double>>C,std::vector<vector<double>>D,double dts);
    double output(double new_input);
    vector<vector<double>> output(vector<double> new_input);
    void InitialCondition(vector<double> x0);
    std::vector<vector<double>> getObservabilityMatrix();
    std::vector<vector<double>> getControllabiltyMatrix();
    double GetState() const;
    std::vector<vector<double>> getState();
    void printSystem();
    void setParam(MatrixXd G,MatrixXd H,MatrixXd C,MatrixXd D,double dts);

protected:
    MatrixXd vector2matrix(std::vector<vector<double>>);
    MatrixXd vector2matrix(std::vector<double>);

    std::vector<vector<double>> matrix2vector(MatrixXd);
    MatrixXd G;
    MatrixXd H;
    MatrixXd C;
    MatrixXd D;
    VectorXd x;
    VectorXd y;
    RowVectorXd u;
    MatrixXd ObservabilityMatrix;
    MatrixXd ControllabilityMatrix;
    double dts;
    int n_state;
    int n_input;
    int n_output;
};

#endif //STATESPACE_H
