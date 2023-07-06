#ifndef SYSTEMTF_H
#define SYSTEMTF_H



using namespace std;
#include "TransferFunction.h"
#include "LinearSystem.h"
#include "Block.h"


/**
 * @brief The SystemBlock class:
 * This class encapsulates a system control block, defined by its transference function G.
 * .
 */


class SystemTF : public LinearSystem
{
public:
    SystemTF();
    SystemTF(const std::vector<double> &new_numCoef, const std::vector<double> &new_denCoef);
    SystemTF(const std::vector<double> &new_numCoef, const std::vector<double> &new_denCoef, double new_gain);
    SystemTF(const std::vector<double> &new_numCoef, const std::vector<double> &new_denCoef,
                const std::vector<double> &new_numExps, const std::vector<double> &new_denExps);
    SystemTF(const TransferFunction &newH);
    SystemTF(double b0,double b1,double a0,double a1);

    double GetNumOrder() const;
    double GetDenOrder() const;
    long GetTransfer(std::vector<double> & numCoefficients, std::vector<double> & denCoefficients) const;
    double GetState() const;


    double OutputUpdate(double new_input);

    long Reset();
    long Reset(double new_state);


    long GetMagnitudeAndPhase(double dts, double w, double &magnitude, double &phase) const;
    double PrintZTransferFunction(double dts);
    double GetZTransferFunction(vector<double> &num, vector<double> &den);


private:
    //transfer function G
    double gain; //the gain for the transfer function.
    std::vector<double> numCoef,denCoef;//numerator and denominator coefficients in z positive powers.
    std::vector<double> numExps,denExps;//numerator and denominator jw  powers.

    long numOrder,denOrder;
    bool InitSystemBlock(const std::vector<double> &new_numCoef, const std::vector<double> &new_denCoef,
            const std::vector<double> &new_numExps, const std::vector<double> &new_denExps);

    //state values
    double state;
    std::vector<double> oldStates;
    std::vector<double> oldInputs;

};



#endif // SYSTEMBLOCK_H
