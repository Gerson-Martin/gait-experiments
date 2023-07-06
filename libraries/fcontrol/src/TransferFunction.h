#ifndef TRANSFERFUNCTION_H
#define TRANSFERFUNCTION_H

#include <vector>
#include <complex>
#include <iostream>

class TransferFunction
{
public:
    TransferFunction();
    TransferFunction(const std::vector<double> &new_num, const std::vector<double> &new_den);
    TransferFunction(const std::vector<double> &new_numCoef, const std::vector<double> &new_denCoef,
                const std::vector<double> &new_numExps, const std::vector<double> &new_denExps);




    std::vector<double> getNumCoef() const;

    std::vector<double> getDenCoef() const;

    std::vector<double> getNumExps() const;

    std::vector<double> getDenExps() const;

private:


    //transfer function G
    //numerator and denominator coefficients and exponents
    std::vector<double> numCoef,denCoef;
    std::vector<double> numExps,denExps;


    std::vector< std::complex<double> > G;//Processed Gain spectral values. Frequency numeric transfer function.
    std::vector<double> g;//Processed gain time values. Time numeric transfer function.




    //private functions

    bool InitTransferFunction(const std::vector<double> &new_numCoef, const std::vector<double> &new_denCoef,
                              const std::vector<double> &new_numExps, const std::vector<double> &new_denExps);


};

#endif // TRANSFERFUNCTION_H
